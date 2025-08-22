#if defined(ARDUINO_FOCSTIM_V3)

#include "bsp.h"
#include "utils.h"
#include "foc_utils.h"
#include "protobuf_api.h"

#include <Arduino.h>
#include <stm32g4xx_ll_dac.h>
#include <stm32g4xx_ll_comp.h>
#include <stm32g4xx_ll_exti.h>

#define ADC_VOLTAGE 3.3f
#define ADC_SCALE   4095
#define DAC_MAX_VALUE 4095

#define APP_TIMER_FREQ_Hz       2000000UL
#define LED_TIMER_FREQ_Hz       1000000UL
#define LED_TIMER_PWM_FREQ_Hz   1000UL
#define LED_TIMER_PEAK          (1000000UL / 1000UL)
#define RED_LED_BRIGHTNESS      1.0f
#define GREEN_LED_BRIGHTNESS    0.2f

#define MAX22213_RISEN 2000
#if MAX22213_HFS == 0
#define MAX22213_KISEN 7500     // HFS 0, max current 3A
#else
#define MAX22213_KISEN 3840     // HFS 1, max current 1.5A, higher resoution
#endif


// MAX22213 fault. Input pull-up
#define FAULT_GPIO_PORT GPIOA
#define FAULT_PIN LL_GPIO_PIN_15    // PA15

// MAX22213 sleep, active low. Output
#define SLEEP_GPIO_PORT GPIOC
#define SLEEP_PIN LL_GPIO_PIN_11    // PC11

// MAX22213 HFS. Output
#define HFS_GPIO_PORT GPIOD
#define HFS_PIN LL_GPIO_PIN_2       // PD2

// MAX22213 enable pins. Output
#define ENABLE_GPIO_PORT GPIOC
#define ENABLE_1_PIN LL_GPIO_PIN_7  // PC7
#define ENABLE_2_PIN LL_GPIO_PIN_8  // PC8
#define ENABLE_3_PIN LL_GPIO_PIN_9  // PC9
#define ENABLE_4_PIN LL_GPIO_PIN_6  // PC6

// MAX22213 DIN (direction) pins. Output via TIM1
#define DIN_GPIO_PORT GPIOA
#define DIN_1_PIN LL_GPIO_PIN_9    // PA9  AF6  tim1_ch2 "D"
#define DIN_2_PIN LL_GPIO_PIN_8    // PA8  AF6  tim1_ch1 "B"
#define DIN_3_PIN LL_GPIO_PIN_10   // PA10 AF6  tim1_ch3 "C"
#define DIN_4_PIN LL_GPIO_PIN_11   // PA11 AF11 tim1_ch4 "A"

// MAX22213 current sense pins. Analog input (opamp)
#define SEN_GPIO_PORT GPIOB
#define SEN1_PIN LL_GPIO_PIN_14    // PB14 opamp2   ADC_CHANNEL_VOPAMP2 on ADC2
#define SEN2_PIN LL_GPIO_PIN_13    // PB13 opamp3   ADC_CHANNEL_VOPAMP3_ADC3 (also available on ADC2)
#define SEN3_PIN LL_GPIO_PIN_11    // PB11 opamp4   ADC_CHANNEL_VOPAMP4 on ADC5
#define SEN4_PIN LL_GPIO_PIN_12    // PB12 opamp6   ADC_CHANNEL_VOPAMP6 on ADC4
#define SEN1_OPAMP OPAMP2
#define SEN2_OPAMP OPAMP3
#define SEN3_OPAMP OPAMP4
#define SEN4_OPAMP OPAMP6

// triac for phase 'D' enable. Output
#define OUT1_EN_GPIO_PORT GPIOB
#define OUT1_EN_PIN LL_GPIO_PIN_4  // PB4

// boost enable. Output open-drain with pullup enabled
#define BOOST_EN_GPIO_PORT GPIOB
#define BOOST_EN_PIN LL_GPIO_PIN_10 // PB10

// boost control. DAC1_OUT1
#define BOOST_DAC_GPIO_PORT GPIOA
#define BOOST_DAC_PIN LL_GPIO_PIN_4 // PA4

// VM sense, ADC2_IN12
#define VM_SENSE_GPIO_PORT GPIOB
#define VM_SENSE_PIN LL_GPIO_PIN_2  // PB2
#define ADC2_CHANNEL_VM_SENSE       ADC_CHANNEL_12

// VSYS sense, ADC2_IN15
#define VSYS_SENSE_GPIO_PORT GPIOB
#define VSYS_SENSE_PIN LL_GPIO_PIN_15 // PB15
#define ADC2_CHANNEL_VSYS_SENSE     ADC_CHANNEL_15

// VSYS comp, COMP1_INP
#define VSYS_COMP_GPIO_PORT GPIOB
#define VSYS_COMP_PIN LL_GPIO_PIN_1 // PB1
#define EXTI_LINE_VSYS_COMP         LL_EXTI_LINE_21

// Potentiometer, ADC12_IN8
#define POTENTIOMETER_GPIO_PORT GPIOC
#define POTENTIOMETER_PIN LL_GPIO_PIN_2 // PC2
#define ADC1_CHANNEL_POTENTIOMETER  ADC_CHANNEL_8

// PGOOD. Input
#define PGOOD_GPIO_PORT GPIOB
#define PGOOD_PIN LL_GPIO_PIN_5 // PB5

// Red LED. Output via TIM8_CH1 (AF5)
#define LED_RED_GPIO_PORT GPIOB
#define LED_RED_PIN LL_GPIO_PIN_6 // PB6

// Green LED. Output via TIM8_CH3n (AF4)
#define LED_GREEN_GPIO_PORT GPIOC
#define LED_GREEN_PIN LL_GPIO_PIN_12    // PC12

// TODO: usart 2


static TIM_TypeDef *const pwm_timer = TIM1;
static TIM_TypeDef *const app_timer = TIM17;
static TIM_TypeDef *const led_timer = TIM8;

static COMP_TypeDef *const vsys_comp = COMP1;
static DAC_TypeDef *const vsys_comp_dac = DAC3;
static DAC_TypeDef *const boost_control_dac = DAC1;
static USART_TypeDef *const usart2 = USART2;


struct BSP {
    BSP() {
        boost_dac_value = DAC_MAX_VALUE;
    }

    OPAMP_HandleTypeDef opamp2; // isen1
    OPAMP_HandleTypeDef opamp3; // isen2
    OPAMP_HandleTypeDef opamp4; // isen3
    OPAMP_HandleTypeDef opamp6; // isen4

    ADC_HandleTypeDef adc1;
    ADC_HandleTypeDef adc2;
    ADC_HandleTypeDef adc3;
    ADC_HandleTypeDef adc4;
    ADC_HandleTypeDef adc5;

    DMA_HandleTypeDef dma1;
    DMA_HandleTypeDef dma2;
    DMA_HandleTypeDef dma3;
    DMA_HandleTypeDef dma4;
    DMA_HandleTypeDef dma5;


    union {
        uint16_t adc1_buffer[3];
        struct {
            volatile uint16_t vrefint;
            volatile uint16_t v_ts;
            volatile uint16_t potentiometer;
        };
    };
    union {
        uint16_t adc2_buffer[3];
        struct {
            volatile uint16_t current_1;    // "D"
            volatile uint16_t vm_sense;     // TODO not trigger so often.
            volatile uint16_t vsys_sense;
        };
    };
    union {
        uint16_t adc3_buffer[1];
        struct {
            volatile uint16_t current_2;    // "B"
        };
    };
    union {
        uint16_t adc4_buffer[1];
        struct {
            volatile uint16_t current_4;    // "A"
        };
    };
    union {
        uint16_t adc5_buffer[1];
        struct {
            volatile uint16_t current_3;    // "C"
        };
    };

    std::function<void()> pwm_callback;

    volatile int vsys_high_cycles = 0;
    uint16_t boost_dac_value = 0;
    uint32_t boost_on_cycles;
    uint32_t boost_off_cycles;

    float potentiometer_filtered;
    float ts_filtered;

    uint32_t app_timer_ticks;

    LedPattern led_pattern;
    uint32_t led_pattern_progress;
};

BSP bsp = {};

static void enableInterruptWithPrio(IRQn_Type intr, int prio)
{
    NVIC_ClearPendingIRQ(intr);
    NVIC_SetPriority(intr, prio);
    NVIC_EnableIRQ(intr);
}

void initGPIO()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // TODO: bootloader pin

    // MAX22213 fault. Input pull-up
    LL_GPIO_SetPinMode(FAULT_GPIO_PORT, FAULT_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(FAULT_GPIO_PORT, FAULT_PIN, LL_GPIO_PULL_UP);

    // MAX22213 sleep, active low. Output
    LL_GPIO_SetPinMode(SLEEP_GPIO_PORT, SLEEP_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_ResetOutputPin(SLEEP_GPIO_PORT, SLEEP_PIN); // do sleep

    // MAX22213 HFS. Output
    LL_GPIO_SetPinMode(HFS_GPIO_PORT, HFS_PIN, LL_GPIO_MODE_OUTPUT);
    HAL_GPIO_WritePin(HFS_GPIO_PORT, HFS_PIN, (GPIO_PinState)MAX22213_HFS);

    // MAX22213 enable pins. Output
    LL_GPIO_SetPinMode(ENABLE_GPIO_PORT, ENABLE_1_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(ENABLE_GPIO_PORT, ENABLE_2_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(ENABLE_GPIO_PORT, ENABLE_3_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(ENABLE_GPIO_PORT, ENABLE_4_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_ResetOutputPin(ENABLE_GPIO_PORT, ENABLE_1_PIN | ENABLE_2_PIN | ENABLE_3_PIN | ENABLE_4_PIN);

    // MAX22213 DIN (direction) pins. Output via TIM1
    LL_GPIO_SetPinMode(DIN_GPIO_PORT, DIN_1_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(DIN_GPIO_PORT, DIN_2_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(DIN_GPIO_PORT, DIN_3_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(DIN_GPIO_PORT, DIN_4_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(DIN_GPIO_PORT, DIN_1_PIN, LL_GPIO_AF_6);
    LL_GPIO_SetAFPin_8_15(DIN_GPIO_PORT, DIN_2_PIN, LL_GPIO_AF_6);
    LL_GPIO_SetAFPin_8_15(DIN_GPIO_PORT, DIN_3_PIN, LL_GPIO_AF_6);
    LL_GPIO_SetAFPin_8_15(DIN_GPIO_PORT, DIN_4_PIN, LL_GPIO_AF_11);
    LL_GPIO_SetPinSpeed(DIN_GPIO_PORT, DIN_1_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed(DIN_GPIO_PORT, DIN_2_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed(DIN_GPIO_PORT, DIN_3_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed(DIN_GPIO_PORT, DIN_4_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);

    // MAX22213 current sense pins. Analog input (opamp)
    LL_GPIO_SetPinMode(SEN_GPIO_PORT, SEN1_PIN, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(SEN_GPIO_PORT, SEN2_PIN, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(SEN_GPIO_PORT, SEN3_PIN, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(SEN_GPIO_PORT, SEN4_PIN, LL_GPIO_MODE_ANALOG);

    // triac for phase 'D' enable. Output
    LL_GPIO_SetPinMode(OUT1_EN_GPIO_PORT, OUT1_EN_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_ResetOutputPin(OUT1_EN_GPIO_PORT, OUT1_EN_PIN); // disable triac

    // boost enable. Output open-drain with pullup enabled
    LL_GPIO_SetPinMode(BOOST_EN_GPIO_PORT, BOOST_EN_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(BOOST_EN_GPIO_PORT, BOOST_EN_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(BOOST_EN_GPIO_PORT, BOOST_EN_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_PORT, BOOST_EN_PIN);

    // boost control. DAC1_OUT1
    LL_GPIO_SetPinMode(BOOST_DAC_GPIO_PORT, BOOST_DAC_PIN, LL_GPIO_MODE_ANALOG);

    // VM sense, ADC2_IN12
    LL_GPIO_SetPinMode(VM_SENSE_GPIO_PORT, VM_SENSE_PIN, LL_GPIO_MODE_ANALOG);

    // VSYS sense, ADC2_IN15
    LL_GPIO_SetPinMode(VSYS_SENSE_GPIO_PORT, VSYS_SENSE_PIN, LL_GPIO_MODE_ANALOG);

    // VSYS comp, COMP1_INP
    LL_GPIO_SetPinMode(VSYS_COMP_GPIO_PORT, VSYS_COMP_PIN, LL_GPIO_MODE_ANALOG);

    // Potentiometer, ADC12_IN8
    LL_GPIO_SetPinMode(POTENTIOMETER_GPIO_PORT, POTENTIOMETER_PIN, LL_GPIO_MODE_ANALOG);

    // PGOOD. Input
    LL_GPIO_SetPinMode(PGOOD_GPIO_PORT, PGOOD_PIN, LL_GPIO_MODE_INPUT);

    // Red LED. Output via TIM8_CH1 (AF5)
    LL_GPIO_SetPinMode(LED_RED_GPIO_PORT, LED_RED_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(LED_RED_GPIO_PORT, LED_RED_PIN, LL_GPIO_AF_5);

    // Green LED. Output via TIM8_CH3n (AF4)
    LL_GPIO_SetPinMode(LED_GREEN_GPIO_PORT, LED_GREEN_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(LED_GREEN_GPIO_PORT, LED_GREEN_PIN, LL_GPIO_AF_4);

    // default states
    BSP_OutputEnable(false, false, false, false);
}

void initPwm()
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    // configure timer frequency
    uint32_t overflow = SystemCoreClock / (STIM_PWM_FREQ * 2);
    uint32_t period = overflow / 0x10000 + 1;
    pwm_timer->PSC = period - 1;
    pwm_timer->ARR = overflow / period - 1;

    // pwm mode, update event every full period
    pwm_timer->RCR = 1;
    pwm_timer->CR1 |= TIM_COUNTERMODE_CENTERALIGNED3;

    // pwm mode 1 and preload enable
    pwm_timer->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    pwm_timer->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    pwm_timer->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
    pwm_timer->CCMR2 |= TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;

    // enable outputs
    pwm_timer->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // only generate interrupt on timer overflow, not EGR
    pwm_timer->CR1  |= TIM_CR1_URS;

    // TODO: test and implement
    // enable break source
    // pulse_timer->BDTR = TIM_BDTR_BK2E;

    // enable update interrupt
    pwm_timer->DIER |= TIM_DIER_UIE;

    // force outputs low after break event.
    pwm_timer->BDTR |= TIM_BDTR_OSSI;

    // enable ADC trigger on update event.
    pwm_timer->CR2 |= LL_TIM_TRGO_UPDATE;

    // main output enable
    pwm_timer->BDTR |= TIM_BDTR_MOE;

    enableInterruptWithPrio(TIM1_UP_TIM16_IRQn, 0);
    // TIM1_BRK_TIM15_IRQn --> TIM1_BRK_TIM15_IRQHandler
}

void initAppTimer()
{
    __HAL_RCC_TIM17_CLK_ENABLE();

    float ms = 1.0f;
    app_timer->PSC = SystemCoreClock / APP_TIMER_FREQ_Hz - 1;
    app_timer->ARR = uint16_t(APP_TIMER_FREQ_Hz * (ms / 1000));
    app_timer->DIER |= TIM_DIER_UIE;            // Interrupt on timer update
    app_timer->EGR |= TIM_EGR_UG;               // Force update of the shadow registers.
    app_timer->CR1 = TIM_CR1_CEN;               // Enable the counter.
    enableInterruptWithPrio(TIM1_TRG_COM_TIM17_IRQn, 1);
}

void initLedTimer()
{
    __HAL_RCC_TIM8_CLK_ENABLE();

    led_timer->PSC = SystemCoreClock / LED_TIMER_FREQ_Hz - 1;
    led_timer->ARR = uint16_t(LED_TIMER_PEAK);

    led_timer->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;    // pwm mode
    led_timer->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;

    led_timer->CCR1 = 0;    // leds off
    led_timer->CCR3 = 0;

    led_timer->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE;  // enable output 1 and 3n

    led_timer->EGR |= TIM_EGR_UG;               // Force update of the shadow registers.
    led_timer->CR1 = TIM_CR1_CEN;               // Enable the counter.
    led_timer->BDTR |= TIM_BDTR_MOE;            // main output enable
}

void configureOpamp(OPAMP_HandleTypeDef *hopamp, OPAMP_TypeDef *OPAMPx_Def)
{
    hopamp->Instance = OPAMPx_Def;
    hopamp->Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
    hopamp->Init.Mode = OPAMP_PGA_MODE;
    if (hopamp->Instance == OPAMP2)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;   // PB14 = ISEN1
    }
    if (hopamp->Instance == OPAMP3)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;   // PB13 = ISEN2
    }
    if (hopamp->Instance == OPAMP4)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;   // PB11 = ISEN3
    }
    if (hopamp->Instance == OPAMP6)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;   // PB12 = ISEN4
    }
    hopamp->Init.InternalOutput = ENABLE;
    hopamp->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
    hopamp->Init.PgaGain = OPAMP_PGA_GAIN_4_OR_MINUS_3;
    hopamp->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

    HAL_StatusTypeDef status;
    status = HAL_OPAMP_Init(hopamp);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("opamp init failed %i\n", status);
        Error_Handler();
    }

    status = HAL_OPAMP_Start(hopamp);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("opamp start failed %i\n", status);
        Error_Handler();
    }
}

void initOpamp()
{
    configureOpamp(&bsp.opamp2, OPAMP2);
    configureOpamp(&bsp.opamp3, OPAMP3);
    configureOpamp(&bsp.opamp4, OPAMP4);
    configureOpamp(&bsp.opamp6, OPAMP6);
}

void configureADC1(ADC_HandleTypeDef *hadc)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /** Common config
     */
    hadc->Instance = ADC1;
    hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.GainCompensation = 0;
    hadc->Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait = DISABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.NbrOfConversion = 3;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc->Init.DMAContinuousRequests = ENABLE;
    hadc->Init.Overrun = ADC_OVR_DATA_PRESERVED;

    int status;
    if ((status = HAL_ADC_Init(hadc)) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_Init failed! %i", status);
        Error_Handler();
    }
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    // vrefint
    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5; // ~6µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    // internal stm32 temp sensor
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5; // ~6µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    // potentiometer
    sConfig.Channel = ADC1_CHANNEL_POTENTIOMETER;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5; // ~6µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
}

void configureADC2(ADC_HandleTypeDef *hadc)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /** Common config
     */
    hadc->Instance = ADC2;
    hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.GainCompensation = 0;
    hadc->Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait = DISABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.NbrOfConversion = 3;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc->Init.DMAContinuousRequests = ENABLE;
    hadc->Init.Overrun = ADC_OVR_DATA_PRESERVED;

    if (HAL_ADC_Init(hadc) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_Init failed!\n");
        Error_Handler();
    }
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    // ISEN1
    sConfig.Channel = ADC_CHANNEL_VOPAMP2;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; // ~0.54µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    // VM_SENSE
    sConfig.Channel = ADC2_CHANNEL_VM_SENSE;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5; // ~0.8µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    // VSYS_sense
    sConfig.Channel = ADC2_CHANNEL_VSYS_SENSE;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5; // ~0.8µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
}

void configureADC3(ADC_HandleTypeDef *hadc)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /** Common config
     */
    hadc->Instance = ADC3;
    hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.GainCompensation = 0;
    hadc->Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait = DISABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.NbrOfConversion = 1;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc->Init.DMAContinuousRequests = ENABLE;
    hadc->Init.Overrun = ADC_OVR_DATA_PRESERVED;

    if (HAL_ADC_Init(hadc) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_Init failed!\n");
        Error_Handler();
    }
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    // ISEN2
    sConfig.Channel = ADC_CHANNEL_VOPAMP3_ADC3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; // ~0.54µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
}

void configureADC4(ADC_HandleTypeDef *hadc)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /** Common config
     */
    hadc->Instance = ADC4;
    hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.GainCompensation = 0;
    hadc->Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait = DISABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.NbrOfConversion = 1;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc->Init.DMAContinuousRequests = ENABLE;
    hadc->Init.Overrun = ADC_OVR_DATA_PRESERVED;

    if (HAL_ADC_Init(hadc) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_Init failed!\n");
        Error_Handler();
    }
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    // ISEN4
    sConfig.Channel = ADC_CHANNEL_VOPAMP6;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; // ~0.54µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
}

void configureADC5(ADC_HandleTypeDef *hadc)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /** Common config
     */
    hadc->Instance = ADC5;
    hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.GainCompensation = 0;
    hadc->Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait = DISABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.NbrOfConversion = 1;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc->Init.DMAContinuousRequests = ENABLE;
    hadc->Init.Overrun = ADC_OVR_DATA_PRESERVED;

    if (HAL_ADC_Init(hadc) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_Init failed!\n");
        Error_Handler();
    }
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    // ISEN3
    sConfig.Channel = ADC_CHANNEL_VOPAMP4; // ISEN3
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; // ~0.54µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
}

void initADC()
{
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC345_CLK_ENABLE();

    // Configure clock on ADC12 and ADC345
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC345;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;     // 170Mhz
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_PLL;   // 170Mhz
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    configureADC1(&bsp.adc1);
    configureADC2(&bsp.adc2);
    configureADC3(&bsp.adc3);
    configureADC4(&bsp.adc4);
    configureADC5(&bsp.adc5);

    // Calibrate the ADCs
    HAL_StatusTypeDef status;
    status = HAL_ADCEx_Calibration_Start(&bsp.adc1, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("ADC calibration failed: %i\n", status);
        Error_Handler();
    }
    status = HAL_ADCEx_Calibration_Start(&bsp.adc2, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("ADC calibration failed: %i\n", status);
        Error_Handler();
    }
    status = HAL_ADCEx_Calibration_Start(&bsp.adc3, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("ADC calibration failed: %i\n", status);
        Error_Handler();
    }
    status = HAL_ADCEx_Calibration_Start(&bsp.adc4, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("ADC calibration failed: %i\n", status);
        Error_Handler();
    }
    status = HAL_ADCEx_Calibration_Start(&bsp.adc5, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("ADC calibration failed: %i\n", status);
        Error_Handler();
    }
}

void configureDMA(ADC_HandleTypeDef *hadc, DMA_HandleTypeDef *hdma_adc, DMA_Channel_TypeDef *channel, uint32_t request)
{
    hdma_adc->Instance = channel;
    hdma_adc->Init.Request = request;
    hdma_adc->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc->Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc->Init.Mode = DMA_CIRCULAR;
    hdma_adc->Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_DeInit(hdma_adc);
    if (HAL_DMA_Init(hdma_adc) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_DMA_Init failed!\n");
        Error_Handler();
    }
    __HAL_LINKDMA(hadc, DMA_Handle, *hdma_adc);
}

void initDMA()
{
    /* DMA controller clock enable */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    configureDMA(&bsp.adc1, &bsp.dma1, DMA1_Channel1, DMA_REQUEST_ADC1);
    configureDMA(&bsp.adc2, &bsp.dma2, DMA1_Channel2, DMA_REQUEST_ADC2);
    configureDMA(&bsp.adc3, &bsp.dma3, DMA1_Channel3, DMA_REQUEST_ADC3);
    configureDMA(&bsp.adc4, &bsp.dma4, DMA1_Channel4, DMA_REQUEST_ADC4);
    configureDMA(&bsp.adc5, &bsp.dma5, DMA1_Channel5, DMA_REQUEST_ADC5);


    HAL_StatusTypeDef status;
    status = HAL_ADC_Start_DMA(&bsp.adc1, (uint32_t *)bsp.adc1_buffer, sizeof(bsp.adc1_buffer) / 2);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("DMA start adc1 failed, %i\n", status);
        Error_Handler();
    }

    status = HAL_ADC_Start_DMA(&bsp.adc2, (uint32_t *)bsp.adc2_buffer, sizeof(bsp.adc2_buffer) / 2);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("DMA start adc2 failed, %i\n", status);
        Error_Handler();
    }

    status = HAL_ADC_Start_DMA(&bsp.adc3, (uint32_t *)bsp.adc3_buffer, sizeof(bsp.adc3_buffer) / 2);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("DMA start adc3 failed, %i\n", status);
        Error_Handler();
    }

    status = HAL_ADC_Start_DMA(&bsp.adc4, (uint32_t *)bsp.adc4_buffer, sizeof(bsp.adc4_buffer) / 2);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("DMA start adc4 failed, %i\n", status);
        Error_Handler();
    }

    status = HAL_ADC_Start_DMA(&bsp.adc5, (uint32_t *)bsp.adc5_buffer, sizeof(bsp.adc5_buffer) / 2);
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("DMA start adc5 failed, %i\n", status);
        Error_Handler();
    }

    /* DMA interrupt init */
    // enableInterruptWithPrio(DMA1_Channel1_IRQn, 0);
    // enableInterruptWithPrio(DMA1_Channel2_IRQn, 0);
    // enableInterruptWithPrio(DMA1_Channel3_IRQn, 0);
    // enableInterruptWithPrio(DMA1_Channel4_IRQn, 0);
    enableInterruptWithPrio(DMA1_Channel5_IRQn, 0);
    DMA1_Channel5->CCR &= ~DMA_CCR_HTIE;    // enable only transfer complete interrupt.
}

void initDAC()
{
    __HAL_RCC_DAC1_CLK_ENABLE();
    __HAL_RCC_DAC3_CLK_ENABLE();

    // DAC1_CH1, used for VMCONT
    LL_DAC_InitTypeDef DAC_InitStruct = {
        .TriggerSource = LL_DAC_TRIG_SOFTWARE,
        .WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE,
        .OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE,
        .OutputConnection = LL_DAC_OUTPUT_CONNECT_GPIO,
        .OutputMode = LL_DAC_OUTPUT_MODE_NORMAL
    };
    ErrorStatus status;
    status = LL_DAC_Init(boost_control_dac, LL_DAC_CHANNEL_1, &DAC_InitStruct);
    if (status != ErrorStatus::SUCCESS)
    {
        BSP_PrintDebugMsg("DAC initialization failed: %i\n", status);
        Error_Handler();
    }
    LL_DAC_Enable(boost_control_dac, LL_DAC_CHANNEL_1);
    // while (boost_control_dac->SR & DAC_SR_DAC1RDY) {}    // wait until DAC is ready


    // DAC3_CH1, used for COMP1
    DAC_InitStruct = LL_DAC_InitTypeDef{
        .TriggerSource = LL_DAC_TRIG_SOFTWARE,
        .WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE,
        .OutputBuffer = LL_DAC_OUTPUT_BUFFER_DISABLE,
        .OutputConnection = LL_DAC_OUTPUT_CONNECT_INTERNAL,
        .OutputMode = LL_DAC_OUTPUT_MODE_NORMAL
    };
    status = LL_DAC_Init(vsys_comp_dac, LL_DAC_CHANNEL_1, &DAC_InitStruct);
    if (status != ErrorStatus::SUCCESS)
    {
        BSP_PrintDebugMsg("DAC initialization failed: %i\n", status);
        Error_Handler();
    }
    LL_DAC_Enable(vsys_comp_dac, LL_DAC_CHANNEL_1);
    // while (vsys_comp_dac->SR & DAC_SR_DAC1RDY) {}    // wait until DAC is ready

    boost_control_dac->DHR12R1 = DAC_MAX_VALUE;
    vsys_comp_dac->DHR12R1 = DAC_MAX_VALUE;
}

void initCOMP()
{
    LL_COMP_InitTypeDef COMP_InitStruct = {
        .InputPlus = LL_COMP_INPUT_PLUS_IO2,        // PB1
        .InputMinus = LL_COMP_INPUT_MINUS_DAC3_CH1,
        .InputHysteresis = LL_COMP_HYSTERESIS_NONE,
        .OutputPolarity = LL_COMP_OUTPUT_LEVEL_HIGH,
        .OutputBlankingSource = LL_COMP_BLANKINGSRC_NONE
    };

    LL_COMP_Init(vsys_comp, &COMP_InitStruct);
    vsys_comp->CSR |= COMP_CSR_EN;
}

void initEXTI()
{
    // COMP1
    LL_EXTI_InitTypeDef EXTI_InitStruct = {
        .Line_0_31 = EXTI_LINE_VSYS_COMP,
        .LineCommand = ENABLE,
        .Mode = LL_EXTI_MODE_IT,
        .Trigger = LL_EXTI_TRIGGER_FALLING
    };
    LL_EXTI_Init(&EXTI_InitStruct);

    enableInterruptWithPrio(COMP1_2_3_IRQn, 0);
}



void BSP_Init()
{
    initGPIO();
    initPwm();
    initADC();
    initOpamp();
    initDMA();
    initDAC();
    initCOMP();
    initEXTI();
    initAppTimer();
    initLedTimer();

    // enable timer. DIR bit aligns timer update event with either peak or through.
    pwm_timer->CR1 |= TIM_CR1_CEN;
}

extern "C"
{
    // DMA interrupt, triggers right after all current sense reads
    // are finished.
    void DMA1_Channel5_IRQHandler(void)
    {
        if (DMA1->ISR | DMA_ISR_TCIF5) {
            if (bsp.pwm_callback) {
                bsp.pwm_callback();
            }

            DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5;
        } else {
            DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CHTIF5;
        }
    }

    // Comparator interrupt. Triggers when VSYS falls below the threshold
    void COMP1_2_3_IRQHandler(void)
    {
        if (EXTI->PR1 & EXTI_LINE_VSYS_COMP) {
            EXTI->PR1 = EXTI_LINE_VSYS_COMP;
            EXTI->EMR1 |= EXTI_LINE_VSYS_COMP;  // mask interrupt to save cycles

            // disable the boost by driving the fb voltage
            // as high as possible.
            boost_control_dac->DHR12R1 = DAC_MAX_VALUE;
            bsp.vsys_high_cycles = 0;
        }
    }

    // tim1 peak interrupt, do some bookkeeping..
    void TIM1_UP_TIM16_IRQHandler(void)
    {
        pwm_timer->SR &= ~TIM_SR_UIF;

        // DEBUG
        if (boost_control_dac->DHR12R1 == DAC_MAX_VALUE) {
            bsp.boost_off_cycles++;
        } else {
            bsp.boost_on_cycles++;
        }

        // count number of cycles VSYS is above the undervoltage threshold
        bool vsys_high = bool(vsys_comp->CSR & COMP_CSR_VALUE_Msk);
        bool interrupt_pending = bool(EXTI->PR1 & EXTI_LINE_VSYS_COMP);
        if (vsys_high && !interrupt_pending) {
            bsp.vsys_high_cycles++;
        } else {
            bsp.vsys_high_cycles = 0;
        }

        // enable boost if VSYS is persistently above the undervoltage threshold
        if (bsp.vsys_high_cycles >= 2) {
            boost_control_dac->DHR12R1 = bsp.boost_dac_value;
            EXTI->EMR1 &= ~EXTI_LINE_VSYS_COMP;  // enable comp interrupt
        }

    }

    void TIM1_TRG_COM_TIM17_IRQHandler(void)
    {
        if (app_timer->SR & TIM_SR_UIF) {         // Update
            app_timer->SR &= ~TIM_SR_UIF;         // Clear the interrupt.

            // average potentiometer value
            bsp.potentiometer_filtered += (float(bsp.potentiometer) - bsp.potentiometer_filtered) * 0.05f;
            // average temperature value -- filters out weird spikes
            bsp.ts_filtered += (float(bsp.v_ts) - bsp.ts_filtered) * 0.05f;

            // REMOVE ME
            bsp.app_timer_ticks++;

            // process LED pattern
            switch (bsp.led_pattern) {
            case Idle:
                bsp.led_pattern_progress = (bsp.led_pattern_progress + 1) % 2048;
                BSP_WriteRedLedBrightness(bsp.led_pattern_progress < 100 ? .5 : .1);
                BSP_WriteGreenLedBrightness(0);
            break;
                case Error:
                bsp.led_pattern_progress = (bsp.led_pattern_progress + 1) % 256;
                BSP_WriteRedLedBrightness(bsp.led_pattern_progress < 100 ? .1 : 1);
                BSP_WriteGreenLedBrightness(0);
            break;
            case PlayingVeryLow:
                BSP_WriteRedLedBrightness(.1);
                BSP_WriteGreenLedBrightness(.1);
            break;
            case PlayingLow:
                bsp.led_pattern_progress = (bsp.led_pattern_progress + 1) % 2048;
                BSP_WriteRedLedBrightness(0);
                BSP_WriteGreenLedBrightness(bsp.led_pattern_progress < 100 ? 1 : .2);
            break;
            case PlayingMedium:
                bsp.led_pattern_progress = (bsp.led_pattern_progress + 1) % 1024;
                BSP_WriteRedLedBrightness(0);
                BSP_WriteGreenLedBrightness(bsp.led_pattern_progress < 100 ? 1 : .2);
            break;
            case PlayingHigh:
                bsp.led_pattern_progress = (bsp.led_pattern_progress + 1) % 512;
                BSP_WriteRedLedBrightness(0);
                BSP_WriteGreenLedBrightness(bsp.led_pattern_progress < 100 ? 1 : .2);
            break;
            }
        }
    }
}

void BSP_OutputEnable(bool a, bool b, bool c, bool d)
{
    // BSP_PrintDebugMsg("output enable: %i %i %i %i", a, b, c, d);
    HAL_GPIO_WritePin(ENABLE_GPIO_PORT, ENABLE_1_PIN, (GPIO_PinState)d);
    HAL_GPIO_WritePin(ENABLE_GPIO_PORT, ENABLE_2_PIN, (GPIO_PinState)b);
    HAL_GPIO_WritePin(ENABLE_GPIO_PORT, ENABLE_3_PIN, (GPIO_PinState)c);
    HAL_GPIO_WritePin(ENABLE_GPIO_PORT, ENABLE_4_PIN, (GPIO_PinState)a);
    HAL_GPIO_WritePin(OUT1_EN_GPIO_PORT, OUT1_EN_PIN, (GPIO_PinState)d);

    if (a || b || c || d) {
        LL_GPIO_SetOutputPin(SLEEP_GPIO_PORT, SLEEP_PIN);
        pwm_timer->BDTR |= TIM_BDTR_MOE;
    } else {
        LL_GPIO_ResetOutputPin(SLEEP_GPIO_PORT, SLEEP_PIN);
        pwm_timer->BDTR &= ~(TIM_BDTR_MOE);
    }
}

void BSP_OutputEnable(bool a, bool b, bool c)
{
    BSP_OutputEnable(a, b, c, false);
}

void BSP_DisableOutputs() {
    BSP_OutputEnable(false, false, false, false);
}

void BSP_AttachPWMInterrupt(std::function<void()> fn)
{
    std::function<void()> tmp;
    __disable_irq();
    __DSB();
    __ISB();
    std::swap(bsp.pwm_callback, tmp);
    std::swap(bsp.pwm_callback, fn);
    __DSB();
    __ISB();
    __enable_irq();

    // tmp destructor called about here
}

void BSP_SetPWM3(float a, float b, float c)
{
    uint32_t arr = pwm_timer->ARR;
    uint32_t ccr1 = constrain(b, 0, 1) * arr;
    uint32_t ccr3 = constrain(c, 0, 1) * arr;
    uint32_t ccr4 = constrain(a, 0, 1) * arr;
    pwm_timer->CCR1 = ccr1; // B
    pwm_timer->CCR3 = ccr3; // C
    pwm_timer->CCR4 = ccr4; // A
}

void BSP_SetPWM3Atomic(float a, float b, float c)
{
    SET_BIT(pwm_timer->CR1, TIM_CR1_UDIS_Msk);
    BSP_SetPWM3(a, b, c);
    CLEAR_BIT(pwm_timer->CR1, TIM_CR1_UDIS_Msk);
}

void BSP_SetPWM4(float a, float b, float c, float d)
{
    uint32_t arr = pwm_timer->ARR;
    uint32_t ccr1 = constrain(b, 0, 1) * arr;
    uint32_t ccr2 = constrain(d, 0, 1) * arr;
    uint32_t ccr3 = constrain(c, 0, 1) * arr;
    uint32_t ccr4 = constrain(a, 0, 1) * arr;
    pwm_timer->CCR1 = ccr1; // B
    pwm_timer->CCR2 = ccr2; // D
    pwm_timer->CCR3 = ccr3; // C
    pwm_timer->CCR4 = ccr4; // A
}

void BSP_SetPWM4Atomic(float a, float b, float c, float d)
{
    SET_BIT(pwm_timer->CR1, TIM_CR1_UDIS_Msk);
    BSP_SetPWM4(a, b, c, d);
    CLEAR_BIT(pwm_timer->CR1, TIM_CR1_UDIS_Msk);
}

Vec3f BSP_ReadPhaseCurrents3()
{
    float gain = 4;
    float factor = ADC_VOLTAGE / ADC_SCALE / MAX22213_RISEN * MAX22213_KISEN / -gain;

    return Vec3f(
        bsp.current_4 * factor,
        bsp.current_2 * factor,
        bsp.current_3 * factor);
}

Vec4f BSP_ReadPhaseCurrents4()
{
    float gain = 4;
    float factor = ADC_VOLTAGE / ADC_SCALE / MAX22213_RISEN * MAX22213_KISEN / -gain;

    return Vec4f(
        bsp.current_4 * factor,
        bsp.current_2 * factor,
        bsp.current_3 * factor,
        bsp.current_1 * factor);
}

bool BSP_ReadFault()
{
    // TODO: use in software.
    return LL_GPIO_IsInputPinSet(FAULT_GPIO_PORT, FAULT_PIN);
}

void BSP_WriteStatusLED(bool on)
{

}

void BSP_WriteFaultLED(bool on)
{

}

float BSP_ReadTemperatureSTM()
{
    // float ts_data = bsp.v_ts * (ADC_VOLTAGE / TEMPSENSOR_CAL_VREFANALOG * 1000);
    float ts_data = bsp.ts_filtered * (ADC_VOLTAGE / TEMPSENSOR_CAL_VREFANALOG * 1000);
    float offset = TEMPSENSOR_CAL1_TEMP;
    float slope = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / float((*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR));
    float temp = offset + (ts_data - *TEMPSENSOR_CAL1_ADDR) * slope;
    return temp;
}

float BSP_ReadPotentiometerPercentage()
{
    float value = inverse_lerp(bsp.potentiometer_filtered, POTMETER_ZERO_PERCENT_VALUE, POTMETER_HUNDRED_PERCENT_VALUE);
    return _constrain(value, 0.f, 1.f);
}

float BSP_ReadTemperatureOnboardNTC()
{
    return 0;   // not supported
}

void BSP_AdjustCurrentSenseOffsets()
{
    // not needed on this board.
}

float BSP_ReadVBus()
{
    float adc_voltage = bsp.vm_sense * (ADC_VOLTAGE / ADC_SCALE);
    const float multiplier = (20 + 200) / 20.f;
    return adc_voltage * multiplier;
}

float BSP_ReadVSYS()
{
    float adc_voltage = bsp.vsys_sense * (ADC_VOLTAGE / ADC_SCALE);
    const float multiplier = (100 + 100) / 100.f;
    return adc_voltage * multiplier;
}

float BSP_ReadChipAnalogVoltage()
{
    return __LL_ADC_CALC_VREFANALOG_VOLTAGE(bsp.vrefint, LL_ADC_RESOLUTION_12B) * 0.001f;
}

void BSP_SetBoostEnable(bool enable)
{
    HAL_GPIO_WritePin(BOOST_EN_GPIO_PORT, BOOST_EN_PIN, (GPIO_PinState)enable);
}

void BSP_SetBoostVoltage(float boost_voltage)
{
    // min 0.21v
    // max 30.21v

    float fb = 1.229f;
    float rtop = 1000000;
    float rbot = 69000;
    float rdac = 110000;
    float vdac = (fb * (1 + rtop / rbot + rtop / rdac) - boost_voltage) * (rdac / rtop);
    // BSP_PrintDebugMsg("VDAC: %f", vdac);
    int value = min(DAC_MAX_VALUE, max(0, int(vdac * (DAC_MAX_VALUE / ADC_VOLTAGE))));

    // value to be written to the DAC on next interrupt.
    bsp.boost_dac_value = value;
}

void BSP_SetBoostMinimumInputVoltage(float voltage)
{
    vsys_comp_dac->DHR12R1 = (voltage / 2) * (ADC_SCALE / ADC_VOLTAGE);
}

float BSP_BoostDutyCycle()
{
    float duty = float(bsp.boost_on_cycles) / float(bsp.boost_on_cycles + bsp.boost_off_cycles);
    bsp.boost_on_cycles = 0;
    bsp.boost_off_cycles = 0;
    return duty;
}

int BSP_AppTimerTicks() {
    return bsp.app_timer_ticks;
}

void BSP_WriteGreenLedBrightness(float a)
{
    led_timer->CCR3 = a * (LED_TIMER_PEAK * GREEN_LED_BRIGHTNESS);
}

void BSP_WriteRedLedBrightness(float a)
{
    led_timer->CCR1 = a * (LED_TIMER_PEAK * RED_LED_BRIGHTNESS);
}

void BSP_WriteLedPattern(LedPattern pattern)
{
    bsp.led_pattern = pattern;
}

bool BSP_ReadPGood()
{
    return LL_GPIO_IsInputPinSet(PGOOD_GPIO_PORT, PGOOD_PIN);
}

void BSP_PrintDebugMsg(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    g_protobuf->transmit_notification_debug_string(fmt, args);
    va_end(args);
}

#endif