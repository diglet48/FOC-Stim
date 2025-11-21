#if defined(BOARD_FOCSTIM_V4)

#include "bsp.h"
#include "utils.h"
#include "foc_utils.h"
#include "protobuf_api.h"

#include <stm32g4xx_ll_dac.h>
#include <stm32g4xx_ll_comp.h>
#include <stm32g4xx_ll_exti.h>


#define ADC_VOLTAGE 2.9f
#define ADC_SCALE   4095
#define DAC_MAX_VALUE 4095
#define ENCODER_100_PERCENT_VALUE   (24 * 4 * 2)    // 2 Rotations
#define ENCODER_INIT_CNT            0

#define LED_TIMER_FREQ_Hz       1'000'000UL
#define LED_TIMER_PWM_FREQ_Hz   1000UL
#define LED_TIMER_PEAK          (LED_TIMER_FREQ_Hz / LED_TIMER_PWM_FREQ_Hz)
#define RED_LED_BRIGHTNESS      1.0f

#define DRIVER_RISEN 1000
#define DRIVER_KISEN 1500 // µa / A
#define DRIVER_SENSE_CURENT_TO_COIL_CURRENT (1e6f / DRIVER_KISEN)


#define ADC_SAMPLETIME_CURRENT_SENSE    ADC_SAMPLETIME_12CYCLES_5   // [ 12.5 + 12.5] clocks = 0.58µs
#define ADC_SAMPLETIME_VREFINT          ADC_SAMPLETIME_247CYCLES_5  // [247.5 + 12.5] clocks = 6.11µs   (datasheet: min 4µs)
#define ADC_SAMPLETIME_V_TS             ADC_SAMPLETIME_247CYCLES_5  // [247.5 + 12.5] clocks = 6.11µs   (datasheet: min 5µs)
#define ADC_SAMPLETIME_VBAT             ADC_SAMPLETIME_640CYCLES_5  // [640.5 + 12.5] clocks = 15.4µs   (datasheet: min 12µs)
#define ADC_SAMPLETIME_VM               ADC_SAMPLETIME_247CYCLES_5  // [247.5 + 12.5] clocks = 6.11µs
#define ADC_SAMPLETIME_VSYS             ADC_SAMPLETIME_247CYCLES_5  // [247.5 + 12.5] clocks = 6.11µs


// Drivers EN1 (enable) pins. Output
#define DRIVER_ENABLE_GPIO_PORT GPIOC
#define DRIVER_A_ENABLE_PIN LL_GPIO_PIN_7  // PC7
#define DRIVER_B_ENABLE_PIN LL_GPIO_PIN_9  // PC9
#define DRIVER_C_ENABLE_PIN LL_GPIO_PIN_8  // PC8
#define DRIVER_D_ENABLE_PIN LL_GPIO_PIN_6  // PC6

// Drivers EN2 (direction) pins. Output via TIM1
#define DRIVER_DIN_GPIO_PORT GPIOA
#define DRIVER_A_DIN_PIN LL_GPIO_PIN_10   // PA10 DIN4 AF6  tim1_ch3
#define DRIVER_B_DIN_PIN LL_GPIO_PIN_11   // PA11 DIN1 AF11 tim1_ch4
#define DRIVER_C_DIN_PIN LL_GPIO_PIN_8    // PA8  DIN2 AF6  tim1_ch1
#define DRIVER_D_DIN_PIN LL_GPIO_PIN_9    // PA9  DIN3 AF6  tim1_ch2

// Driver IPROPI current sense pins. Analog input (opamp)
#define SEN_GPIO_PORT GPIOB
#define SEN_A_PIN LL_GPIO_PIN_12    // PB12 opamp6   ADC_CHANNEL_VOPAMP6 on ADC4
#define SEN_B_PIN LL_GPIO_PIN_14    // PB14 opamp2   ADC_CHANNEL_VOPAMP2 on ADC2
#define SEN_C_PIN LL_GPIO_PIN_13    // PB13 opamp3   ADC_CHANNEL_VOPAMP3_ADC3 (also available on ADC2)
#define SEN_D_PIN LL_GPIO_PIN_11    // PB11 opamp4   ADC_CHANNEL_VOPAMP4 on ADC5

// triac for phase 'D' enable. Output
#define OUT_D_EN_GPIO_PORT GPIOB
#define OUT_D_EN_PIN LL_GPIO_PIN_4   // PB4

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

// VSYS sense, ADC2_IN15 and ADC4_IN5
#define VSYS_SENSE_GPIO_PORT GPIOB
#define VSYS_SENSE_PIN LL_GPIO_PIN_15 // PB15
#define ADC4_CHANNEL_VSYS_SENSE     ADC_CHANNEL_5

// VSYS comp, COMP1_INP
#define VSYS_COMP_GPIO_PORT GPIOB
#define VSYS_COMP_PIN LL_GPIO_PIN_1 // PB1
#define EXTI_LINE_VSYS_COMP         LL_EXTI_LINE_21

// PGOOD. Input
#define PGOOD_GPIO_PORT GPIOF
#define PGOOD_PIN LL_GPIO_PIN_1 // PF1-OSC_OUT

// Fault LED. Output via TIM20_CH2 (AF6)
#define LED_FAULT_GPIO_PORT GPIOC
#define LED_FAULT_PIN LL_GPIO_PIN_2 // PC2

// Frontpanel button (input pullup) / encoder (input)
#define ENCODER_GPIO_PORT GPIOA
#define ENCODER_BUTTON_PIN LL_GPIO_PIN_5 // PA5
#define ENCODER_A_PIN LL_GPIO_PIN_6      // PA6 TIM3_CH1 (AF2)
#define ENCODER_B_PIN LL_GPIO_PIN_7      // PA7 TIM3_CH2 (AF2)

// STM32 sleep pin to esp32. Input (TOOD: pullup/down?)
#define STM32_SLEEP_GPIO_PORT GPIOB
#define STM32_SLEEP_PIN LL_GPIO_PIN_7    // PB7




static TIM_TypeDef *const pwm_timer = TIM1;
static TIM_TypeDef *const adc_trigger_timer = TIM2;
static TIM_TypeDef *const encoder_timer = TIM3;
static TIM_TypeDef *const app_and_led_timer = TIM20;

static COMP_TypeDef *const vsys_comp = COMP1;
static DAC_TypeDef *const vsys_comp_dac = DAC3;
static DAC_TypeDef *const boost_control_dac = DAC1;
static USART_TypeDef *const usart2 = USART2;


struct BSP {
    BSP() {
        boost_dac_value = DAC_MAX_VALUE;
    }

    OPAMP_HandleTypeDef opamp2; // current b
    OPAMP_HandleTypeDef opamp3; // current c
    OPAMP_HandleTypeDef opamp4; // current a
    OPAMP_HandleTypeDef opamp6; // current d

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
        uint16_t adc1_buffer[2];
        struct {
            volatile uint16_t vrefint;
            volatile uint16_t v_ts;
            // TODO: vbat?
        };
    };
    union {
        uint16_t adc2_buffer[2];
        struct {
            volatile uint16_t current_b;
            volatile uint16_t vm_sense;
        };
    };
    union {
        uint16_t adc3_buffer[1];
        struct {
            volatile uint16_t current_c;
        };
    };
    union {
        uint16_t adc4_buffer[2];
        struct {
            volatile uint16_t current_a;
            volatile uint16_t vsys_sense;
        };
    };
    union {
        uint16_t adc5_buffer[1];
        struct {
            volatile uint16_t current_d;
        };
    };

    std::function<void()> pwm_callback;

    volatile int vsys_high_cycles = 0;
    uint16_t boost_dac_value = 0;
    uint32_t boost_on_cycles;
    uint32_t boost_off_cycles;

    uint16_t vsys_min = 0;
    uint16_t vsys_max = 0;

    float current_a_offset = 0;
    float current_b_offset = 0;
    float current_c_offset = 0;
    float current_d_offset = 0;

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
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // TODO: bootloader pin

    // Drivers EN1 (enable) pins. Output
    LL_GPIO_SetPinMode(DRIVER_ENABLE_GPIO_PORT, DRIVER_A_ENABLE_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(DRIVER_ENABLE_GPIO_PORT, DRIVER_B_ENABLE_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(DRIVER_ENABLE_GPIO_PORT, DRIVER_C_ENABLE_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(DRIVER_ENABLE_GPIO_PORT, DRIVER_D_ENABLE_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_ResetOutputPin(DRIVER_ENABLE_GPIO_PORT, DRIVER_A_ENABLE_PIN | DRIVER_B_ENABLE_PIN | DRIVER_C_ENABLE_PIN | DRIVER_D_ENABLE_PIN);

    // Drivers EN2 (direction) pins. Output via TIM1
    LL_GPIO_SetPinMode(DRIVER_DIN_GPIO_PORT, DRIVER_A_DIN_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(DRIVER_DIN_GPIO_PORT, DRIVER_B_DIN_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(DRIVER_DIN_GPIO_PORT, DRIVER_C_DIN_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(DRIVER_DIN_GPIO_PORT, DRIVER_D_DIN_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(DRIVER_DIN_GPIO_PORT, DRIVER_A_DIN_PIN, LL_GPIO_AF_6);
    LL_GPIO_SetAFPin_8_15(DRIVER_DIN_GPIO_PORT, DRIVER_B_DIN_PIN, LL_GPIO_AF_11);
    LL_GPIO_SetAFPin_8_15(DRIVER_DIN_GPIO_PORT, DRIVER_C_DIN_PIN, LL_GPIO_AF_6);
    LL_GPIO_SetAFPin_8_15(DRIVER_DIN_GPIO_PORT, DRIVER_D_DIN_PIN, LL_GPIO_AF_6);
    LL_GPIO_SetPinSpeed(DRIVER_DIN_GPIO_PORT, DRIVER_A_DIN_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed(DRIVER_DIN_GPIO_PORT, DRIVER_B_DIN_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed(DRIVER_DIN_GPIO_PORT, DRIVER_C_DIN_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed(DRIVER_DIN_GPIO_PORT, DRIVER_D_DIN_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(DRIVER_DIN_GPIO_PORT, DRIVER_A_DIN_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(DRIVER_DIN_GPIO_PORT, DRIVER_B_DIN_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(DRIVER_DIN_GPIO_PORT, DRIVER_C_DIN_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(DRIVER_DIN_GPIO_PORT, DRIVER_D_DIN_PIN, LL_GPIO_OUTPUT_PUSHPULL);

    // Driver IPROPI current sense pins. Analog input (opamp)
    LL_GPIO_SetPinMode(SEN_GPIO_PORT, SEN_B_PIN, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(SEN_GPIO_PORT, SEN_C_PIN, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(SEN_GPIO_PORT, SEN_D_PIN, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(SEN_GPIO_PORT, SEN_A_PIN, LL_GPIO_MODE_ANALOG);

    // triac for phase 'D' enable. Output
    LL_GPIO_SetPinMode(OUT_D_EN_GPIO_PORT, OUT_D_EN_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_ResetOutputPin(OUT_D_EN_GPIO_PORT, OUT_D_EN_PIN); // disable triac

    // boost enable. Output open-drain with pullup enabled
    LL_GPIO_SetPinMode(BOOST_EN_GPIO_PORT, BOOST_EN_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(BOOST_EN_GPIO_PORT, BOOST_EN_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(BOOST_EN_GPIO_PORT, BOOST_EN_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_ResetOutputPin(BOOST_EN_GPIO_PORT, BOOST_EN_PIN); // disable boost

    // boost control. DAC1_OUT1
    LL_GPIO_SetPinMode(BOOST_DAC_GPIO_PORT, BOOST_DAC_PIN, LL_GPIO_MODE_ANALOG);

    // VM sense, ADC2_IN12
    LL_GPIO_SetPinMode(VM_SENSE_GPIO_PORT, VM_SENSE_PIN, LL_GPIO_MODE_ANALOG);

    // VSYS sense, ADC2_IN15
    LL_GPIO_SetPinMode(VSYS_SENSE_GPIO_PORT, VSYS_SENSE_PIN, LL_GPIO_MODE_ANALOG);

    // VSYS comp, COMP1_INP
    LL_GPIO_SetPinMode(VSYS_COMP_GPIO_PORT, VSYS_COMP_PIN, LL_GPIO_MODE_ANALOG);

    // PGOOD. Input
    LL_GPIO_SetPinMode(PGOOD_GPIO_PORT, PGOOD_PIN, LL_GPIO_MODE_INPUT);

    // Fault LED. Output via TIM20_CH2 (AF6)
    LL_GPIO_SetPinMode(LED_FAULT_GPIO_PORT, LED_FAULT_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(LED_FAULT_GPIO_PORT, LED_FAULT_PIN, LL_GPIO_AF_6);

    // Frontpanel button (input pullup) / encoder (input)
    LL_GPIO_SetPinMode(ENCODER_GPIO_PORT, ENCODER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(ENCODER_GPIO_PORT, ENCODER_BUTTON_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinMode(ENCODER_GPIO_PORT, ENCODER_A_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(ENCODER_GPIO_PORT, ENCODER_A_PIN, LL_GPIO_AF_2);
    LL_GPIO_SetPinMode(ENCODER_GPIO_PORT, ENCODER_B_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(ENCODER_GPIO_PORT, ENCODER_B_PIN, LL_GPIO_AF_2);


    // default states
    BSP_OutputEnable(false, false, false, false);
}

void initPwm()
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    // configure timer frequency
    uint32_t overflow = HAL_RCC_GetPCLK2Freq() / (STIM_PWM_FREQ * 2);
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

    pwm_timer->CCR1 = 0;
    pwm_timer->CCR2 = 0;
    pwm_timer->CCR3 = 0;
    pwm_timer->CCR4 = 0;

    // flip polarity (because DRV8231A)
    pwm_timer->CCER |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P | TIM_CCER_CC4P;

    // only generate interrupt on timer overflow, not EGR
    pwm_timer->CR1  |= TIM_CR1_URS;

    // enable update interrupt
    pwm_timer->DIER |= TIM_DIER_UIE;

    // force outputs low after break event.
    pwm_timer->BDTR |= TIM_BDTR_OSSI;

    // select update event as trigger output
    pwm_timer->CR2 |= LL_TIM_TRGO_UPDATE;

    enableInterruptWithPrio(TIM1_UP_TIM16_IRQn, 0);
    // TIM1_BRK_TIM15_IRQn --> TIM1_BRK_TIM15_IRQHandler
}

void initADCTriggerTimer()
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    // uint32_t clk = HAL_RCC_GetPCLK1Freq();
    // BSP_PrintDebugMsg("clock: %u\r\n", clk);

    adc_trigger_timer->PSC = 0;
    adc_trigger_timer->ARR = uint32_t(HAL_RCC_GetPCLK1Freq() * 700e-9f) - 1; // 700ns, driver input to output propagation delay
    adc_trigger_timer->EGR |= TIM_EGR_UG;  // force update of shadow registers
    adc_trigger_timer->CR1 |= TIM_CR1_OPM; // set one-pulse-mode
    adc_trigger_timer->CR1 |= TIM_CR1_URS; // only update event on timer overflow, not EGR
    adc_trigger_timer->CR2 |= LL_TIM_TRGO_UPDATE;   // select update event as trigger output

    MODIFY_REG(adc_trigger_timer->SMCR, TIM_SMCR_TS_Msk, TIM_TS_ITR0);                  // internal triger tim_itr0 = TIM1
    MODIFY_REG(adc_trigger_timer->SMCR, TIM_SMCR_SMS_Msk, TIM_SMCR_SMS_3);              // Combined reset + trigger mode
    MODIFY_REG(adc_trigger_timer->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_3);         // Retrigerrable OPM mode 1


    // DEBUG: output timer to PA0
    // adc_trigger_timer->CCR1 = 1;
    // adc_trigger_timer->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;     // pwm mode 2
    // adc_trigger_timer->CCER |= TIM_CCER_CC1E;
    // adc_trigger_timer->BDTR |= TIM_BDTR_MOE;    // main output enable

    // LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
    // LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_0, LL_GPIO_AF_1);   // AF1 = TIM2_CH1
    // LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    // LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
}

void initLedTimer()
{
    __HAL_RCC_TIM20_CLK_ENABLE();

    app_and_led_timer->PSC = HAL_RCC_GetPCLK2Freq() / LED_TIMER_FREQ_Hz - 1;
    app_and_led_timer->ARR = uint16_t(LED_TIMER_PEAK) - 1;
    app_and_led_timer->DIER |= TIM_DIER_UIE;            // Interrupt on timer update
    app_and_led_timer->EGR |= TIM_EGR_UG;               // Force update of the shadow registers.

    app_and_led_timer->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;    // pwm mode 1
    app_and_led_timer->CCER |= TIM_CCER_CC2E;  // enable TIMx_CH2s

    app_and_led_timer->CCR2 = 0;    // led off
    app_and_led_timer->CR1 |= TIM_CR1_CEN;              // Enable the counter.
    app_and_led_timer->BDTR |= TIM_BDTR_MOE;            // main output enable

    enableInterruptWithPrio(TIM20_UP_IRQn, 1);
}

void configureOpamp(OPAMP_HandleTypeDef *hopamp, OPAMP_TypeDef *OPAMPx_Def)
{
    hopamp->Instance = OPAMPx_Def;
    hopamp->Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
    hopamp->Init.Mode = OPAMP_FOLLOWER_MODE;
    if (hopamp->Instance == OPAMP2)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;   // OPAMP2_VINP = PB14 = B
    }
    if (hopamp->Instance == OPAMP3)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;   // OPAMP3_VINP = PB13 = C
    }
    if (hopamp->Instance == OPAMP4)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;   // OPAMP4_VINP = PB11 = D
    }
    if (hopamp->Instance == OPAMP6)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;   // OPAMP6_VINP = PB12 = A
    }
    hopamp->Init.InternalOutput = ENABLE;
    hopamp->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
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
    hadc->Init.NbrOfConversion = 2;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
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
    sConfig.SamplingTime = ADC_SAMPLETIME_VREFINT;
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    // internal stm32 temp sensor
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_V_TS;
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
    hadc->Init.NbrOfConversion = 2;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
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

    // SEN_B
    sConfig.Channel = ADC_CHANNEL_VOPAMP2;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_CURRENT_SENSE;
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    // VM_SENSE
    sConfig.Channel = ADC2_CHANNEL_VM_SENSE;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_VM;
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
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
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

    // SEN_C
    sConfig.Channel = ADC_CHANNEL_VOPAMP3_ADC3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_CURRENT_SENSE;
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
    hadc->Init.NbrOfConversion = 2;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
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

    // SEN_A
    sConfig.Channel = ADC_CHANNEL_VOPAMP6;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_CURRENT_SENSE;
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    // VSYS_sense
    sConfig.Channel = ADC4_CHANNEL_VSYS_SENSE;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_VSYS;
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
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
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

    // SEN_D
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

    // DAC1_CH1, used for boost control
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

void initEncoderTimer()
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    encoder_timer->CNT = ENCODER_INIT_CNT;
    encoder_timer->TISEL = 0;					// TIMx_CH1 = tim_ti1_in0 and TIMx_CH2 = tim_ti2_in0
    encoder_timer->CCMR1 |= TIM_CCMR1_CC1S_0;	// tim_ti1fp1 mapped on tim_ti1
    encoder_timer->CCMR1 |= TIM_CCMR1_CC2S_0;	// tim_ti2fp2 mapped on tim_ti2
    encoder_timer->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;	// Counting on tim_ti1 and tim_ti2 x4 mode
    encoder_timer->CCER |= TIM_CCER_CC1P;       // invert polarity so clockwise = increase

    encoder_timer->CR1 |= TIM_CR1_CEN;

    uint32_t cnt = encoder_timer->CNT;
}

void initVRef()
{
    HAL_SYSCFG_DisableVREFBUF();
    MODIFY_REG(VREFBUF->CSR, VREFBUF_CSR_VRS_Msk, VREFBUF_CSR_VRS_1); // VRS 10 = 2.9v
    CLEAR_BIT(VREFBUF->CSR, VREFBUF_CSR_HIZ);  // VREF+ pin is internally connected to the voltage reference buffer output

    HAL_StatusTypeDef status;
    status = HAL_SYSCFG_EnableVREFBUF();
    if (status != HAL_OK)
    {
        BSP_PrintDebugMsg("vrefbuf init failed %i\n", status);
        Error_Handler();
    }
}

void BSP_Init()
{
    initGPIO();
    initPwm();
    initADCTriggerTimer();
    initVRef();
    initADC();
    initOpamp();
    initDMA();
    initDAC();
    initCOMP();
    initEXTI();
    initLedTimer();
    initEncoderTimer();

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

            bsp.vsys_min = min(bsp.vsys_min, (uint16_t)bsp.vsys_sense);
            bsp.vsys_max = max(bsp.vsys_max, (uint16_t)bsp.vsys_sense);

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

    // 1000hz timer for LED and app.
    void TIM20_UP_IRQHandler(void)
    {
        if (app_and_led_timer->SR & TIM_SR_UIF) {         // Update
            app_and_led_timer->SR &= ~TIM_SR_UIF;         // Clear the interrupt.

            // process LED pattern
            switch (bsp.led_pattern) {
            case Idle:
                bsp.led_pattern_progress = (bsp.led_pattern_progress + 1) % 1024;
                BSP_WriteRedLedBrightness(bsp.led_pattern_progress < 100 ? .5 : .1);
            break;
            case Error:
                bsp.led_pattern_progress = (bsp.led_pattern_progress + 1) % 400;
                BSP_WriteRedLedBrightness(bsp.led_pattern_progress < 100 ? 1 : .5);
            break;
            case PlayingVeryLow:
            case PlayingLow:
            case PlayingMedium:
            case PlayingHigh:
                bsp.led_pattern_progress = (bsp.led_pattern_progress + 1) % 1024;
                BSP_WriteRedLedBrightness(bsp.led_pattern_progress / 1024.f * 0.5f);
            break;
            }
        }
    }
}

void BSP_OutputEnable(bool a, bool b, bool c, bool d)
{
    // TODO TODO: race conditions!

    // BSP_PrintDebugMsg("output enable: %i %i %i %i", a, b, c, d);
    HAL_GPIO_WritePin(DRIVER_ENABLE_GPIO_PORT, DRIVER_A_ENABLE_PIN, (GPIO_PinState)a);
    HAL_GPIO_WritePin(DRIVER_ENABLE_GPIO_PORT, DRIVER_B_ENABLE_PIN, (GPIO_PinState)b);
    HAL_GPIO_WritePin(DRIVER_ENABLE_GPIO_PORT, DRIVER_C_ENABLE_PIN, (GPIO_PinState)c);
    HAL_GPIO_WritePin(DRIVER_ENABLE_GPIO_PORT, DRIVER_D_ENABLE_PIN, (GPIO_PinState)d);
    HAL_GPIO_WritePin(OUT_D_EN_GPIO_PORT, OUT_D_EN_PIN, d ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (a) {
        SET_BIT(pwm_timer->CCER, TIM_CCER_CC3E);
    } else {
        CLEAR_BIT(pwm_timer->CCER, TIM_CCER_CC3E);
    }
    if (b) {
        SET_BIT(pwm_timer->CCER, TIM_CCER_CC4E);
    } else {
        CLEAR_BIT(pwm_timer->CCER, TIM_CCER_CC4E);
    }
    if (c) {
        SET_BIT(pwm_timer->CCER, TIM_CCER_CC1E);
    } else {
        CLEAR_BIT(pwm_timer->CCER, TIM_CCER_CC1E);
    }
    if (d) {
        SET_BIT(pwm_timer->CCER, TIM_CCER_CC2E);
    } else {
        CLEAR_BIT(pwm_timer->CCER, TIM_CCER_CC2E);
    }


    if (a || b || c || d) {
        pwm_timer->BDTR |= TIM_BDTR_MOE;
    } else {
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
    uint32_t ccr_a = constrain(a, 0, 1) * arr;
    uint32_t ccr_b = constrain(b, 0, 1) * arr;
    uint32_t ccr_c = constrain(c, 0, 1) * arr;
    uint32_t ccr_d = 0;

    pwm_timer->CCR1 = ccr_c;
    pwm_timer->CCR2 = ccr_d;
    pwm_timer->CCR3 = ccr_a;
    pwm_timer->CCR4 = ccr_b;
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
    uint32_t ccr_a = constrain(a, 0, 1) * arr;
    uint32_t ccr_b = constrain(b, 0, 1) * arr;
    uint32_t ccr_c = constrain(c, 0, 1) * arr;
    uint32_t ccr_d = constrain(d, 0, 1) * arr;

    pwm_timer->CCR1 = ccr_c;
    pwm_timer->CCR2 = ccr_d;
    pwm_timer->CCR3 = ccr_a;
    pwm_timer->CCR4 = ccr_b;
}

void BSP_SetPWM4Atomic(float a, float b, float c, float d)
{
    SET_BIT(pwm_timer->CR1, TIM_CR1_UDIS_Msk);
    BSP_SetPWM4(a, b, c, d);
    CLEAR_BIT(pwm_timer->CR1, TIM_CR1_UDIS_Msk);
}

Vec3f BSP_ReadPhaseCurrents3()
{
    float gain = 1;
    float factor = ADC_VOLTAGE / ADC_SCALE / DRIVER_RISEN * DRIVER_SENSE_CURENT_TO_COIL_CURRENT / -gain;

    return Vec3f(
        (bsp.current_a - bsp.current_a_offset) * factor,
        (bsp.current_b - bsp.current_b_offset) * factor,
        (bsp.current_c - bsp.current_c_offset) * factor);
}

Vec4f BSP_ReadPhaseCurrents4()
{
    float gain = 1;
    float factor = ADC_VOLTAGE / ADC_SCALE / DRIVER_RISEN * DRIVER_SENSE_CURENT_TO_COIL_CURRENT / -gain;

    return Vec4f(
        (bsp.current_a - bsp.current_a_offset) * factor,
        (bsp.current_b - bsp.current_b_offset) * factor,
        (bsp.current_c - bsp.current_c_offset) * factor,
        (bsp.current_d - bsp.current_d_offset) * factor);
}

void BSP_WriteStatusLED(bool on)
{

}

void BSP_WriteFaultLED(bool on)
{

}

float BSP_ReadTemperatureSTM()
{
    float ts_data = bsp.v_ts * (ADC_VOLTAGE / TEMPSENSOR_CAL_VREFANALOG * 1000);
    float offset = TEMPSENSOR_CAL1_TEMP;
    float slope = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / float((*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR));
    float temp = offset + (ts_data - *TEMPSENSOR_CAL1_ADDR) * slope;
    return temp;
}

float BSP_ReadPotentiometerPercentage()
{
    // FIXME
    // return 1;

    int16_t cnt = encoder_timer->CNT & 0xFFFF;
    if (cnt < 0) {
        cnt = 0;
        CLEAR_BIT(encoder_timer->CNT, 0xFFFF);
    } else if (cnt >= ENCODER_100_PERCENT_VALUE) {
        cnt = ENCODER_100_PERCENT_VALUE;
        MODIFY_REG(encoder_timer->CNT, 0xFFFF, cnt);
    }
    return _constrain(cnt / float(ENCODER_100_PERCENT_VALUE), 0.f, 1.f);
}

float BSP_ReadTemperatureOnboardNTC()
{
    return 0;   // not supported
}

void BSP_AdjustCurrentSenseOffsets()
{
    // The DRV8231A has some offset error, on my board between 0v and 0.008v (~5mA drive current)
    bsp.current_a_offset += (bsp.current_a - bsp.current_a_offset) * 0.01f;
    bsp.current_b_offset += (bsp.current_b - bsp.current_b_offset) * 0.01f;
    bsp.current_c_offset += (bsp.current_c - bsp.current_c_offset) * 0.01f;
    bsp.current_d_offset += (bsp.current_d - bsp.current_d_offset) * 0.01f;
}

float BSP_ReadVBus()
{
    float adc_voltage = bsp.vm_sense * (ADC_VOLTAGE / ADC_SCALE);
    const float multiplier = (20 + 200) / 20.f;
    const float correction_factor = (1.9996 / 1.9848);  // correct for the voltage drop from ADC sampling. Measured with multimeter at boost voltage divider at 22v and 50khz pwm.
    return adc_voltage * (multiplier * correction_factor);
}

float BSP_ReadVSYS()
{
    // VSYS reads slightly low (about 2.5%) because of the high sampling frequency and high resistors used in the divider.
    float adc_voltage = bsp.vsys_sense * (ADC_VOLTAGE / ADC_SCALE);
    const float multiplier = (100 + 100) / 100.f;
    return adc_voltage * multiplier;
}

Vec2f BSP_ReadVSYSRange()
{
    // VSYS reads slightly low (about 2.5%) because of the high sampling frequency and high resistors used in the divider.
    float adc_voltage_min = bsp.vsys_min * (ADC_VOLTAGE / ADC_SCALE);
    bsp.vsys_min = ADC_SCALE;
    float adc_voltage_max = bsp.vsys_max * (ADC_VOLTAGE / ADC_SCALE);
    bsp.vsys_max = 0;
    const float multiplier = (100 + 100) / 100.f;
    return {adc_voltage_min * multiplier, adc_voltage_max * multiplier};
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
    // DAC 2.4v or higher -> boost ~4v
    // DAC 0.2v           -> boost 31.9v
    // DAC 0.0v           -> boost 34.4v
    // With output buffer on, min DAC output is 0.2v, and VM_sense caps out at 31.9v.
    // use 30v max to have some headroom for over-voltage detection.
    boost_voltage = min(30.f, max(0.f, boost_voltage));

    float fb = 1.229f;
    float rtop = 1000000;
    float rbot = 69000;
    float rdac = 80000;
    float vdac = (fb * (1 + rtop / rbot + rtop / rdac) - boost_voltage) * (rdac / rtop);
    // inverse: boost_voltage = fb * (1 + rtop / rbot + rtop / rdac) - vdac * (rtop / rdac);
    int value = min(DAC_MAX_VALUE, max(0, int(vdac * (DAC_MAX_VALUE / ADC_VOLTAGE))));

    // value to be written to the DAC on next interrupt.
    bsp.boost_dac_value = value;
}

void BSP_SetBoostMinimumInputVoltage(float voltage)
{
    vsys_comp_dac->DHR12R1 = (voltage / 2) * (ADC_SCALE / ADC_VOLTAGE);
}

void BSP_SetTriacEnable(bool enable)
{
    HAL_GPIO_WritePin(OUT_D_EN_GPIO_PORT, OUT_D_EN_PIN, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

float BSP_BoostDutyCycle()
{
    float duty = float(bsp.boost_on_cycles) / float(bsp.boost_on_cycles + bsp.boost_off_cycles);
    bsp.boost_on_cycles = 0;
    bsp.boost_off_cycles = 0;
    return duty;
}

void BSP_WriteGreenLedBrightness(float a)
{

}

void BSP_WriteRedLedBrightness(float a)
{
    app_and_led_timer->CCR2 = a * (LED_TIMER_PEAK * RED_LED_BRIGHTNESS);
}

void BSP_WriteLedPattern(LedPattern pattern)
{
    bsp.led_pattern = pattern;
}

bool BSP_ReadPGood()
{
    return LL_GPIO_IsInputPinSet(PGOOD_GPIO_PORT, PGOOD_PIN);
}

bool BSP_ReadEncoderButton()
{
    return LL_GPIO_IsInputPinSet(ENCODER_GPIO_PORT, ENCODER_BUTTON_PIN);
}

void BSP_PrintDebugMsg(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    g_protobuf->transmit_notification_debug_string(fmt, args);
    va_end(args);
}

#endif