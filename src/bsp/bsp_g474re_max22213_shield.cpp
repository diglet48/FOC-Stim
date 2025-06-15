#if defined(ARDUINO_NUCLEO_G474RE)
#include "bsp.h"

#include <Arduino.h>
#include <stm32g4xx_ll_dac.h>
#include <stm32g4xx_ll_comp.h>
#include <stm32g4xx_ll_exti.h>

#define ADC_VOLTAGE 3.272f
#define ADC_SCALE   4095

#define MAX22213_RISEN 2000
#if MAX22213_HFS == 0
#define MAX22213_KISEN 7500     // HFS 0, max current 3A
#else
#define MAX22213_KISEN 3840     // HFS 1, max current 1.5A, higher resoution
#endif


/* ADC connections
phase 1/A = PB0 / PB2   to opamp3. Internal output ADC2 and ADC3
phase 2/B = PB11 / PB10 to opamp4. Internal output ADC5
phase 3/C = PB14 / PB15 to opamp5. Internal output ADC5
phase 4/D = PB13 / PA1  to opamp6. Internal output ADC4
*/

#define ADC2_CHANNEL_VM_SENSE   ADC_CHANNEL_17; // PA4
#define ADC2_CHANNEL_VSYS_SENSE   ADC_CHANNEL_6; // PC6


#define A_FAULT 	PC3
#define A_SLEEP 	PA12
#define A_HFS 		PB12
#define A_EN1 		PC6
#define A_EN2 		PC7
#define A_EN3 		PC8
#define A_EN4 		PC9
#define A_DIN1 		PA8
#define A_DIN2 		PA9
#define A_DIN3 		PA10
#define A_DIN4 		PA11_ALT1
#define A_SEN1_PLUS PB0
#define A_SEN2_PLUS PB11
#define A_SEN3_PLUS PB14
#define A_SEN4_PLUS PB13
#define A_SEN1_MINUS PB2
#define A_SEN2_MINUS PB10
#define A_SEN3_MINUS PB15
#define A_SEN4_MINUS PA1
#define A_VM_SENSE PA4

#define A_BOOST_EN PB9
#define A_VSYS_SENSE PC0


static TIM_TypeDef *const pwm_timer = TIM1;

struct BSP {
    OPAMP_HandleTypeDef opamp3; // ISEN1
    OPAMP_HandleTypeDef opamp4; // ISEN2
    OPAMP_HandleTypeDef opamp5; // ISEN3
    OPAMP_HandleTypeDef opamp6; // ISEN4

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
        };
    };
    union {
        uint16_t adc2_buffer[2];
        struct {
            volatile uint16_t vm_sense;  // TODO not trigger so often.
            volatile uint16_t vsys_sense;
        };
    };
    union {
        uint16_t adc3_buffer[1];
        struct {
            volatile uint16_t current_a;
        };
    };
    union {
        uint16_t adc4_buffer[1];
        struct {
            volatile uint16_t current_d;
        };
    };
    union {
        uint16_t adc5_buffer[2];
        struct {
            volatile uint16_t current_b;
            volatile uint16_t current_c;
        };
    };

    std::function<void()> pwm_callback;

    volatile int vsys_high_cycles = 0;
    uint16_t boost_set_value = 4095;
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
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DAC2_CLK_ENABLE();
    __HAL_RCC_DAC3_CLK_ENABLE();

    // TODO: bootloader

    // init pin modes
    pinMode(A_FAULT, INPUT);
    pinMode(A_SLEEP, OUTPUT);
    pinMode(A_HFS, OUTPUT);

    pinMode(A_EN1, OUTPUT);
    pinMode(A_EN2, OUTPUT);
    pinMode(A_EN3, OUTPUT);
    pinMode(A_EN4, OUTPUT);

    pinMode(A_BOOST_EN, OUTPUT);
    digitalWrite(A_BOOST_EN, 0);
    pinMode(A_VSYS_SENSE, INPUT_ANALOG);

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6; // TIM1_CH1, TIM1_CH2, TIM1_CH3
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
    LL_GPIO_ResetOutputPin(GPIOA, GPIO_InitStruct.Pin);
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_11; // TIM1_CH4
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
    LL_GPIO_ResetOutputPin(GPIOA, GPIO_InitStruct.Pin);
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // DAC
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6;    // PA6
    LL_GPIO_ResetOutputPin(GPIOA, GPIO_InitStruct.Pin);
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    // COMP
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;    // PC1
    LL_GPIO_ResetOutputPin(GPIOC, GPIO_InitStruct.Pin);
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    // default states
    digitalWrite(A_HFS, MAX22213_HFS);

    // BSP_OutputEnable(false, false, false, false);
    BSP_OutputEnable(false, false, false);
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

void configureOpamp(OPAMP_HandleTypeDef *hopamp, OPAMP_TypeDef *OPAMPx_Def)
{
    hopamp->Instance = OPAMPx_Def;
    hopamp->Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
    hopamp->Init.Mode = OPAMP_PGA_MODE;
    if (hopamp->Instance == OPAMP3)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    }
    if (hopamp->Instance == OPAMP4)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
    }
    if (hopamp->Instance == OPAMP5)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    }
    if (hopamp->Instance == OPAMP6)
    {
        hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
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
    configureOpamp(&bsp.opamp3, OPAMP3);
    configureOpamp(&bsp.opamp4, OPAMP4);
    configureOpamp(&bsp.opamp5, OPAMP5);
    configureOpamp(&bsp.opamp6, OPAMP6);
}

/**
 * @brief OPAMP MSP Initialization
 * @param hopamp-> OPAMP handle pointer
 * @retval None
 */
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef *hopamp)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hopamp->Instance == OPAMP3)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**OPAMP3 GPIO Configuration
        PB0     ------> A_SEN1_PLUS
        PB2     ------> A_SEN1_MINUS
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (hopamp->Instance == OPAMP4)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**OPAMP4 GPIO Configuration
        PB11     ------> A_SEN2_PLUS
        PB10     ------> A_SEN2_MINUS
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (hopamp->Instance == OPAMP5)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**OPAMP5 GPIO Configuration
        PB14     ------> A_SEN3_PLUS
        PB15     ------> A_SEN4_MINUS
        */
        GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (hopamp->Instance == OPAMP6)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**OPAMP6 GPIO Configuration
        PB13     ------> A_SEN4_PLUS
        PA1      ------> A_SEN4_MINUS
        */
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/**
 * @brief OPAMP MSP De-Initialization
 * @param hopamp-> OPAMP handle pointer
 * @retval None
 */
void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef *hopamp)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hopamp->Instance == OPAMP3)
    {
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_2);
    }
    else if (hopamp->Instance == OPAMP4)
    {
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    }
    else if (hopamp->Instance == OPAMP5)
    {
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14 | GPIO_PIN_15);
    }
    else if (hopamp->Instance == OPAMP6)
    {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
    }
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

    // VM_SENSE
    sConfig.Channel = ADC2_CHANNEL_VM_SENSE
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5; // ~0.8µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    // VSYS_sense
    sConfig.Channel = ADC2_CHANNEL_VSYS_SENSE
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5; // ~0.8µs
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

    // ISEN1
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
    hadc->Init.NbrOfConversion = 2;
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
    sConfig.Channel = ADC_CHANNEL_VOPAMP4; // ISEN2
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; // ~0.54µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    // ISEN3
    sConfig.Channel = ADC_CHANNEL_VOPAMP5; // ISEN3
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; // ~0.54µs
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        BSP_PrintDebugMsg("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
}

void initADC()
{
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
    // DMA1_Channel2->CCR &= ~DMA_CCR_HTIE;    // enable only transfer complete interrupt.
    DMA1_Channel5->CCR &= ~DMA_CCR_HTIE;    // enable only transfer complete interrupt.
}

void initDAC()
{
    // DAC2_CH1, used for VMCONT
    LL_DAC_InitTypeDef DAC_InitStruct = {
        .TriggerSource = LL_DAC_TRIG_SOFTWARE,
        .WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE,
        .OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE,
        .OutputConnection = LL_DAC_OUTPUT_CONNECT_GPIO,
        .OutputMode = LL_DAC_OUTPUT_MODE_NORMAL
    };
    LL_DAC_Init(DAC2, LL_DAC_CHANNEL_1, &DAC_InitStruct);
    LL_DAC_Enable(DAC2, LL_DAC_CHANNEL_1);
    while (DAC2->SR & DAC_SR_DAC1RDY) {}    // wait until DAC is ready

    // DAC3_CH1, used for COMP3
    DAC_InitStruct = LL_DAC_InitTypeDef{
        .TriggerSource = LL_DAC_TRIG_SOFTWARE,
        .WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE,
        .OutputBuffer = LL_DAC_OUTPUT_BUFFER_DISABLE,
        .OutputConnection = LL_DAC_OUTPUT_CONNECT_INTERNAL,
        .OutputMode = LL_DAC_OUTPUT_MODE_NORMAL
    };
    LL_DAC_Init(DAC3, LL_DAC_CHANNEL_1, &DAC_InitStruct);
    LL_DAC_Enable(DAC3, LL_DAC_CHANNEL_1);
    while (DAC3->SR & DAC_SR_DAC1RDY) {}    // wait until DAC is ready


    DAC2->DHR12R1 = 4095;
    DAC3->DHR12R1 = 4095;
}

void initCOMP()
{
    LL_COMP_InitTypeDef COMP_InitStruct = {
        .InputPlus = LL_COMP_INPUT_PLUS_IO2,        // PC1
        .InputMinus = LL_COMP_INPUT_MINUS_DAC3_CH1,
        .InputHysteresis = LL_COMP_HYSTERESIS_NONE,
        .OutputPolarity = LL_COMP_OUTPUT_LEVEL_HIGH,
        .OutputBlankingSource = LL_COMP_BLANKINGSRC_NONE
    };

    LL_COMP_Init(COMP3, &COMP_InitStruct);
    COMP3->CSR |= COMP_CSR_EN;
}

void initEXTI()
{
    // COMP3
    LL_EXTI_InitTypeDef EXTI_InitStruct = {
        .Line_0_31 = LL_EXTI_LINE_29,
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

    // enable timer. DIR bit aligns timer update event with either peak or through.
    pwm_timer->CR1 |= TIM_CR1_CEN;
}

extern "C"
{
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

    void COMP1_2_3_IRQHandler(void)
    {
        if (EXTI->PR1 & LL_EXTI_LINE_29) {
            EXTI->PR1 = LL_EXTI_LINE_29;
            EXTI->EMR1 |= LL_EXTI_LINE_29;  // mask interrupt to save cycles

            // disable the boost
            DAC2->DHR12R1 = 4095;
            bsp.vsys_high_cycles = 0;
        }
    }

    void TIM1_UP_TIM16_IRQHandler(void)
    {
        pwm_timer->SR &= ~TIM_SR_UIF;

        bool vsys_high = bool(COMP3->CSR & COMP_CSR_VALUE_Msk);
        bool interrupt_pending = bool(EXTI->PR1 & LL_EXTI_LINE_29);
        if (vsys_high && !interrupt_pending) {
            bsp.vsys_high_cycles++;
        } else {
            bsp.vsys_high_cycles = 0;
        }

        if (bsp.vsys_high_cycles >= 2) {
            // enable boost
            DAC2->DHR12R1 = bsp.boost_set_value;
            EXTI->EMR1 &= ~LL_EXTI_LINE_29;  // enable comp interrupt
        }
    }
}

void BSP_OutputEnable(bool a, bool b, bool c, bool d)
{
    // BSP_PrintDebugMsg("output enable: %i %i %i %i", a, b, c, d);
    digitalWrite(A_EN1, a);
    digitalWrite(A_EN2, b);
    digitalWrite(A_EN3, c);
    digitalWrite(A_EN4, d);

    if (a || b || c || d) {
        digitalWrite(A_SLEEP, 1);
        pwm_timer->BDTR |= TIM_BDTR_MOE;
    } else {
        digitalWrite(A_SLEEP, 0);
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
    uint32_t ccr1 = constrain(a, 0, 1) * arr;
    uint32_t ccr2 = constrain(b, 0, 1) * arr;
    uint32_t ccr3 = constrain(c, 0, 1) * arr;
    pwm_timer->CCR1 = ccr1;
    pwm_timer->CCR2 = ccr2;
    pwm_timer->CCR3 = ccr3;
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
    uint32_t ccr1 = constrain(a, 0, 1) * arr;
    uint32_t ccr2 = constrain(b, 0, 1) * arr;
    uint32_t ccr3 = constrain(c, 0, 1) * arr;
    uint32_t ccr4 = constrain(d, 0, 1) * arr;
    pwm_timer->CCR1 = ccr1;
    pwm_timer->CCR2 = ccr2;
    pwm_timer->CCR3 = ccr3;
    pwm_timer->CCR4 = ccr4;
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
        bsp.current_a * factor,
        bsp.current_b * factor,
        bsp.current_c * factor);
}

Vec4f BSP_ReadPhaseCurrents4()
{
    float gain = 4;
    float factor = ADC_VOLTAGE / ADC_SCALE / MAX22213_RISEN * MAX22213_KISEN / -gain;

    return Vec4f(
        bsp.current_a * factor,
        bsp.current_b * factor,
        bsp.current_c * factor,
        bsp.current_d * factor);
}

bool BSP_ReadFault()
{
    // TODO: use in software.
    return digitalRead(A_FAULT);
}

void BSP_WriteStatusLED(bool on)
{
    digitalWrite(LED_BUILTIN, on);
}

float BSP_ReadTemperatureSTM()
{
    float ts_data = bsp.v_ts * (ADC_VOLTAGE / TEMPSENSOR_CAL_VREFANALOG * 1000);
    float offset = TEMPSENSOR_CAL1_TEMP;
    float slope = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / float((*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR));
    float temp = offset + (ts_data - *TEMPSENSOR_CAL1_ADDR) * slope;
    return temp;
}

float BSP_ReadPotentiometer()
{
    return 0;   // not supported
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
    const float multiplier = (18 + 169) / 18.f;
    return adc_voltage * multiplier;
}

float BSP_ReadVSYS()
{
    float adc_voltage = bsp.vsys_sense * (ADC_VOLTAGE / ADC_SCALE);
    const float multiplier = (10 + 10) / 10.f;
    return adc_voltage * multiplier;
}

float BSP_ReadChipAnalogVoltage()
{
    return __LL_ADC_CALC_VREFANALOG_VOLTAGE(bsp.vrefint, LL_ADC_RESOLUTION_12B) * 0.001f;
}

void BSP_SetBoostEnable(bool enable)
{
    digitalWrite(A_BOOST_EN, enable);
}

void BSP_SetBoostVoltage(float boost_voltage)
{
    // min ~12.0
    // max ~36.5

    float fb = 1.229f;
    float rtop = 1000000;
    float rbot = 47000;
    float rdac = 133000;
    float vdac = (fb * (1 + rtop / rbot + rtop / rdac) - boost_voltage) * (rdac / rtop);
    int value = min(4095, max(0, int(vdac * (4095 / ADC_VOLTAGE))));

    bsp.boost_set_value = value;
}

void BSP_SetBoostMinimumInputVoltage(float voltage)
{
    DAC3->DHR12R1 = (voltage / 2) * (ADC_SCALE / ADC_VOLTAGE);
}

#endif