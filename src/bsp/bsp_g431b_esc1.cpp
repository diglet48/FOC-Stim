#include "bsp.h"

#include "config.h"
#include "utils.h"

#include "stm32g4xx_hal.h"
#include <Arduino.h>



#define ADC_VOLTAGE 3.3f
#define ADC_SCALE   4095

#define DEAD_TIME_NS    300
// #define DEAD_TIME_NS 1e9 / (STIM_PWM_FREQ * 2)  // simpleFOC default


static TIM_TypeDef *const pwm_timer = TIM1;

struct BSP {
    OPAMP_HandleTypeDef opamp1;
    OPAMP_HandleTypeDef opamp2;
    OPAMP_HandleTypeDef opamp3;

    ADC_HandleTypeDef adc1;
    ADC_HandleTypeDef adc2;

    DMA_HandleTypeDef dma1;
    DMA_HandleTypeDef dma2;

    union {
        uint16_t adc1_buffer[5];
        struct {
            volatile uint16_t current_b;        // ADC1_IN12 = PB1 = OP3_OUT, output on corner of board. "left"
            volatile uint16_t current_c;        // ADC1_IN3  = PA2 = OP1_OUT, output closest to pot. "right"
            volatile uint16_t potentiometer;    // ADC1_IN11 = PB12
            volatile uint16_t onboard_ntc;      // ADC1_IN15 = PB14
            volatile uint16_t vbus;             // ADC1_IN1  = PA0
            // TODO: vrefint
            // todo: TS
        };
    };
    union {
        uint16_t adc2_buffer[1];
        struct {
            volatile uint16_t current_a;        // ADC2_IN3 = PA6 = OP2_OUT, middle output. "center" or "neutral"
        };
    };

    float current_a_offset;
    float current_b_offset;
    float current_c_offset;

    std::function<void()> pwm_callback;
} bsp;



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

    pinMode(LED_BUILTIN, OUTPUT);

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6; // TIM1_CH1, TIM1_CH2, TIM1_CH3, TIM1_CH2N
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_12;
    LL_GPIO_ResetOutputPin(GPIOA, GPIO_InitStruct.Pin);
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4; // TIM3_CH1N
    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    LL_GPIO_ResetOutputPin(GPIOB, GPIO_InitStruct.Pin);
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4; // TIM1_CH1N
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
    LL_GPIO_ResetOutputPin(GPIOC, GPIO_InitStruct.Pin);
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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

    // enable outputs and complementary outputs
    pwm_timer->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    pwm_timer->CCER |= TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;

    // only generate interrupt on timer overflow, not EGR
    pwm_timer->CR1 |= TIM_CR1_URS;

    // Enable update interrupt.
    // TODO: disable interrupt when no callback active to save some cycles
    pwm_timer->DIER |= TIM_DIER_UIE;

    // setup deadtime.
    uint32_t dead_time = __LL_TIM_CALC_DEADTIME(SystemCoreClock, LL_TIM_GetClockDivision(pwm_timer), DEAD_TIME_NS);
    if (dead_time>255) dead_time = 255;
    if (dead_time==0) {
      dead_time = 255; // LL_TIM_CALC_DEADTIME returns 0 if dead_time_ns is too large
      Serial.printf("STM32-DRV: WARN: dead time too large, setting to max\n");
    }
    LL_TIM_OC_SetDeadTime(pwm_timer, dead_time); // deadtime is non linear!

    // force outputs low when timer disable.
    pwm_timer->BDTR &= ~TIM_BDTR_OSSI;
    pwm_timer->BDTR |= TIM_BDTR_OSSR;

    enableInterruptWithPrio(TIM1_UP_TIM16_IRQn, 0);
    // TIM1_BRK_TIM15_IRQn --> TIM1_BRK_TIM15_IRQHandler
}

void configureOpamp(OPAMP_HandleTypeDef *hopamp, OPAMP_TypeDef *OPAMPx_Def)
{
    hopamp->Instance = OPAMPx_Def;
    hopamp->Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
    hopamp->Init.Mode = OPAMP_PGA_MODE;
    hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    hopamp->Init.InternalOutput = DISABLE;
    hopamp->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
    hopamp->Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
    hopamp->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

    HAL_StatusTypeDef status;
    status = HAL_OPAMP_Init(hopamp);
    if (status != HAL_OK)
    {
        Serial.printf("opamp init failed %i\n", status);
        Error_Handler();
    }

    status = HAL_OPAMP_Start(hopamp);
    if (status != HAL_OK)
    {
        Serial.printf("opamp start failed %i\n", status);
        Error_Handler();
    }
}

void initOpamp()
{
    // 28100 bytes
    configureOpamp(&bsp.opamp1, OPAMP1);
    configureOpamp(&bsp.opamp2, OPAMP2);
    configureOpamp(&bsp.opamp3, OPAMP3);
}


/**
 * @brief OPAMP MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hopamp-> OPAMP handle pointer
 * @retval None
 */
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef *hopamp)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hopamp->Instance == OPAMP1)
    {
        /* USER CODE BEGIN OPAMP1_MspInit 0 */

        /* USER CODE END OPAMP1_MspInit 0 */

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**OPAMP1 GPIO Configuration
        PA1     ------> OPAMP1_VINP
        PA2     ------> OPAMP1_VOUT
        PA3     ------> OPAMP1_VINM
        */
        GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USER CODE BEGIN OPAMP1_MspInit 1 */

        /* USER CODE END OPAMP1_MspInit 1 */
    }
    else if (hopamp->Instance == OPAMP2)
    {
        /* USER CODE BEGIN OPAMP2_MspInit 0 */

        /* USER CODE END OPAMP2_MspInit 0 */

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**OPAMP2 GPIO Configuration
        PA5     ------> OPAMP2_VINM
        PA6     ------> OPAMP2_VOUT
        PA7     ------> OPAMP2_VINP
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USER CODE BEGIN OPAMP2_MspInit 1 */

        /* USER CODE END OPAMP2_MspInit 1 */
    }
    else if (hopamp->Instance == OPAMP3)
    {
        /* USER CODE BEGIN OPAMP3_MspInit 0 */

        /* USER CODE END OPAMP3_MspInit 0 */

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**OPAMP3 GPIO Configuration
        PB0     ------> OPAMP3_VINP
        PB1     ------> OPAMP3_VOUT
        PB2     ------> OPAMP3_VINM
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN OPAMP3_MspInit 1 */

        /* USER CODE END OPAMP3_MspInit 1 */
    }
}

/**
 * @brief OPAMP MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hopamp-> OPAMP handle pointer
 * @retval None
 */
void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef *hopamp)
{
    if (hopamp->Instance == OPAMP1)
    {
        /* USER CODE BEGIN OPAMP1_MspDeInit 0 */

        /* USER CODE END OPAMP1_MspDeInit 0 */

        /**OPAMP1 GPIO Configuration
        PA1     ------> OPAMP1_VINP
        PA2     ------> OPAMP1_VOUT
        PA3     ------> OPAMP1_VINM
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

        /* USER CODE BEGIN OPAMP1_MspDeInit 1 */

        /* USER CODE END OPAMP1_MspDeInit 1 */
    }
    else if (hopamp->Instance == OPAMP2)
    {
        /* USER CODE BEGIN OPAMP2_MspDeInit 0 */

        /* USER CODE END OPAMP2_MspDeInit 0 */

        /**OPAMP2 GPIO Configuration
        PA5     ------> OPAMP2_VINM
        PA6     ------> OPAMP2_VOUT
        PA7     ------> OPAMP2_VINP
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

        /* USER CODE BEGIN OPAMP2_MspDeInit 1 */

        /* USER CODE END OPAMP2_MspDeInit 1 */
    }
    else if (hopamp->Instance == OPAMP3)
    {
        /* USER CODE BEGIN OPAMP3_MspDeInit 0 */

        /* USER CODE END OPAMP3_MspDeInit 0 */

        /**OPAMP3 GPIO Configuration
        PB0     ------> OPAMP3_VINP
        PB1     ------> OPAMP3_VOUT
        PB2     ------> OPAMP3_VINM
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

        /* USER CODE BEGIN OPAMP3_MspDeInit 1 */

        /* USER CODE END OPAMP3_MspDeInit 1 */
    }
}

/**
 * Enable DMA controller clock
 */
void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    // enableInterruptWithPrio(DMA1_Channel1_IRQn, 0);
    // enableInterruptWithPrio(DMA1_Channel2_IRQn, 0);

    // Enable external clock for ADC12
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

void MX_DMA1_Init(ADC_HandleTypeDef *hadc, DMA_HandleTypeDef *hdma_adc, DMA_Channel_TypeDef *channel, uint32_t request)
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
        Serial.printf("HAL_DMA_Init failed!\n");
        Error_Handler();
    }
    __HAL_LINKDMA(hadc, DMA_Handle, *hdma_adc);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
void MX_ADC1_Init(ADC_HandleTypeDef *hadc1)
{
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Common config
     */
    hadc1->Instance = ADC1;
    hadc1->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
    hadc1->Init.Resolution = ADC_RESOLUTION_12B;
    hadc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1->Init.GainCompensation = 0;
    hadc1->Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1->Init.LowPowerAutoWait = DISABLE;
    hadc1->Init.ContinuousConvMode = DISABLE;
    hadc1->Init.NbrOfConversion = 5;
    hadc1->Init.DiscontinuousConvMode = DISABLE;
    hadc1->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc1->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1->Init.DMAContinuousRequests = ENABLE;
    hadc1->Init.Overrun = ADC_OVR_DATA_PRESERVED;

    if (HAL_ADC_Init(hadc1) != HAL_OK)
    {
        Serial.printf("HAL_ADC_Init failed!\n");
        Error_Handler();
    }

    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(hadc1, &multimode) != HAL_OK)
    {
        Serial.printf("HAL_ADCEx_MultiModeConfigChannel failed!");
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_12; // ADC1_IN12 = PB1 = OP3_OUT
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
    {
        Serial.printf("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_3; // ADC1_IN3 = PA2 = OP1_OUT
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
    {
        Serial.printf("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }

    //******************************************************************
    // Temp, Poti ....
    /* Configure Regular Channel (PB12, Potentiometer)
     */
    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
    {
        Serial.printf("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }

    /** Configure Regular Channel (PB14, Temperature)
     */
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
    {
        Serial.printf("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }

    /** Configure Regular Channel (PA0, vbus)
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_5;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
    {
        Serial.printf("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
void MX_ADC2_Init(ADC_HandleTypeDef *hadc2)
{
    /* USER CODE BEGIN ADC2_Init 0 */

    /* USER CODE END ADC2_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC2_Init 1 */

    /* USER CODE END ADC2_Init 1 */
    /** Common config
     */
    hadc2->Instance = ADC2;
    hadc2->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
    hadc2->Init.Resolution = ADC_RESOLUTION_12B;
    hadc2->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2->Init.GainCompensation = 0;
    hadc2->Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc2->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc2->Init.LowPowerAutoWait = DISABLE;
    hadc2->Init.ContinuousConvMode = DISABLE;
    hadc2->Init.NbrOfConversion = 1;
    hadc2->Init.DiscontinuousConvMode = DISABLE;
    hadc2->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc2->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2->Init.DMAContinuousRequests = ENABLE;
    hadc2->Init.Overrun = ADC_OVR_DATA_PRESERVED;

    if (HAL_ADC_Init(hadc2) != HAL_OK)
    {
        Serial.printf("HAL_ADC_Init failed!\n");
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_3; // ADC2_IN3 = PA6
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
    {
        Serial.printf("HAL_ADC_ConfigChannel failed!");
        Error_Handler();
    }
    /* USER CODE BEGIN ADC2_Init 2 */

    /* USER CODE END ADC2_Init 2 */
}

void initADC()
{
    __HAL_RCC_ADC12_CLK_ENABLE();

    MX_ADC1_Init(&bsp.adc1);
    MX_ADC2_Init(&bsp.adc2);

    // Calibrate the ADCs
    HAL_StatusTypeDef status;
    status = HAL_ADCEx_Calibration_Start(&bsp.adc1, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        Serial.printf("ADC calibration failed: %i\n", status);
        Error_Handler();
    }
    status = HAL_ADCEx_Calibration_Start(&bsp.adc2, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        Serial.printf("ADC calibration failed: %i\n", status);
        Error_Handler();
    }
}

void calibrateCurrentSenseOffsets() {
    float shunt = 0.003f;
    float gain = -64.0f / 7.0f;

    uint32_t a_sum = 0;
    uint32_t b_sum = 0;
    uint32_t c_sum = 0;
    int NUM_TRIALS = 1000;
    for (int i = 0; i < NUM_TRIALS; i++) {
        delayMicroseconds(50);
        a_sum += bsp.current_a;
        b_sum += bsp.current_b;
        c_sum += bsp.current_c;
    }

    a_sum /= NUM_TRIALS;
    b_sum /= NUM_TRIALS;
    c_sum /= NUM_TRIALS;

    float factor = ADC_VOLTAGE / ADC_SCALE;

    Serial.printf("avg voltage a: %f\r\n", a_sum * factor);
    Serial.printf("avg voltage b: %f\r\n", b_sum * factor);
    Serial.printf("avg voltage c: %f\r\n", c_sum * factor);

    bsp.current_a_offset = a_sum;
    bsp.current_b_offset = b_sum;
    bsp.current_c_offset = c_sum;
}

extern "C"
{
    // void DMA1_Channel1_IRQHandler(void)
    // {
    //     HAL_DMA_IRQHandler(&bsp.dma1);
    // }

    void TIM1_UP_TIM16_IRQHandler(void)
    {
        pwm_timer->SR &= ~TIM_SR_UIF;
        if (bsp.pwm_callback)
        {
            bsp.pwm_callback();
        }
    }
}


void BSP_Init()
{
    initGPIO();
    initPwm();
    MX_DMA_Init();
    initADC();
    initOpamp();

    MX_DMA1_Init(&bsp.adc1, &bsp.dma1, DMA1_Channel1, DMA_REQUEST_ADC1);
    MX_DMA1_Init(&bsp.adc2, &bsp.dma2, DMA1_Channel2, DMA_REQUEST_ADC2);

    HAL_StatusTypeDef status;
    status = HAL_ADC_Start_DMA(&bsp.adc1, (uint32_t *)bsp.adc1_buffer, sizeof(bsp.adc1_buffer) / 2);
    if (status != HAL_OK)
    {
        Serial.printf("DMA start adc2 failed, %i\n", status);
        Error_Handler();
    }

    status = HAL_ADC_Start_DMA(&bsp.adc2, (uint32_t *)bsp.adc2_buffer, sizeof(bsp.adc2_buffer) / 2);
    if (status != HAL_OK)
    {
        Serial.printf("DMA start adc3 failed, %i\n", status);
        Error_Handler();
    }

    // enable tim dma trigger
    LL_TIM_SetTriggerOutput(pwm_timer, LL_TIM_TRGO_UPDATE);

    // start the timer
    pwm_timer->CR1 |= TIM_CR1_CEN;

    calibrateCurrentSenseOffsets();
}

void BSP_EnableOutputs()
{
    pwm_timer->BDTR |= TIM_BDTR_MOE;
}

void BSP_DisableOutputs()
{
    pwm_timer->BDTR &= ~TIM_BDTR_MOE;
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
    uint32_t ccr1 = constrain(c, 0, 1) * arr;
    uint32_t ccr2 = constrain(a, 0, 1) * arr;
    uint32_t ccr3 = constrain(b, 0, 1) * arr;
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

Vec3f BSP_ReadPhaseCurrents3()
{
    float shunt = 0.003f;
    float gain = -64.0f / 7.0f;
    float factor = ADC_VOLTAGE / ADC_SCALE / gain / shunt;

    return Vec3f(
        (bsp.current_a - bsp.current_a_offset) * factor,
        (bsp.current_b - bsp.current_b_offset) * factor,
        (bsp.current_c - bsp.current_c_offset) * factor);
}

float BSP_ReadPotentiometer()
{
    float factor = ADC_VOLTAGE / ADC_SCALE;
    return bsp.potentiometer * factor;
}

float BSP_ReadTemperatureOnboardNTC()
{
    float factor = ADC_VOLTAGE / ADC_SCALE;
    return ntc_voltage_to_temp(bsp.onboard_ntc * factor);
}

float BSP_ReadVBus()
{
    float multiplier = (18 + 169) / 18.f;
    float factor = ADC_VOLTAGE / ADC_SCALE * multiplier;
    return bsp.vbus * factor;
}

void BSP_AdjustCurrentSenseOffsets()
{
    // automatically correct for drift in the current sense circuit.
    float gain = 1.f/1000;
    bsp.current_a_offset += (bsp.current_a - bsp.current_a_offset) * gain;
    bsp.current_b_offset += (bsp.current_b - bsp.current_b_offset) * gain;
    bsp.current_c_offset += (bsp.current_c - bsp.current_c_offset) * gain;
}

void BSP_WriteStatusLED(bool on)
{
    digitalWrite(LED_BUILTIN, on);
}

// float BSP_ReadTemperatureInternal()
// {
//     float ts_data = bsp.v_ts * (3.272f / TEMPSENSOR_CAL_VREFANALOG * 1000);
//     float offset = TEMPSENSOR_CAL1_TEMP;
//     float slope = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / float((*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR));
//     float temp = offset + (ts_data - *TEMPSENSOR_CAL1_ADDR) * slope;
//     return temp;
// }

// float BSP_ReadChipAnalogVoltage()
// {
//     return __LL_ADC_CALC_VREFANALOG_VOLTAGE(bsp.vrefint, LL_ADC_RESOLUTION_12B) * 0.001f;
// }