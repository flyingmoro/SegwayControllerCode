#include "encoderHandling.h"

#include "mbed.h"

// #define ENCODER_COUNT 8192
#define ENCODER_COUNT 65535

TIM_HandleTypeDef encoderTimerLeft;
TIM_HandleTypeDef encoderTimerRight;

void initEncoderCounter();
static void initGPIO(void);



void updatePosition(Position *worldPosition) {
    worldPosition->x = (uint16_t)__HAL_TIM_GET_COUNTER(&encoderTimerLeft);
    worldPosition->y = (uint16_t)__HAL_TIM_GET_COUNTER(&encoderTimerRight);
}

void initEncoder() {
    initGPIO();
    initEncoderCounter();
}

void initEncoderCounter(){

    // muss das ?
    // __HAL_RCC_TIM3_CLK_ENABLE();  // steht jetzt in Gpio-Init
    // __TIM3_CLK_ENABLE();

    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;
    // TIM_OC_InitTypeDef sConfigOC;

    encoderTimerLeft.Instance = TIM3;
    encoderTimerLeft.Init.Prescaler = 0;
    encoderTimerLeft.Init.CounterMode = TIM_COUNTERMODE_UP;
    encoderTimerLeft.Init.Period = ENCODER_COUNT;
    encoderTimerLeft.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    encoderTimerLeft.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(&encoderTimerLeft);

    encoderTimerRight.Instance = TIM4;
    encoderTimerRight.Init.Prescaler = 0;
    encoderTimerRight.Init.CounterMode = TIM_COUNTERMODE_UP;
    encoderTimerRight.Init.Period = ENCODER_COUNT;
    encoderTimerRight.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    encoderTimerRight.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(&encoderTimerRight);

    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    HAL_TIM_Encoder_Init(&encoderTimerLeft, &sConfig);
    HAL_TIM_Encoder_Init(&encoderTimerRight, &sConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&encoderTimerLeft, &sMasterConfig);
    HAL_TIMEx_MasterConfigSynchronization(&encoderTimerRight, &sMasterConfig);

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&encoderTimerLeft, &sConfigIC, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&encoderTimerLeft, &sConfigIC, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&encoderTimerRight, &sConfigIC, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&encoderTimerRight, &sConfigIC, TIM_CHANNEL_2);

    // // activate channel 4 capture/compare mode
    // //TIM3->DIER |= TIM_DIER_CC4IE;
    // sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
    // // sConfigOC.Pulse = 0;
    // // sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    // // sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    // HAL_TIM_OC_ConfigChannel(&encoderTimerLeft, &sConfigOC, TIM_CHANNEL_4);

    // manuell hinzugefügt
    HAL_TIM_Encoder_Start_IT(&encoderTimerLeft, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&encoderTimerLeft);
    HAL_TIM_Encoder_Start_IT(&encoderTimerRight, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&encoderTimerRight);
}


static void initGPIO(void) {
    /* GPIO Ports Clock Enable */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __GPIOE_CLK_ENABLE();

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();



    // encoder ports

    // ports fors TIM3
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ports fors TIM4
    GPIO_InitTypeDef GPIO_InitStructB;
    GPIO_InitStructB.Pin = GPIO_PIN_12;
    GPIO_InitStructB.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructB.Pull = GPIO_NOPULL;
    GPIO_InitStructB.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructB.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructB);

    GPIO_InitStructB.Pin = GPIO_PIN_13;
    GPIO_InitStructB.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructB.Pull = GPIO_NOPULL;
    GPIO_InitStructB.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructB.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructB);


    /* TIM3 interrupt Init */
    // HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(TIM3_IRQn);


}
