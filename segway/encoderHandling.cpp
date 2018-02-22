#include "encoderHandling.h"

#include "mbed.h"

// #define ENCODER_COUNT 8192
#define ENCODER_COUNT 65535

TIM_HandleTypeDef encoderTimerLeft;
TIM_HandleTypeDef encoderTimerRight;

#define DELTA_T 0.001

int32_t encoderChange(int32_t oldEncValue, int32_t newEncValue);
void initEncoderCounter();
static void initGPIO(void);


int32_t oldEncLeft = 0;
int32_t oldEncRight = 0;
void updatePosition(Position *worldPosition) {
    static float xRLeft, xRRight, diffX, deltaForward, deltaGamma;

    // read the hardware counter
    worldPosition->encLeft = (int32_t)__HAL_TIM_GET_COUNTER(&encoderTimerLeft);
    worldPosition->encRight = (int32_t)__HAL_TIM_GET_COUNTER(&encoderTimerRight);

    // calculate the deltas
    xRLeft = (float)encoderChange(oldEncLeft, worldPosition->encLeft) * METERS_PER_ENCODER_STEP;
    xRRight = (float)encoderChange(oldEncRight, worldPosition->encRight) * METERS_PER_ENCODER_STEP;

    // see documentation concerning the following formulas
    diffX = (float)(xRRight - xRLeft);
    deltaGamma = diffX / WHEEL_SPAN;
    deltaForward = (xRLeft + xRRight) / 2.0;

    worldPosition->x += (cos(worldPosition->gamma) * deltaForward);
    worldPosition->y += (sin(worldPosition->gamma) * deltaForward);
    worldPosition->forwardSpeed = deltaForward / DELTA_T;
    worldPosition->gamma += deltaGamma;
    worldPosition->gammaP = deltaGamma / DELTA_T;


    // constrain gamma
    while (worldPosition->gamma > PI) {
      worldPosition->gamma -= TWO_PI;
    }
    while (worldPosition->gamma <= NEG_PI) {
      worldPosition->gamma += TWO_PI;
    }

    oldEncLeft = worldPosition->encLeft;
    oldEncRight = worldPosition->encRight;
}

int32_t encoderChange(int32_t oldEncValue, int32_t newEncValue) {
    // encoder counter has a resolution of approx 8 full turns
    // assuming that within 1ms the motor never turns 8 times

    // check if counter has passed its limit
    if (oldEncValue > 50000 && newEncValue < 10000) {
        return newEncValue + 65536 - oldEncValue;
    } else if (newEncValue > 50000 && oldEncValue < 10000) {
        return oldEncValue + 65536 - newEncValue;
    } else {
        return newEncValue - oldEncValue;
    }



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

    // // activate channel 4 capture/compare mode for interrupt on encoder index
    // sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
    // // sConfigOC.Pulse = 0;
    // // sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    // // sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    // HAL_TIM_OC_ConfigChannel(&encoderTimerLeft, &sConfigOC, TIM_CHANNEL_4);

    // manuell hinzugefügt
    HAL_TIM_Encoder_Start(&encoderTimerLeft, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start(&encoderTimerLeft);
    HAL_TIM_Encoder_Start(&encoderTimerRight, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start(&encoderTimerRight);
    // HAL_TIM_Encoder_Start_IT(&encoderTimerLeft, TIM_CHANNEL_ALL);
    // HAL_TIM_Base_Start_IT(&encoderTimerLeft);
    // HAL_TIM_Encoder_Start_IT(&encoderTimerRight, TIM_CHANNEL_ALL);
    // HAL_TIM_Base_Start_IT(&encoderTimerRight);
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


    // timer clock enable
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
