

#include <mbed.h>
#include <stdint.h>

Serial pc(USBTX, USBRX);

#define ENCODER_COUNT 4096
#define ONE_6_ENCODER_COUNT 170

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

// void TIM3_IRQHandler(void);


static void initSpaceVectorPWM(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __TIM2_CLK_ENABLE();

    // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // TIM_ClockConfigTypeDef sClockSourceConfig;
    // TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 19;
    htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    htim2.Init.Period = 99;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_Base_Init(&htim2);


    // sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    // HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    // HAL_TIM_PWM_Init(&htim2);

    // sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    // sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    // HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 0;
    // can invert the output signal
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;


    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    // HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);



    // das hier is wichtig und kommt nicht aus dem StmCube Dings raus !!!!!!!!!!!!!!!
    // (damit wird auch das preload Verhalten angeschaltet)
    HAL_TIM_Base_Start_IT(&htim2);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    // __HAL_TIM_ENABLE(&htim2);
}

void initEncoderCounter(){

    // muss das ?
    // __HAL_RCC_TIM3_CLK_ENABLE();  // steht jetzt in Gpio-Init
    // __TIM3_CLK_ENABLE();

    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;
    TIM_OC_InitTypeDef sConfigOC;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = ENCODER_COUNT;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(&htim3);

    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    HAL_TIM_Encoder_Init(&htim3, &sConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);

    // activate channel 4 capture/compare mode
    //TIM3->DIER |= TIM_DIER_CC4IE;
    sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
    // sConfigOC.Pulse = 0;
    // sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    // sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

    // manuell hinzugefügt
    HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim3);
}

void initTimerInterrupt() {

}


static void initGPIO(void) {
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __GPIOE_CLK_ENABLE();

    __HAL_RCC_TIM3_CLK_ENABLE();

    // space vector pwm ports
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // encoder ports
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

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    // GPIO_InitTypeDef GPIO_InitStructEncoders;
    // GPIO_InitStructEncoders.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    // GPIO_InitStructEncoders.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStructEncoders.Pull = GPIO_NOPULL;
    // GPIO_InitStructEncoders.Speed = GPIO_SPEED_FREQ_LOW;
    // GPIO_InitStructEncoders.Alternate = GPIO_AF2_TIM4;
    // HAL_GPIO_Init(GPIOB, &GPIO_InitStructEncoders);

    // test Pin
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}


// void initPwmViaRegisters() {
//     // set pwm mode 1 OC1M = 0110
//     TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
//
//     // set Pwm-Mode to center aligned with trigger on way up and down
//     TIM2->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CMS_1;
//
//     // enable preload function or not
//     TIM2->CR1 &= ~TIM_CR1_ARPE;
//
//     // enable channels
//     TIM2->CCER |= TIM_CCER_CC3E;
//     TIM2->CCER |= TIM_CCER_CC4E;
//
//     // set Prescaler
//     TIM2->PSC = (uint16_t)2;
//
//     // max count value for the timer
//     TIM2->ARR = (uint16_t)8399;
//
//     // set pwm percentage
//     TIM2->CCR3 = 4000;
//     TIM2->CCR4 = 3000;
//
//     // write preloaded bits into shadow bits
//     TIM2->EGR |= TIM_EGR_UG;
//
//     // start timer
//     TIM2->CR1 |= TIM_CR1_CEN;
//
//     // write preloaded bits into shadow bits
//     TIM2->EGR |= TIM_EGR_UG;
//
// }

// void initGPIOViaRegisters() {
//     // configure GPIO for PWM output
//
//     // set alternate function mode AF1
//     GPIOB->MODER |= GPIO_MODER_MODER2_1;
//     GPIOB->MODER |= GPIO_MODER_MODER3_1;
//
//     // set alternate function AF1
//     // GPIOB->AFRH |= GPIO_AFRH_AFRH2_0;
//     // GPIOB->AFR[1] |= GPIO_AFRH_AFRH3_0;
//
//     GPIOB->AFR[1] |= GPIO_AFRH_AFRH2_0;
//     GPIOB->AFR[1] |= GPIO_AFRH_AFRH3_0;
//
//
//
// }


/**
* @brief This function handles TIM3 global interrupt.
*/
extern "C" void TIM3_IRQHandler(void);
void TIM3_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim3);
    // HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
    /* USER CODE END TIM3_IRQn 0 */
    // HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */

    /* USER CODE END TIM3_IRQn 1 */
}

volatile int ISR_currentMotorSector = 0;
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    // ISR_currentMotorSector++;
    if (htim->Instance == TIM3) {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
        bool isCountingDown = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);
        if (isCountingDown) {
            ISR_currentMotorSector--;
        }
        else {
            ISR_currentMotorSector++;
        }
    //
    //     if (ISR_currentMotorSector > 5) {
    //         ISR_currentMotorSector = 0;
    //     }
    //     if (ISR_currentMotorSector < 0) {
    //         ISR_currentMotorSector = 5;
    //     }
    //
    //     HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
    //
    }
}


// extern "C" void HAL_TIM_OC_DelayElapsedCallback (TIM_HandleTypeDef * htim);
// void HAL_TIM_OC_DelayElapsedCallback (TIM_HandleTypeDef * htim) {
//     HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
// }

#define TEST_CYCLE 50
int main(void) {

    initGPIO();
    initSpaceVectorPWM();
    initEncoderCounter();
    initTimerInterrupt();

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

    int sectorCounter = 0;
    while (1) {

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, TEST_CYCLE);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, TEST_CYCLE);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

        continue;

        switch (sectorCounter) {
            case 0:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, TEST_CYCLE);
                break;
            case 1:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, TEST_CYCLE);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, TEST_CYCLE);
                break;
            case 2:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, TEST_CYCLE);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                break;
            case 3:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, TEST_CYCLE);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, TEST_CYCLE);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                break;
            case 4:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, TEST_CYCLE);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                break;
            case 5:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, TEST_CYCLE);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, TEST_CYCLE);
                break;
            default:
                sectorCounter = 0;
                break;

        }
        sectorCounter += 1;
        if (sectorCounter > 5) {
            sectorCounter = 0;
        }

        wait(1.0f);


        continue;





        int i = -1;
        for (i = -1; i < 10000; i+=1000) {
            if (i == -1) {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
            }
            else {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
            }
            uint16_t encoderPosition = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
            pc.printf("Encoder: %u, Sector: %i\n", encoderPosition, ISR_currentMotorSector);
            // HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
            wait(1.0f);
        }
    }
}
