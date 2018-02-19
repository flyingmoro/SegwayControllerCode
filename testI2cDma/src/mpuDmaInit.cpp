#include "mpuDmaInit.h"


void MPU_GPIO_Init() {

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void MPU_DMA_Init(uint32_t rx_buffer) {
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_i2c2_rx.Instance = DMA1_Stream2;
    // hdma_i2c2_rx.DMA_MemoryBaseAddr = rx_buffer;
    hdma_i2c2_rx.Init.Channel = DMA_CHANNEL_7;
    hdma_i2c2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c2_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_i2c2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_i2c2_rx);
}

void MPU_I2C_Init() {

    __HAL_RCC_I2C2_CLK_ENABLE();

    i2cHandle.Instance = I2C2;
    i2cHandle.Init.Timing = 400000;
    i2cHandle.Init.OwnAddress1 = 0;
    i2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2cHandle.Init.OwnAddress2 = 0;
    i2cHandle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    i2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&i2cHandle);

    // Configure Analogue filter
    HAL_I2CEx_ConfigAnalogFilter(&i2cHandle, I2C_ANALOGFILTER_ENABLE);

    //**Configure Digital filter
    HAL_I2CEx_ConfigDigitalFilter(&i2cHandle, 0);
}


void MPU_NVIC_Init() {
    /* DMA interrupt init */
    /* DMA1_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

    /* I2C2 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

}

void MPU_POST_Init(uint8_t *hdmarx) {
    __HAL_LINKDMA(i2cHandle, hdmarx, hdma_i2c2_rx);
}



// void MPU6050_Setup_DMA() {
//     NVIC_InitTypeDef NVIC_InitStructure;
//     DMA_InitTypeDef  DMA_InitStructure;
//
//     DMA_DeInit(MPU6050_DMA_Channel); //reset DMA1 channe1 to default values;
//
//     DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C_DR_Address; //=0x40005410 : address of data reading register of I2C1
//     DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)I2C_Rx_Buffer; //variable to store data
//     DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //channel will be used for peripheral to memory transfer
//     DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;    //setting normal mode (non circular)
//     DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;    //medium priority
//     DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;    //Location assigned to peripheral register will be source
//     DMA_InitStructure.DMA_BufferSize = 14;    //number of data to be transfered
//     DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //automatic memory increment disable for peripheral
//     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;    //automatic memory increment enable for memory
//     DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;    //source peripheral data size = 8bit
//     DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    //destination memory data size = 8bit
//     DMA_Init(MPU6050_DMA_Channel, &DMA_InitStructure);
//     DMA_ITConfig(MPU6050_DMA_Channel, DMA_IT_TC, ENABLE);
//
//     NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn; //I2C1 connect to channel 7 of DMA1
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&NVIC_InitStructure);
// }
