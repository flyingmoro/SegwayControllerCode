#include "mpuDma.h"
#include "mpuDmaInit.h"
#include "MPU_REGISTERS.h"
#include "mbed.h"

// I2C i2c(PB_11, PB_10);

void I2C_DMA_Read(uint8_t slaveAddr, uint8_t readAddr, uint8_t sensor);
void MPU_Initialize();

MpuData IT_mpuData;
uint8_t mpu_rx_buffer[14];

I2C_HandleTypeDef MPU6050_I2C;

void initMPU6050() {

    // configure stm32 peripherals
    MPU_GPIO_Init();
    MPU_DMA_Init();
    MPU_I2C_Init();
    MPU_NVIC_Init();
    MPU_POST_Init(mpu_rx_buffer);

    // configure mpu
    MPU_Initialize();

    // now the mpu should run mostly in the background and data will be available
    // via the getMpuData function
}


MpuData getMpuData() {
    __disable_irq();
    MpuData tempData = IT_mpuData;
    tempData.rawAngularRate_alpha = IT_mpuData.rawAngularRate_alpha;
    __enable_irq();
    return tempData;
}



// on receive of "data available trigger signal" from mpu send i2c request for data transmission
void EXTI4_IRQHandler(void) {
    if (EXTI_GetITStatus(MPU6050_INT_Exti)) {           //MPU6050_INT
        EXTI_ClearITPendingBit(MPU6050_INT_Exti);
        I2C_DMA_Read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, MPU6050);
    }
}



void I2C_DMA_Read(uint8_t slaveAddr, uint8_t readAddr, uint8_t sensor) {
    /* Disable DMA channel*/
    DMA_Cmd(MPU6050_DMA_Channel, DISABLE);
    /* Set current data number again to 14 for MPu6050, only possible after disabling the DMA channel */
    DMA_SetCurrDataCounter(MPU6050_DMA_Channel, 14);

    /* While the bus is busy */
    while(I2C_GetFlagStatus(i2cHandle, I2C_FLAG_BUSY));

    /* Enable DMA NACK automatic generation */
    I2C_DMALastTransferCmd(i2cHandle, ENABLE);                    //Note this one, very important

    /* Send START condition */
    I2C_GenerateSTART(i2cHandle, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(i2cHandle, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(i2cHandle, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(i2cHandle, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(i2cHandle, ENABLE);

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(i2cHandle, readAddr);

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(i2cHandle, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send START condition a second time */
    I2C_GenerateSTART(i2cHandle, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(i2cHandle, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for read */
    I2C_Send7bitAddress(i2cHandle, slaveAddr, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(i2cHandle, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* Start DMA to receive data from I2C */
    DMA_Cmd(MPU6050_DMA_Channel, ENABLE);
    I2C_DMACmd(i2cHandle, ENABLE);

    // When the data transmission is complete, it will automatically jump to DMA interrupt routine to finish the rest.
    //now go back to the main routine
}





// called on receive of all 14 bytes from mpu
void DMA1_Stream2_IRQHandler(void) {
    if (DMA_GetFlagStatus(DMA1_FLAG_TC7)) {

        /* Clear transmission complete flag */
        DMA_ClearFlag(DMA1_FLAG_TC7);

        I2C_DMACmd(i2cHandle, DISABLE);
        /* Send I2C1 STOP Condition */
        I2C_GenerateSTOP(i2cHandle, ENABLE);
        /* Disable DMA channel*/
        DMA_Cmd(MPU6050_DMA_Channel, DISABLE);

        //Read Accel data from byte 0 to byte 2
        for(i=0; i<3; i++) {
            AccelGyro[i]=((int16_t)((u16)I2C_Rx_Buffer[2*i] << 8) + I2C_Rx_Buffer[2*i+1]);
        }

        //Skip byte 3 of temperature data

        //Read Gyro data from byte 4 to byte 6
        for(i=4; i<7; i++) {
            AccelGyro[i-1]=((int16_t)((u16)I2C_Rx_Buffer[2*i] << 8) + I2C_Rx_Buffer[2*i+1]);
        }
    }
}












void MPU_Initialize() {
    //reset the whole module first
    MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1<<7);

    //wait for 50ms for the gyro to stabilize
    wait(0.05);

    //PLL with Z axis gyroscope reference
    MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO);

    //DLPF_CFG = 1: Fs=1khz; bandwidth=42hz
    MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x01);

    //500Hz sample rate ~ 2ms
    MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x01);

    //Gyro full scale setting
    MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000);

    //Accel full scale setting
    MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_16);

    //interrupt status bits are cleared on any read operation
    MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1<<4);

    //interupt occurs when data is ready. The interupt routine is in the receiver.c file.
    MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 1<<0);

    //reset gyro and accel sensor
    MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x07);
}


void MPU6050_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t data) {
    uint8_t tmp;
    tmp = data;
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);
}

void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr)
{
    /* Send START condition */
    I2C_GenerateSTART(i2cHandle, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(i2cHandle, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(i2cHandle, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(i2cHandle, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(i2cHandle, writeAddr);

    /* Test on EV8 and clear it */
    //while(!I2C_CheckEvent(i2cHandle, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
    /* Send the byte to be written */
    if (pBuffer!=0) I2C_SendData(i2cHandle, pBuffer);

    /* Test on EV8_2 and clear it */
    while(!I2C_CheckEvent(i2cHandle, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(i2cHandle, ENABLE);
}
