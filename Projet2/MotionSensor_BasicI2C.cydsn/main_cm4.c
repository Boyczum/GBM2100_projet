/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "bmi160.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

static struct bmi160_dev bmi160Dev;

static int8_t  Write_I2C(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    Cy_SCB_I2C_MasterSendStart(I2C_HW, dev_addr,CY_SCB_I2C_WRITE_XFER,0,&I2C_context); //Mode écriture sur le capteur
    Cy_SCB_I2C_MasterWriteByte(I2C_HW, reg_addr,0,&I2C_context); //Précision du registre
    for(int i = 0; i< len;i++)
    {
        Cy_SCB_I2C_MasterWriteByte(I2C_HW, data[i],0,&I2C_context); //Écriture dans le registre       
    }
    Cy_SCB_I2C_MasterSendStop(I2C_HW,0,&I2C_context); //arrêt
    
    return 0;
}

static int8_t  Read_I2C(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    Cy_SCB_I2C_MasterSendStart(I2C_HW, dev_addr,CY_SCB_I2C_WRITE_XFER,0,&I2C_context); //Mode écriture sur le capteur
    Cy_SCB_I2C_MasterWriteByte(I2C_HW, reg_addr,0,&I2C_context); //Précision du registre
    Cy_SCB_I2C_MasterSendReStart(I2C_HW, dev_addr,CY_SCB_I2C_READ_XFER,0,&I2C_context); //Mode écriture sur le capteur
    for(int i = 0; i< len-1;i++)
    {
        Cy_SCB_I2C_MasterReadByte(I2C_HW,CY_SCB_I2C_ACK, &data[i],0,&I2C_context); //Lecture dans le registre       
    }
    Cy_SCB_I2C_MasterReadByte(I2C_HW,CY_SCB_I2C_NAK, &data[len-1],0,&I2C_context); //Fin de lecture dans le registre       
    Cy_SCB_I2C_MasterSendStop(I2C_HW,0,&I2C_context); //arrêt
    
    return 0;
}

static void bmi160Init(void)
{   
    vTaskDelay(100);
    
    bmi160Dev.read = (bmi160_com_fptr_t)Read_I2C;
    bmi160Dev.write = (bmi160_com_fptr_t)Write_I2C;
    bmi160Dev.delay_ms = (bmi160_delay_fptr_t)vTaskDelay;
    bmi160Dev.id = BMI160_I2C_ADDR; //Adresse du capteur
    
    bmi160_init(&bmi160Dev);//Initialiser le capteur
    
    /* Select the Output data rate, range of accelerometer sensor */
    bmi160Dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi160Dev.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi160Dev.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;

    /* Select the power mode of accelerometer sensor */
    bmi160Dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160Dev.gyro_cfg.odr = BMI160_GYRO_ODR_1600HZ;
    bmi160Dev.gyro_cfg.range = BMI160_GYRO_RANGE_125_DPS;
    bmi160Dev.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160Dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    bmi160_set_sens_conf(&bmi160Dev);
    bmi160Dev.delay_ms(50);
}

void motionTask(void *arg)
{
    (void)arg;
    
    I2C_Start();
    bmi160Init();
    struct bmi160_sensor_data acc;
    float gx, gy, gz;
    
    for(;;)
    {
        bmi160_get_sensor_data(BMI160_ACCEL_ONLY,&acc,NULL, &bmi160Dev);
    }
    gx = acc.x;
    gy = acc.y;
    gz = acc.z;
    
    printf("ax = %1.2f,ay = %1.2f,az = %1.2f\r\n",gx,gy,gz);
    
    vTaskDelay(200);

}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    
    UART_Start();
    //I2C_Start();
    
    //UART_PutString("Acceleration du BMI160\r\n");
    printf("Acceleration du BMI160\r\n");
    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    
    
    xTaskCreate(motionTask,"motion task",400,0,1,0);
    vTaskStartScheduler();
    
    
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
