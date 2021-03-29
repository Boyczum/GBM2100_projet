/*


Notes : RTOS et I/O doivent être activées dans la PDL (projet --> build settings --> pdl --> cocher)
*/

//Includes
	#include "project.h"
	#include "bmi160.h"
	#include "FreeRTOS.h"
	#include "task.h"
	#include <stdio.h>
	#include "semphr.h"

//Définitions 

	/* Priority of user tasks in this project */
	#define TASK_MOTION_SENSOR_PRIORITY (configMAX_PRIORITIES - 1)

	/* Stack size of user tasks in this project */
	#define TASK_MOTION_SENSOR_STACK_SIZE   (configMINIMAL_STACK_SIZE)

	/* Semaphore handle for I2C */
	SemaphoreHandle_t xSemaphoreI2C;   

	/* Taille du buffer I2C */
	#define I2C_BUFFER_SIZE (10u)
    
	//Structure contenant l'information du BMI160
	static struct bmi160_dev sensor; 
	
	//Handler pour la tâche du capteur de mouvement
	extern TaskHandle_t xTaskHandleMotion; 

//Fonctions et tâches

	//Fonction d'écriture I2C
	unsigned int I2C_WriteBytes(uint8_t Address, uint8_t RegisterAddr, uint8_t *RegisterValue, uint8_t RegisterLen)
	{
		/* Variable used for status of I2C operation */
		unsigned int status;
		
		/* Temporary buffer used for I2C transfer */ 
		static uint8_t tempBuff[I2C_BUFFER_SIZE];
		tempBuff[0] = RegisterAddr;
		memcpy(tempBuff+1, RegisterValue, RegisterLen);

		/* Local variables for storing I2C Master transfer configuration structure */
		cy_stc_scb_i2c_master_xfer_config_t config = 
		{  
			.slaveAddress = (uint32)Address,
			.buffer =  tempBuff,
			.bufferSize = RegisterLen+1,
			.xferPending = false
		};
		
		/* Start I2C write and take the semaphore */
		status = (unsigned int)I2C_MasterWrite(&config);
		xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY);
			
		return status;
	}

	//Fonction de lecture I2C
	unsigned int I2C_ReadBytes(uint8_t Address, uint8_t RegisterAddr, uint8_t *RegisterValue, uint8_t RegisterLen)
	{
		/* Variable used for status of I2C operation */
		static unsigned int status;
		
		/* Local variables for storing I2C Master transfer configuration structure */
		cy_stc_scb_i2c_master_xfer_config_t config;
		
		config.slaveAddress = (uint32)Address;
		config.buffer       = &RegisterAddr;
		config.bufferSize   = 1;
		config.xferPending  = true;
		
		/* Start I2C write and take the semaphore */
		status = (unsigned int)I2C_MasterWrite(&config);
		xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY);
		
		if(status == CY_SCB_I2C_SUCCESS)
		{
			config.slaveAddress = (uint32)Address;
			config.buffer =  RegisterValue;
			config.bufferSize = RegisterLen;
			config.xferPending = false;
			
			/* Start I2C read and take the semaphore */
			status = (unsigned int)I2C_MasterRead(&config);
			xSemaphoreTake(xSemaphoreI2C, portMAX_DELAY);
		}
		return status;
	}

	//Fonction qui assure le bon déroulement de la fin d'une écriture ou lecture I2C
	void I2C_Callback(uint32_t events)
	{
		BaseType_t xHigherPriorityTaskWoken;
		/**
		 * Unblock the task by releasing the semaphore only if no hardware error is 
		 * detected and I2C master read or write is completed
		 */
		if(events & CY_SCB_I2C_MASTER_ERR_EVENT)
		{
			printf("Failure!  : I2C hardware error detected\r\n");
			CY_ASSERT(0u); /* Halt CPU */
		}
		else if((events & CY_SCB_I2C_MASTER_WR_CMPLT_EVENT)|| 
				(events & CY_SCB_I2C_MASTER_RD_CMPLT_EVENT))
		{
			xSemaphoreGiveFromISR(xSemaphoreI2C, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}

	//Fonction d'initialisation du BMI160 (capteur de mouvement)
	static int8_t MotionSensor_Init(void)
	{
		/* Variable used to store the return values of BMI160 APIs */
		uint8_t rslt        = BMI160_OK;
		
		/* Start the I2C master interface for BMI160 motion sensor */
		I2C_Start();
		
		/* Register interrupt callback */
		I2C_RegisterEventCallback(I2C_Callback);
		
		/* Sensor configurations */
		sensor.id           = BMI160_I2C_ADDR;
		sensor.interface    = BMI160_I2C_INTF;
		sensor.read         = (bmi160_com_fptr_t)I2C_ReadBytes;
		sensor.write        = (bmi160_com_fptr_t)I2C_WriteBytes;
		sensor.delay_ms     = vTaskDelay;
		
		/* Initialize BNI160 sensor */
		rslt += bmi160_init(&sensor);
			
		if(rslt == BMI160_OK) /* BMI160 initialization successful */
		{
			/* Select the Output data rate, range of accelerometer sensor */
			sensor.accel_cfg.odr    = BMI160_ACCEL_ODR_12_5HZ;
			sensor.accel_cfg.range  = BMI160_ACCEL_RANGE_2G;
			sensor.accel_cfg.bw     = BMI160_ACCEL_BW_NORMAL_AVG4;

			/* Select the power mode of accelerometer sensor */
			sensor.accel_cfg.power  = BMI160_ACCEL_NORMAL_MODE;

			/* Select the Output data rate, range of Gyroscope sensor */
			sensor.gyro_cfg.odr     = BMI160_GYRO_ODR_3200HZ;
			sensor.gyro_cfg.range   = BMI160_GYRO_RANGE_2000_DPS;
			sensor.gyro_cfg.bw      = BMI160_GYRO_BW_NORMAL_MODE;

			/* Select the power mode of Gyroscope sensor */
			sensor.gyro_cfg.power   = BMI160_GYRO_NORMAL_MODE; 

			/* Set the sensor configuration */
			rslt = bmi160_set_sens_conf(&sensor);
		}
		return rslt;
	}


	//Tâche associé au capteur de mouvement
	void Task_Motion(void* pvParameters)
	{
		/* Variable used to store the return values of Motion Sensor APIs */
		static int8_t motionSensorApiResult;
		
		/* Remove warning for unused parameter */
		(void)pvParameters;
		
		/* Create binary semaphore and check the operation */
		xSemaphoreI2C = xSemaphoreCreateBinary();
		if(xSemaphoreI2C == NULL)
		{
			printf("Failure!  : Motion Sensor - Failed to create semaphore");
		}
		
		/* Initialize BMI160 Motion Sensor and check the operation */
		motionSensorApiResult = MotionSensor_Init();
		if(motionSensorApiResult != BMI160_OK) 
		{
			/* If initialization fails then print error message and suspend the task */
			printf("Failure!  : Motion Sensor initialization. Check hardware connection");
			vTaskSuspend(NULL);
		}
		
		/* Configure anymotion interrupt and check the operation */
		motionSensorApiResult = MotionSensor_ConfigAnyMotionIntr();
		if(motionSensorApiResult != BMI160_OK)
		{
			/* If configuration fails then print error message and suspend the task */
			printf("Failure!  : Motion Sensor interrupt configuration. Press Reset(SW1) to restart", \
								motionSensorApiResult);
			vTaskSuspend(NULL);
		}
        
			
		for(;;)
		{  
			 /* Structure that stores accelerometer data */
			struct bmi160_sensor_data accel;
            
			/* Variable used to store the return values of BMI160 APIs */
			int8_t rslt = BMI160_OK;
            
			/* Local variable used to store accelerometer data */
			uint16_t absX, absY, absZ;
		
			/* Read x, y, z components of acceleration */
			rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &sensor);
		
            //Sensibilité 2g --> g environ égal 16384
            
			/* Get the absolute value of accelerations */
			absX = abs(accel.x);
			absY = abs(accel.y);
			absZ = abs(accel.z);
			
            //Afficher les valeurs d'accélération à chaque seconde dans Tera
			//printf("ax = %1.2hu ay = %1.2hu az = %1.2hu\r\n",absX,absY,absZ);
			//vTaskDelay(1000);
		}
	}
	
	//Interuption AnyMotion (BMI160)
	static void AnyMotion_Interrupt(void)
	{
		
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		printf("Mouvement detected! \r\n");
		
		//FLAG : Il y a eu du mouvement ! 
		
		/* Clear any pending interrupts */
		Cy_GPIO_ClearInterrupt(Pin_AnyMotion_INT_PORT, Pin_AnyMotion_INT_NUM);
		NVIC_ClearPendingIRQ(SysInt_AnyMotionINT_cfg.intrSrc);
		
		/* Resume Task_Motion */
		xHigherPriorityTaskWoken = xTaskResumeFromISR(xTaskHandleMotion);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken );
	}	
	//Configuration de l'interuption AnyMotion
	static int8_t MotionSensor_ConfigAnyMotionIntr(void)
	{
		/* Structure for storing interrupt configuration */
		struct bmi160_int_settg int_config;
		/* Variable used to store the return values of BMI160 APIs */
		uint8_t rslt = BMI160_OK;
		
		/* Map the step interrupt to INT1 pin */
		int_config.int_channel = BMI160_INT_CHANNEL_2;

		/* Select the Interrupt type Step Detector interrupt */
		int_config.int_type = BMI160_ACC_ANY_MOTION_INT; //Anymotion int
		
		/* Interrupt pin configuration */
		/* Enabling interrupt pins to act as output pin */
		int_config.int_pin_settg.output_en = BMI160_ENABLE;
		/*Choosing push-pull mode for interrupt pin */
		int_config.int_pin_settg.output_mode = BMI160_DISABLE;
		/* Choosing active High output */
		int_config.int_pin_settg.output_type = BMI160_ENABLE;
		/* Choosing edge triggered output */
		int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;
		/* Disabling interrupt pin to act as input */
		int_config.int_pin_settg.input_en = BMI160_DISABLE;
		/* 2.5 millisecond latched output */
		int_config.int_pin_settg.latch_dur =BMI160_LATCH_DUR_2_5_MILLI_SEC;
		
		/* Interrupt type configuration */
	   // na - Select the Any Motion Interrupt parameter
		
	   // na - 1- Enable the any-motion, 0- disable any-motion
	   int_config.int_type_cfg.acc_any_motion_int.anymotion_en = BMI160_ENABLE; 
	   // na - Enabling x-axis for any motion interrupt - monitor x axis 
	   int_config.int_type_cfg.acc_any_motion_int.anymotion_x = BMI160_ENABLE; 
	   // na - Enabling y-axis for any motion interrupt - monitor y axis 
	   int_config.int_type_cfg.acc_any_motion_int.anymotion_y = BMI160_ENABLE; 
	   // na - Enabling z-axis for any motion interrupt - monitor z axis
	   int_config.int_type_cfg.acc_any_motion_int.anymotion_z = BMI160_ENABLE; 
		
	   //***SENSIBILITÉ de l'intteruption***//
		
	   // na - any-motion duration. This is the consecutive datapoints -> see datasheet pg32 section 2.6.1 <int_anym_dur> and pg78
	   int_config.int_type_cfg.acc_any_motion_int.anymotion_dur = 3;            
	   // na - An interrupt will be generated if the absolute value of two consecutive accelation signal exceeds the threshold value -> see datasheet pg32 section 2.6.1 <int_anym_th> and pg78 INT_MOTION[1] 
	   int_config.int_type_cfg.acc_any_motion_int.anymotion_thr = 10;          
	   // na - (2-g range) -> (anymotion_thr) * 3.91 mg, (4-g range) -> (anymotion_thr) * 7.81 mg, (8-g range) ->(anymotion_thr) * 15.63 mg, (16-g range) -> (anymotion_thr) * 31.25 mg

		
		/* Set the Step Detector interrupt */
		rslt = bmi160_set_int_config(&int_config, &sensor);
		
		/* Initialize and enable Orientation Interrupt*/
		Cy_SysInt_Init(&SysInt_AnyMotionINT_cfg, AnyMotion_Interrupt);
		NVIC_EnableIRQ(SysInt_AnyMotionINT_cfg.intrSrc);
		
		return rslt;
	}
	
//main

	int main(void)
	{
		__enable_irq(); /* Enable global interrupts. */
		
		UART_Start();
		//I2C_Start();
		
		printf("Acceleration du BMI160\r\n");
		
		/* Place your initialization/startup code here (e.g. MyInst_Start()) */
	   
		xTaskCreate(Task_Motion, "Motion Task", TASK_MOTION_SENSOR_STACK_SIZE, NULL, TASK_MOTION_SENSOR_PRIORITY, 0);
		vTaskStartScheduler();
		
		
		for(;;)
		{
			/* Place your application code here. */
		}
	}

/* [] END OF FILE */