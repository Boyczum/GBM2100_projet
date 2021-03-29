/******************************************************************************
* File Name: main_cm4.c
*
* Detection du mouvement (message d'alerte dans Tera) par interuption
* 
* Pour que le projet marche, vous devez cocher RTOS et I/O dans la pdl :
* --> click droit sur le projet --> build setting --> PDL --> cocher
*
* TERA : Baud rate = 115200
*******************************************************************************/
/* Header file includes */
#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "motion_task.h"
#include "uart_debug.h" 

/* Priority of user tasks in this project */
#define TASK_MOTION_SENSOR_PRIORITY (configMAX_PRIORITIES - 1)

/* Stack size of user tasks in this project */
#define TASK_MOTION_SENSOR_STACK_SIZE   (configMINIMAL_STACK_SIZE)

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function sets up user tasks and then starts 
*  the RTOS scheduler.
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main()
{            
    /* Initialize the hardware used to send debug messages. */
    DebugPrintfInit();
    
    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    DebugPrintf("\x1b[2J\x1b[;H");
    DebugPrintf("PSoC 6 MCU: Interfacing BMI160 (I2C) in FreeRTOS\r\n");
    
    /* Create the user Tasks. See the respective Task definition for more
       details of these tasks */       
    xTaskCreate(Task_Motion, "Motion Task", TASK_MOTION_SENSOR_STACK_SIZE,
                NULL, TASK_MOTION_SENSOR_PRIORITY, &xTaskHandleMotion);

    /* Initialize thread-safe debug message printing. See uart_debug.h header 
       file to enable / disable this feature */
    Task_DebugInit();
    
    /* Start the RTOS scheduler. This function should never return */
    vTaskStartScheduler();
    
    /* Should never get here! */ 
    DebugPrintf("Error!   : RTOS - scheduler crashed \r\n");
    
    /* Halt the CPU if scheduler exits */
    CY_ASSERT(0);
    
    for(;;)
    {
    }	
}

/*******************************************************************************
* Function Name: void vApplicationIdleHook(void)
********************************************************************************
*
* Summary:
*  This function is called when the RTOS in idle mode
*    
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationIdleHook(void)
{
    /* Enter sleep-mode */
    Cy_SysPm_Sleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
}

/*******************************************************************************
* Function Name: void vApplicationStackOverflowHook(TaskHandle_t *pxTask, 
                                                    signed char *pcTaskName)
********************************************************************************
*
* Summary:
*  This function is called when a stack overflow has been detected by the RTOS
*    
* Parameters:
*  TaskHandle_t  : Handle to the task
*  signed char   : Name of the task
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationStackOverflowHook(TaskHandle_t *pxTask, signed char *pcTaskName)
{
    /* Remove warning for unused parameters */
    (void)pxTask;
    
    /* Print the error message with task name if debug is enabled in 
       uart_debug.h file */
    DebugPrintf("Error!   : RTOS - stack overflow in %s \r\n", pcTaskName);
    
    /* Halt the CPU */
    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: void vApplicationMallocFailedHook(void)
********************************************************************************
*
* Summary:
*  This function is called when a memory allocation operation by the RTOS
*  has failed
*    
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationMallocFailedHook(void)
{
    /* Print the error message if debug is enabled in uart_debug.h file */
    DebugPrintf("Error!   : RTOS - Memory allocation failed \r\n");
    
    /* Halt the CPU */
    CY_ASSERT(0);
}

/* [] END OF FILE */
