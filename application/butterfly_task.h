#pragma once

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "butterfly.h"
#include "motor.h"

osThreadId motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId butterflyTaskHandle;
const osThreadAttr_t butterflyTask_attributes = {
  .name = "butterflyTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void motorTASK(void *argument);
void butterflyTASK(void *argument);

void OSTask_Init(void)
{
    motorTaskHandle = osThreadNew(motorTASK, NULL, &motorTask_attributes);
    butterflyTaskHandle = osThreadNew(butterflyTASK, NULL, &butterflyTask_attributes);
}

void motorTASK(void *argument)
{

    /* Infinite loop */
    for(;;)
    {
        MotorControl();
        osDelay(10);
    }
}

void butterflyTASK(void *argument)
{
    
    /* Infinite loop */
    for(;;)
    {
        Butterfly_Task();
        osDelay(100);
    }
}