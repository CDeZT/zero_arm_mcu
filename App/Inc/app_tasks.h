#pragma once

#include "cmsis_os2.h"

extern const osThreadAttr_t host_task_attributes;
extern const osThreadAttr_t motor_task_attributes;
extern const osThreadAttr_t motion_task_attributes;

void HostTask(void *argument);
void MotorTask(void *argument);
void MotionTask(void *argument);
