/*
 * adc_task.h
 *
 *  Created on: Feb 23, 2016
 *      Author: mortenopprudjakobsen
 */

#ifndef SOURCE_ADC_TASK_H_
#define SOURCE_ADC_TASK_H_

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#define NO_SAMPLES	2048 //4096 //1024 //8192//1024

#define DEMO_ADC16_USER_CHANNEL 12U /* PTB2, ADC0_SE12 */

void startSampling(void);
void adc_init(int channel);
void adc_task(void *pvParameters);
void init_trigger_source(uint32_t adcInstance);

/* RTOS resources */
static volatile SemaphoreHandle_t xSemaphore = NULL;
static volatile QueueHandle_t sampleQhdl;




#endif /* SOURCE_ADC_TASK_H_ */
