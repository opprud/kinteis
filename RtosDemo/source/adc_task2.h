/*
 * adc_task.h
 *
 *  Created on: Feb 23, 2016
 *      Author: mortenopprudjakobsen
 */

#ifndef SOURCE_ADC_TASK2_H_
#define SOURCE_ADC_TASK2_H_


#define DEMO_ADC16_USER_CHANNEL 12U /* PTB2, ADC0_SE12 */

void adc_init2(int channel);
void adc_task2(void *pvParameters);


#endif /* SOURCE_ADC_TASK_H_ */
