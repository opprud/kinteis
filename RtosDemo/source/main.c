/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This is template for main module created by New Kinetis SDK 2.x Project Wizard. Enjoy!
 **/

#include <string.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "adc_task.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


//extern void adc_task(void *pvParameters);


/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 2)
#define adc_task_PRIORITY (configMAX_PRIORITIES - 1)
#define network_task_PRIORITY (configMAX_PRIORITIES - 3)

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void network_task(void *pvParameters)
{
	unsigned short *samplPtr;
	unsigned short txBuff[NO_SAMPLES];

	for (;;)
	{
/*
		if(xQueueReceive(sampleQhdl, *samplPtr, 500))
		{
			PRINTF("Got pointer from Q");
			memcpy(&txBuff,samplPtr,NO_SAMPLES);

		}
*/
	}
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void hello_task(void *pvParameters)
{

	for (;;)
	{
		LED_BLUE_ON();
		vTaskDelay(200);
		LED_BLUE_OFF();
		vTaskDelay(350);
//		PRINTF("Hello world.\r\n");
		PRINTF(".");
		/* Add your code here */
		//vTaskSuspend(NULL);
	}
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
	/* Init board hardware. */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

	//adc_init(DEMO_ADC16_USER_CHANNEL);

	/* Create RTOS task */
	xTaskCreate(hello_task, "Hello_task", configMINIMAL_STACK_SIZE, NULL, hello_task_PRIORITY, NULL);
	xTaskCreate(adc_task, "ADC_task", configMINIMAL_STACK_SIZE, NULL, adc_task_PRIORITY, NULL);
	//xTaskCreate(hello_task, "Network_task", 1024, NULL, hello_task_PRIORITY, NULL);


	vTaskStartScheduler();

	for (;;)
	{ /* Infinite loop to avoid leaving the main function */
		__asm("NOP");
		/* something to use as a breakpoint stop while looping */
	}
}

