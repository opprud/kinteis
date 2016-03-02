/*
 * adc_task.c
 *
 *  Created on: Feb 23, 2016
 *      Author: mortenopprudjakobsen
 */

#include <string.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "adc_task.h"

#include "fsl_pit.h"
#include "clock_config.h"

/* FreeRTOS kernel includes. */
/*
 #include "FreeRTOS.h"
 #include "task.h"
 #include "queue.h"
 #include "timers.h"
 #include "semphr.h"
 */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U

#define DEMO_ADC16_IRQn ADC0_IRQn
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

/*******************************************************************************
 * module variables
 ******************************************************************************/
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;

unsigned short adcSamples[2][NO_SAMPLES];
unsigned int sample_index = 0;
unsigned int sample_buff_in_use = 0;

volatile bool pitIsrFlag = false;


void PIT0_IRQHandler(void)
{
	GPIO_TogglePinsOutput(GPIOE, 1U << 24);

	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, PIT_TFLG_TIF_MASK);
	//pitIsrFlag = true;

	/*start ADC*/
	//ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
}

/*!
 * @brief ADC interrupt handler
 */
void ADC0_IRQHandler(void)
{
	//uint32_t g_Adc16ConversionValue;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	//debug
	//GPIO_TogglePinsOutput(GPIOE, 1U << 24);

	/* Read conversion result to clear the conversion completed flag. */
	//g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
	adcSamples[sample_buff_in_use][sample_index] = (unsigned short) ADC16_GetChannelConversionValue(DEMO_ADC16_BASE,
	DEMO_ADC16_CHANNEL_GROUP); //g_Adc16ConversionValue;

	/* buffer is full, notify and update */
	if (sample_index++ == NO_SAMPLES)
	{
		sample_index = 0;
		if (sample_buff_in_use == 0)
			sample_buff_in_use = 1;
		else
			sample_buff_in_use = 0;

		/* wake up adc ADC thread */
		/* Unblock the task by releasing the semaphore. */
		xSemaphoreGiveFromISR(xSemaphore, NULL);
		//xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);

		/* If xHigherPriorityTaskWoken was set to true you
		 we should yield.  The actual macro used here is
		 port specific. */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void startSampling(void)
{
	/* !IMPORTANT !
	 * set priority to allow OS function from within interrupt*/
//	NVIC_SetPriority(PIT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
//	NVIC_SetPriority(ADC0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	EnableIRQ(PIT0_IRQn);
	//EnableIRQ(ADC0_IRQn);

	/* Start channel 0 */
	PRINTF("\r\nStarting channel No.0 ...");
	PIT_StartTimer(PIT, kPIT_Chnl_0);


}

/*!
 * @Brief enable the trigger source of PIT0, chn0
 */
void init_trigger_source(uint32_t adcInstance)
{
	uint32_t freqUs;

	freqUs = 1000000U / NO_SAMPLES;

	/* Structure of initialize PIT */
	pit_config_t pitConfig;

	/*pitConfig.enableRunInDebug = false; */
	PIT_GetDefaultConfig(&pitConfig);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig);

	/* Set timer period for channel 0 */
	//PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(20000U, PIT_SOURCE_CLOCK));
//	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(freqUs, PIT_SOURCE_CLOCK));
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(100, PIT_SOURCE_CLOCK));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	//EnableIRQ(PIT0_IRQn);
	/* Start channel 0 */
	//PRINTF("\r\nStarting channel No.0 ...");
	//PIT_StartTimer(PIT, kPIT_Chnl_0);
}

/*!
 * @brief Task responsible for initializing ADC
 */
void adc_init(int channel)
{
	//debug
	GPIO_PinInit(GPIOE, 24, &(gpio_pin_config_t){ kGPIO_DigitalOutput, (1) });

	/* !IMPORTANT !
	 * set priority to allow OS function from within interrupt*/
	NVIC_SetPriority(DEMO_ADC16_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	//EnableIRQ(DEMO_ADC16_IRQn);

	/*
	 * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	 * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	 * adc16ConfigStruct.enableAsynchronousClock = true;
	 * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	 * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
	 * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	 * adc16ConfigStruct.enableHighSpeed = false;
	 * adc16ConfigStruct.enableLowPower = false;
	 * adc16ConfigStruct.enableContinuousConversion = false;
	 */
	/* run continiously*/
	//adc16ConfigStruct.enableContinuousConversion = true;
	ADC16_GetDefaultConfig(&adc16ConfigStruct);
	ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false);

	if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
	{
		PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	}

	adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
	adc16ChannelConfigStruct.enableDifferentialConversion = false;

	startSampling();

}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
void adc_task(void *pvParameters)
{

	int res;
	int val = 2;
	unsigned short *ptr;

#if 1
	/* We are using the semaphore for synchronisation so we create a binary
	 semaphore rather than a mutex.
	 We must make sure that the interrupt
	 does not attempt to use the semaphore before it is created! */
	xSemaphore = xSemaphoreCreateBinary();
	/* queue for passing pointers to the sample array */
	//sampleQhdl = xQueueCreate(10, sizeof(unsigned short *));
	/* setup ADC*/
	//adc_init(DEMO_ADC16_USER_CHANNEL);
	/* and trigger timer */
	init_trigger_source(1);
	startSampling();

	/* semaphore from ISR signaling sample buffer full */
	if (xSemaphore == NULL)
	{
		PRINTF("Error creating semaphore\r\n");
		vTaskSuspend(NULL);
	}
#endif

	for (;;)
	{

		/*Block waiting for the samples/semaphore to become available. */
		if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE)
		{
#if 0
			//val++;
			if (pdTRUE != xQueueSend(sampleQhdl, &val, 500))
			{
				PRINTF("Queue 1 send failed\n");
			}
#endif

			/*if current sampling runs in buffer 0, send pointer to buffer 1*/
			if (sample_buff_in_use == 0)
			{
				ptr = &adcSamples[1][0];
			}
			else
			{
				ptr = &adcSamples[0][0];
			}

#if 0
			if (pdTRUE != xQueueSend(sampleQhdl, &ptr, 500))
			{
				PRINTF("Queue 0 send failed\n");
			}
#endif

			//PRINTF("got samples");
			LED_RED_TOGGLE();
		}
		//vTaskSuspend(NULL);
	}
}
