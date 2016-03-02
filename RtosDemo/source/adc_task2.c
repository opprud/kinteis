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
#include "fsl_pdb.h"
#include "adc_task2.h"

#include "pin_mux.h"
#include "clock_config.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U

#define DEMO_ADC16_IRQn ADC0_IRQn
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)


#define DEMO_PDB_BASE PDB0
#define DEMO_PDB_IRQ_ID PDB0_IRQn
#define DEMO_PDB_IRQ_HANDLER PDB0_IRQHandler

#define DEMO_PDB_ADC_TRIGGER_CHANNEL 0U    /* For ADC0. */
#define DEMO_PDB_ADC_PRETRIGGER_CHANNEL 0U /* For ADC0_SC1[0]. */

#define DEMO_ADC_BASE ADC0
#define DEMO_ADC_CHANNEL_GROUP 0U
//#define DEMO_ADC_USER_CHANNEL 12U
#define DEMO_ADC_IRQ_ID ADC0_IRQn
#define DEMO_ADC_IRQ_HANDLER ADC0_IRQHandler



/*******************************************************************************
 * module variables
 ******************************************************************************/
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;

volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue;
volatile uint32_t g_Adc16InterruptCounter;

#define NO_SAMPLES	128 //1024
unsigned short adcSamples[2][NO_SAMPLES];
unsigned int sample_index = 0;
unsigned int sample_buff_in_use = 0;

xSemaphoreHandle adc_sem;


//////
volatile uint32_t g_PdbDelayInterruptCounter;
volatile bool g_PdbDelayInterruptFlag;


/*!
 * @brief ISR for PDB interrupt function
 */
void PDB0_IRQHandler(void)
{
    PDB_ClearStatusFlags(DEMO_PDB_BASE, kPDB_DelayEventFlag);
    g_PdbDelayInterruptCounter++;
    g_PdbDelayInterruptFlag = true;
}

/*!
 * @brief ADC interrupt handler
 */
void ADC0_IRQHandler(void)
{
	long task_woken;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	g_Adc16ConversionDoneFlag = true;
	/* Read conversion result to clear the conversion completed flag. */
	g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
	g_Adc16InterruptCounter++;

	adcSamples[sample_buff_in_use][sample_index] = (unsigned short) g_Adc16ConversionValue;

	/* buffer is full, notify and update */
	if (sample_index++ == NO_SAMPLES)
	{
		sample_index = 0;
		if (sample_buff_in_use == 0)
			sample_buff_in_use = 1;
		else
			sample_buff_in_use = 0;

		/* wake up adc ADC thread */
		xSemaphoreGiveFromISR(adc_sem, &task_woken);
	}
}

/*!
 * @brief Task responsible for initializing ADC
 */
void adc_init2(int channel)
{
	/*** PDB part ***/
	pdb_config_t pdbConfigStruct;
	pdb_adc_pretrigger_config_t pdbAdcPreTriggerConfigStruct;

	EnableIRQ(DEMO_PDB_IRQ_ID); //TODO!!
	EnableIRQ(DEMO_ADC_IRQ_ID);

	/* Configure the PDB counter. */
	/*
	 * pdbConfigStruct.loadValueMode = kPDB_LoadValueImmediately;
	 * pdbConfigStruct.prescalerDivider = kPDB_PrescalerDivider1;
	 * pdbConfigStruct.dividerMultiplicationFactor = kPDB_DividerMultiplicationFactor1;
	 * pdbConfigStruct.triggerInputSource = kPDB_TriggerSoftware;
	 * pdbConfigStruct.enableContinuousMode = false;
	 */
	PDB_GetDefaultConfig(&pdbConfigStruct);
	PDB_Init(DEMO_PDB_BASE, &pdbConfigStruct);

	/* Configure the delay interrupt. */
	PDB_SetModulusValue(DEMO_PDB_BASE, 1000U);

	/* The available delay value is less than or equal to the modulus value. */
	PDB_SetCounterDelayValue(DEMO_PDB_BASE, 1000U);
	PDB_EnableInterrupts(DEMO_PDB_BASE, kPDB_DelayInterruptEnable);

	/* Configure the ADC Pre-Trigger. */
	pdbAdcPreTriggerConfigStruct.enablePreTriggerMask = 1U << DEMO_PDB_ADC_PRETRIGGER_CHANNEL;
	pdbAdcPreTriggerConfigStruct.enableOutputMask = 1U << DEMO_PDB_ADC_PRETRIGGER_CHANNEL;
	pdbAdcPreTriggerConfigStruct.enableBackToBackOperationMask = 0U;
	PDB_SetADCPreTriggerConfig(DEMO_PDB_BASE, DEMO_PDB_ADC_TRIGGER_CHANNEL, &pdbAdcPreTriggerConfigStruct);
	PDB_SetADCPreTriggerDelayValue(DEMO_PDB_BASE, DEMO_PDB_ADC_TRIGGER_CHANNEL, DEMO_PDB_ADC_PRETRIGGER_CHANNEL, 200U);
	/* The available Pre-Trigger delay value is less than or equal to the modulus value. */

	PDB_DoLoadValues(DEMO_PDB_BASE);

	/*** ADC Part **/
	adc16_config_t adc16ConfigStruct;
	adc16_channel_config_t adc16ChannelConfigStruct;

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
	ADC16_GetDefaultConfig(&adc16ConfigStruct);
	ADC16_Init(DEMO_ADC_BASE, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(DEMO_ADC_BASE, false);
	ADC16_DoAutoCalibration(DEMO_ADC_BASE);

	adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
	adc16ChannelConfigStruct.enableDifferentialConversion = false;

	g_PdbDelayInterruptCounter = 0;
	g_Adc16InterruptCounter = 0;

}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
void adc_task2(void *pvParameters)
{

	init_trigger_source(1);

	//xSemaphoreCreateBinary(adc_sem);
	vSemaphoreCreateBinary(adc_sem);

	for (;;)
	{

		LED_RED_ON();

		g_Adc16ConversionDoneFlag = false;
		/*
		 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
		 function, which works like writing a conversion command and executing it. For another channel's conversion,
		 just to change the "channelNumber" field in channel configuration structure, and call the function
		 "ADC16_ChannelConfigure()"" again.
		 Also, the "enableInterruptOnConversionCompleted" inside the channel configuration structure is a parameter for
		 the conversion command. It takes affect just for the current conversion. If the interrupt is still required
		 for the following conversion, it is necessary to assert the "enableInterruptOnConversionCompleted" every time
		 for each command.
		 */
		ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);

		/* ADC ISR happended */
		if (xSemaphoreTake(adc_sem, 999999))
		{
			PRINTF("got samples");
		}

		LED_RED_OFF();
		//vTaskSuspend(NULL);
	}
}
