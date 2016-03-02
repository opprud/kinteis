/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/* Application Included Files */
#include "echo.h"

#include "lwip/opt.h"

#if LWIP_NETCONN
/* Standard C Included Files */
#include <stdio.h>
/* lwip Included Files */
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "netif/etharp.h"
/* SDK Included Files */
#include "ethernetif.h"
#include "board.h"

/*  ADC related */
#include "pin_mux.h"
#include "clock_config.h"
//#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_pit.h"
#include "clock_config.h"

#define NO_SAMPLES	1024//2048 //4096 //1024 //8192//1024

#define DEMO_ADC16_USER_CHANNEL 12U /* PTB2, ADC0_SE12 */
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;

unsigned short adcSamples[2][NO_SAMPLES];
unsigned int sample_index = 0;
unsigned int sample_buff_in_use = 0;

/* RTOS resources */
static volatile SemaphoreHandle_t xSemaphore = NULL;
//static volatile QueueHandle_t sampleQhdl;

/*******************************************************************************
 * Variables
 ******************************************************************************/

struct netif fsl_netif0;

#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define EXAMPLE_ENET ENET

/* IP address configuration. */
#define configIP_ADDR0 192
#define configIP_ADDR1 168
//#define configIP_ADDR2 1
#define configIP_ADDR2 2
#define configIP_ADDR3 102

/* Netmask configuration. */
#define configNET_MASK0 255
#define configNET_MASK1 255
#define configNET_MASK2 255
#define configNET_MASK3 0

/* Default gateway address configuration */
#define configGW_ADDR0 192
#define configGW_ADDR1 168
//#define configGW_ADDR2 1
#define configGW_ADDR2 2
#define configGW_ADDR3 100

#define configPHY_ADDRESS 1
#ifndef TCPECHO_DBG
#define TCPECHO_DBG LWIP_DBG_ON
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void PIT0_IRQHandler(void)
{
	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, PIT_TFLG_TIF_MASK);
	/*start ADC*/
	ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
	//ADC16_SetChannelConfig()
}

#if 0
/*********HAAAAAAAAAAAACK!!!!************/
void ADC16_SetChannelConfig(ADC_Type *base, uint32_t channelGroup, const adc16_channel_config_t *config)
{
	assert(channelGroup < ADC_SC1_COUNT);
	assert(NULL != config);

	uint32_t sc1 = ADC_SC1_ADCH(config->channelNumber); /* Set the channel number. */

	/* Enable the differential conversion. */
	if (config->enableDifferentialConversion)
	{
		sc1 |= ADC_SC1_DIFF_MASK;
	}
	/* Enable the interrupt when the conversion is done. */
	if (config->enableInterruptOnConversionCompleted)
	{
		sc1 |= ADC_SC1_AIEN_MASK;
	}
	base->SC1[channelGroup] = sc1;
}
#endif
/*!
 * @brief ADC interrupt handler
 */
void ADC0_IRQHandler(void)
{
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	//debug
	GPIO_TogglePinsOutput(GPIOE, 1U << 24);

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
//		xSemaphoreGiveFromISR(xSemaphore, NULL);
		//xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);

		/* If xHigherPriorityTaskWoken was set to true you
		 we should yield.  The actual macro used here is
		 port specific. */
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void startSampling(void)
{
	/* !IMPORTANT !
	 * set priority to allow OS function from within interrupt*/
	//NVIC_SetPriority(ADC0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	//NVIC_SetPriority(PIT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	EnableIRQ(PIT0_IRQn);
	EnableIRQ(ADC0_IRQn);

	/* Start channel 0 */
	//PRINTF("\r\nStarting channel No.0 ...");
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
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(freqUs, PIT_SOURCE_CLOCK));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	//EnableIRQ(PIT0_IRQn);
	/* Start channel 0 */
	LWIP_PLATFORM_DIAG(("\r\nStarting channel No.0 ..."));
	//PIT_StartTimer(PIT, kPIT_Chnl_0);
}


/*!
 * @brief Task responsible for initializing ADC
 */
void adc_init(int channel)
{
	//debug
	GPIO_PinInit(GPIOE, 24, &(gpio_pin_config_t
			)
			{ kGPIO_DigitalOutput, (1) });

	/* !IMPORTANT !
	 * set priority to allow OS function from within interrupt*/
	//NVIC_SetPriority(ADC0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
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
		LWIP_PLATFORM_DIAG(("ADC16_DoAutoCalibration() Done.\r\n"));
	}

	adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
	adc16ChannelConfigStruct.enableDifferentialConversion = false;

}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
void adc_task(void *pvParameters)
{
	/* setup ADC*/
	adc_init(DEMO_ADC16_USER_CHANNEL);
	/* and trigger timer */
	init_trigger_source(1);

	startSampling();
#endif

	for (;;)
	{

		/*Block waiting for the samples/semaphore to become available. */
		/*
		if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE)
		{
			LED_RED_TOGGLE();
		}*/

		LED_RED_TOGGLE();
		vTaskDelay(400);
	}
}

/* toggle LED  */
static void led_thread(void *arg)
{
	/* Initialize LED pins below */
	LED_RED_INIT(1U);
	LED_GREEN_INIT(1U);
	LED_BLUE_INIT(1U);
	while (1)
	{
		LED_GREEN_ON();
		vTaskDelay(200);
		LED_GREEN_OFF();
		vTaskDelay(1500);
	}
}

static void tcpdata_thread(void *arg)
{
	//int i = 0;
	struct netconn *conn, *newconn;
	err_t err;

	LWIP_UNUSED_ARG(arg);
	netif_set_up(&fsl_netif0);
	// Create a new connection identifier
	conn = netconn_new(NETCONN_TCP);

	// Bind connection to well known port number 7
	netconn_bind(conn, NULL, 7);

	// Tell connection to go into listening mode
	netconn_listen(conn);

	while (1)
	{

		/* Grab new connection. */
		err = netconn_accept(conn, &newconn);
		/* Process the new connection. */
		if (err == ERR_OK)
		{
			struct netbuf *buf;
			void *data;
			unsigned int len;
			char cmd[40];

			while ((err = netconn_recv(newconn, &buf)) == ERR_OK)
			{
				do
				{
					netbuf_data(buf, &data, &len);
					strcpy(cmd, data);

					if (!strcmp(cmd, "GET_SAMPLES"))
					{
						len = sizeof(adcSamples)/2;

						if (sample_buff_in_use == 1)
							err = netconn_write(newconn, (unsigned char*)&adcSamples[0][0], len, NETCONN_COPY);

						else
							err = netconn_write(newconn, (unsigned char*)&adcSamples[1][0], len, NETCONN_COPY);

//						for (i = 0; i < NO_SAMPLES; i++)
//							txBuff[i] = rand()%0xffff;

//						err = netconn_write(newconn, (unsigned char*)txBuff, len, NETCONN_COPY);
					}

				} while (netbuf_next(buf) >= 0);
				netbuf_delete(buf);
			}
			/* Close connection and discard connection identifier. */
			netconn_close(newconn);
			netconn_delete(newconn);
		}
	}
}

void tcpecho_init(void)
{

	sys_thread_new("tcpdata_thread", tcpdata_thread, NULL, TCPECHO_STACKSIZE, TCPECHO_PRIORITY);
	//sys_thread_new("tcpecho_thread", tcpecho_thread, NULL, TCPECHO_STACKSIZE, TCPECHO_PRIORITY);
	sys_thread_new("led_thread", led_thread, NULL, 512, 2);
	sys_thread_new("adc_thread", adc_task, NULL, 512, 3);

	//sys_thread_new("tcpecho_thread", tcpecho_thread, NULL, TCPECHO_STACKSIZE, TCPECHO_PRIORITY);
	vTaskStartScheduler();
}

/*!
 * @brief Main function
 */
int main(void)
{
	ip_addr_t fsl_netif0_ipaddr, fsl_netif0_netmask, fsl_netif0_gw;

	MPU_Type *base = MPU;
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	/* Disable MPU. */
	base->CESR &= ~MPU_CESR_VLD_MASK;

	LWIP_DEBUGF(TCPECHO_DBG, ("TCP/IP initializing...\r\n"));
	tcpip_init(NULL, NULL);
	LWIP_DEBUGF(TCPECHO_DBG, ("TCP/IP initializing...\r\n"));

	IP4_ADDR(&fsl_netif0_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
	IP4_ADDR(&fsl_netif0_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
	IP4_ADDR(&fsl_netif0_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);
	netif_add(&fsl_netif0, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw, NULL, ethernetif_init, tcpip_input);
	netif_set_default(&fsl_netif0);

	LWIP_PLATFORM_DIAG(("\r\n************************************************"));
	LWIP_PLATFORM_DIAG((" TCP Echo example"));
	LWIP_PLATFORM_DIAG(("************************************************"));
	LWIP_PLATFORM_DIAG(
			(" IPv4 Address     : %u.%u.%u.%u", ((u8_t *)&fsl_netif0_ipaddr)[0], ((u8_t *)&fsl_netif0_ipaddr)[1], ((u8_t *)&fsl_netif0_ipaddr)[2], ((u8_t *)&fsl_netif0_ipaddr)[3]));
	LWIP_PLATFORM_DIAG(
			(" IPv4 Subnet mask : %u.%u.%u.%u", ((u8_t *)&fsl_netif0_netmask)[0], ((u8_t *)&fsl_netif0_netmask)[1], ((u8_t *)&fsl_netif0_netmask)[2], ((u8_t *)&fsl_netif0_netmask)[3]));
	LWIP_PLATFORM_DIAG(
			(" IPv4 Gateway     : %u.%u.%u.%u", ((u8_t *)&fsl_netif0_gw)[0], ((u8_t *)&fsl_netif0_gw)[1], ((u8_t *)&fsl_netif0_gw)[2], ((u8_t *)&fsl_netif0_gw)[3]));
	LWIP_PLATFORM_DIAG(("************************************************"));

	tcpecho_init();
	/* should not reach this statement */
	for (;;)
		;
}
 /* LWIP_NETCONN */
