/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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
#include "board.h"

#include "pin_mux.h"

#include "fsl_common.h"

#include "fsl_rtc.h"

#include <stdbool.h>

#include "lpcxpresso_5410x_rtc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile uint8_t gLPC5410xRTCBusyWait = 0;  /* 0-is not Busy, 1-is Busy */
volatile rtc_datetime_t gRTCDate;


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
* @brief ISR for Alarm interrupt
*
* This function changes the state of busyWait.
*/
void RTC_IRQHandler(void)
{
    if (RTC_GetStatusFlags(RTC) & kRTC_AlarmFlag)
    {
        gLPC5410xRTCBusyWait = 0;
        /* Clear alarm flag */
        RTC_ClearStatusFlags(RTC, kRTC_AlarmFlag);
    }
}

/*!
 * @brief Main function
 */
uint8_t rtc_init(rtc_datetime_t *date)
{	
    /* Enable the RTC 32K Oscillator */
    SYSCON->RTCOSCCTRL |= SYSCON_RTCOSCCTRL_EN_MASK;
    /* Init RTC */
    RTC_Init(RTC);
    
    /* Set a start date time and start RT */
//    date.year = 2014U;
//    date.month = 12U;
//    date.day = 25U;
//    date.hour = 19U;
//    date.minute = 0;
//    date.second = 0;
    /* RTC time counter has to be stopped before setting the date & time in the TSR register */
    RTC_StopTimer(RTC);
    /* Set RTC time to default */
    RTC_SetDatetime(RTC, date);
#if LPC_ENABLE_RTC_ALARM    
    /* Enable RTC alarm interrupt */
    RTC_EnableInterrupts(RTC, kRTC_AlarmInterruptEnable);
    /* Enable at the NVIC */
    EnableIRQ(RTC_IRQn);
#endif   

    gLPC5410xRTCBusyWait = 1;
    /* Start the RTC time counter */
    RTC_StartTimer(RTC);
    
    return 1;
}

uint8_t rtc_deinit(void)
{

    return 1;
}

uint8_t rtc_set(rtc_datetime_t *date)
{
    RTC_SetDatetime(RTC, date);
    return 1;
}

uint8_t rtc_get(rtc_datetime_t *date)
{
    RTC_GetDatetime(RTC, date);
    return 1;
}

// end file
