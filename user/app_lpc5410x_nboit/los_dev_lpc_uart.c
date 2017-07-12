#include "los_dev_lpc_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "fsl_usart.h"
#include "fsl_common.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

#define LOS_DEV_UART_NUM       4

los_dev_uart_t comx[LOS_DEV_UART_NUM];

/*
    LPC5410x uarts
*/
USART_Type *puarts[] = {
    USART0,
    USART1,
    USART2,
    USART3,
};

static void los_dev_uart_gpio_init(int uartidx)
{
    if(uartidx == 0)
    {
    
    }
    if(uartidx == 1)
    {
    
    }
    if(uartidx == 2)
    {
    
    
    }
    if(uartidx == 3)
    {
        IOCON_PinMuxSet(IOCON, 1, 12, (IOCON_MODE_INACT | IOCON_FUNC2 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)); /* PORT1 PIN12 (coords: 12) is configured as U3_RXD */
        IOCON_PinMuxSet(IOCON, 1, 13, (IOCON_MODE_INACT | IOCON_FUNC2 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)); /* PORT1 PIN13 (coords: 13) is configured as U3_TXD */
    }
}

int los_dev_uart_init(int uartidx, int baudrate, char *buf, int size)
{
    if (uartidx > LOS_DEV_UART_NUM || uartidx <= 0)
    {
        return -1;
    }
    if (NULL == buf || size <= 0)
    {
        return -1;
    }
    if (baudrate != 9600 && baudrate != 115200)
    {
        return -1;
    }
    
    los_dev_uart_gpio_init(uartidx);
    
    /* init read write mutex and lock it */
    LOS_SemCreate(0,&comx[uartidx].rsem);
    LOS_SemCreate(1,&comx[uartidx].wsem);
    //LOS_SemPend(comx[uartidx].rsem, LOS_WAIT_FOREVER);
    //LOS_SemPend(comx[uartidx].wsem, LOS_WAIT_FOREVER);
    
    comx[uartidx].dev = puarts[uartidx];
    comx[uartidx].config.baudRate_Bps = 9600;
    comx[uartidx].config.bitCountPerChar = kUSART_8BitsPerChar;
    comx[uartidx].config.stopBitCount = kUSART_OneStopBit; 
    comx[uartidx].config.parityMode = kUSART_ParityDisabled; 
    comx[uartidx].config.enableRx = true;             
    comx[uartidx].config.enableTx = true;
    comx[uartidx].config.fifoConfig.enableTxFifo = false;
    comx[uartidx].config.fifoConfig.enableRxFifo = false;
    comx[uartidx].config.fifoConfig.rxFifoSize = 16U;
    comx[uartidx].config.fifoConfig.txFifoSize = 16U;
    comx[uartidx].config.fifoConfig.txFifoThreshold = 0U;
    comx[uartidx].config.fifoConfig.rxFifoThreshold = 0U;
    
    if(uartidx == 0)
    {
        comx[uartidx].irq_number = USART0_IRQn;
    }
    if(uartidx == 1)
    {
        comx[uartidx].irq_number = USART1_IRQn;
    }
    if(uartidx == 2)
    {
        comx[uartidx].irq_number = USART2_IRQn;
    }
    if(uartidx == 3)
    {
        comx[uartidx].irq_number = USART3_IRQn;
    }
    
    if(USART_Init(comx[uartidx].dev, &comx[uartidx].config, CLOCK_GetFreq(kCLOCK_Usart)) != kStatus_Success)
    {
        while(1);
    }
    comx[uartidx].pbuf = buf;
    comx[uartidx].dualbuf = buf + size / 2;
    comx[uartidx].interrbuf = buf;
    comx[uartidx].size = size;
    comx[uartidx].readcnt = 0;
    
    comx[uartidx].pRxBuffPtr  = (uint8_t *)buf;
    comx[uartidx].RxXferSize  = size / 2;
    comx[uartidx].RxXferCount = size / 2;
    
    /* Enable RX interrupt. */
    if (USART_IsRxFifoEnable(comx[uartidx].dev))
    {
        USART_EnableFifoInterrupts(comx[uartidx].dev, kUSART_RxFifoThresholdInterruptEnable);
    }
    else
    {
        USART_EnableInterrupts(comx[uartidx].dev, kUSART_RxReadyInterruptEnable | kUSART_HardwareOverRunInterruptEnable);
    }
    EnableIRQ(comx[uartidx].irq_number);
    
    return 0;
}

void los_dev_uart_irqhandler(int uartidx)
{
    volatile uint8_t* tmp;
    
    if ((USART_IsRxFifoEnable(comx[uartidx].dev)) ?
            (kUSART_RxFifoThresholdFlag & USART_GetFifoStatusFlags(comx[uartidx].dev)) :
            ((kUSART_RxReady | kUSART_HardwareOverrunFlag) & USART_GetStatusFlags(comx[uartidx].dev)))
    {
        tmp = (uint8_t*)comx[uartidx].pRxBuffPtr;
        
        *tmp = USART_ReadByte(comx[uartidx].dev);
        comx[uartidx].pRxBuffPtr++;
        comx[uartidx].readcnt++;
        
        if(--comx[uartidx].RxXferCount == 0)
        {
            if (comx[uartidx].pRxBuffPtr > (uint8_t *)comx[uartidx].dualbuf)
            {
                comx[uartidx].pRxBuffPtr = (uint8_t *)comx[uartidx].pbuf;
                comx[uartidx].interrbuf  = comx[uartidx].dualbuf;
            }
            else
            {
                comx[uartidx].pRxBuffPtr = (uint8_t *)comx[uartidx].dualbuf;
                comx[uartidx].interrbuf  = comx[uartidx].pbuf;
            }
            comx[uartidx].RxXferCount = comx[uartidx].RxXferSize;
            comx[uartidx].readcnt = comx[uartidx].RxXferSize;
            //LOS_SemPost(comx[uartidx].rsem);
        }
        LOS_SemPost(comx[uartidx].rsem);
    }
}

void USART3_IRQHandler(void)
{
    los_dev_uart_irqhandler(LOS_USART3);
}

int los_dev_uart_read(int uartidx, char *outbuf, int rsize, int mstimout)
{
    unsigned long tick_start;
    unsigned long tick_cur;
    UINT32 uwRet;
    int tmpread = rsize;
    int tmptimout = mstimout;
    char *pouttmp = outbuf;
    
    if (uartidx > LOS_DEV_UART_NUM || uartidx <= 0)
    {
        return -1;
    }
    if (NULL == outbuf || rsize <= 0)
    {
        return -1;
    }
    
    // no nonblock read uart buf
    if (0 == mstimout)
    {
        if (comx[uartidx].readcnt == 0)
        {
            return 0;
        }
        else
        {
            if (rsize > comx[uartidx].readcnt)
            {
                memcpy((void *)outbuf, (const void *)comx[uartidx].interrbuf, (unsigned int)comx[uartidx].readcnt);
                comx[uartidx].readcnt = 0;
                return comx[uartidx].readcnt;
            }
            else
            {
                memcpy(outbuf, comx[uartidx].interrbuf, rsize);
                comx[uartidx].interrbuf = comx[uartidx].interrbuf + rsize;
                comx[uartidx].readcnt = comx[uartidx].readcnt - rsize;
                return rsize;
            }
        }
    }
    
    tick_start = (UINT32)LOS_TickCountGet();
    
    while(1)
    {
        uwRet = LOS_SemPend(comx[uartidx].rsem, tmptimout);
        if(uwRet == LOS_OK)
        {
            if (tmpread <= comx[uartidx].readcnt)
            {
                // read enough, directly return
                memcpy(pouttmp, comx[uartidx].interrbuf, comx[uartidx].readcnt);
                memset(comx[uartidx].interrbuf, 0, (comx[uartidx].size/2));
                
                rsize = comx[uartidx].readcnt;
                comx[uartidx].readcnt = 0;
                comx[uartidx].pRxBuffPtr  = comx[uartidx].pbuf;
                comx[uartidx].RxXferCount = comx[uartidx].size/2;
                return rsize;
            }
            
                
            tick_cur = (UINT32)LOS_TickCountGet();
            if (tick_cur - tick_start < tmptimout)
            {
                tmptimout = tmptimout - (tick_cur - tick_start);
                tick_start = (UINT32)LOS_TickCountGet();
                continue;
            }
            else
            {
                break ;
            }
        }
        else if(uwRet == LOS_ERRNO_SEM_TIMEOUT )
        {
            // not read enough, directly return
            memcpy(pouttmp, comx[uartidx].interrbuf, comx[uartidx].readcnt);
//            memset(comx[uartidx].interrbuf, 0, (comx[uartidx].size/2));
            rsize = comx[uartidx].readcnt;
            
            //comx[uartidx].RxXferCount = comx[uartidx].size/2;
            // no data come, nee return ;
            return rsize;
        }
    }
    return 0;
}

int los_dev_uart_write(int uartidx, char *inbuf, int wsize, int mstimout)
{
    UINT32 uwRet;
    if (uartidx > LOS_DEV_UART_NUM || uartidx <= 0)
    {
        return -1;
    }
    if (NULL == inbuf || wsize <= 0)
    {
        return -1;
    }
    uwRet = LOS_SemPend(comx[uartidx].wsem, mstimout);
    if(uwRet == LOS_OK)
    {
        //HAL_UART_Transmit(&comx[uartidx].dev, (uint8_t*)inbuf, wsize, 1000);
        USART_WriteBlocking(comx[uartidx].dev, (uint8_t*)inbuf, wsize);
        LOS_SemPost(comx[uartidx].wsem);
        return wsize;
    }
    else if(uwRet == LOS_ERRNO_SEM_TIMEOUT )
    {
        return 0;
    }
    return 0;
}

void los_dev_uart_clean(int uartidx)
{
    comx[uartidx].pRxBuffPtr = comx[uartidx].pbuf;
    memset(comx[uartidx].interrbuf, 0, (comx[uartidx].size/2));
    comx[uartidx].readcnt = 0;
    comx[uartidx].RxXferCount = comx[uartidx].size/2;
}

