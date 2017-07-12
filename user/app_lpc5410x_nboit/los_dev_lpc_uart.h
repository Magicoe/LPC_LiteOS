#ifndef _LOS_DEV_LPC_UART_H
#define _LOS_DEV_LPC_UART_H


//LiteOS header files

#include "los_base.ph"
#include "los_hwi.h"
#include "los_sem.h"

#include "fsl_usart.h"

//BSP header files

#define LOS_USART0   0
#define LOS_USART1   1
#define LOS_USART2   2
#define LOS_USART3   3

typedef struct _los_dev_uart_t
{
    USART_Type *dev;                // LPC5410x usart number
    usart_config_t config;          // LPC5410x usart config
    uint8_t *pRxBuffPtr;            // LPC5410x usart Rx receive buffer
    uint32_t RxXferSize;            // LPC5410x usart Rx receive size 
    uint32_t RxXferCount;           // LPC5410x usart Rx receive counter 
    
    IRQn_Type irq_number;
    uint32_t rsem;                    // liteos mutex for uart read
    uint32_t wsem;                    // liteos mutex for uart write
    uint8_t *pbuf;                     // uart receive data buf
    uint8_t *dualbuf;                  // uart receive data buf
    uint8_t *interrbuf;                // uart receive data buf
    uint32_t size;                       // uart receive data buf size
    uint32_t readcnt;                    // received data bytes
}los_dev_uart_t;

// init uart puarts[uartidx], puarts please see los_dev_st_uart.c
int los_dev_uart_init(int uartidx, int baudrate, char *buf, int size);
// read data from puarts[uartidx] , puarts please see los_dev_st_uart.c
int los_dev_uart_read(int uartidx, char *outbuf, int rsize, int mstimout);
// write data to puarts[uartidx] , puarts please see los_dev_st_uart.c
int los_dev_uart_write(int uartidx, char *inbuf, int wsize, int mstimout);

void los_dev_uart_clean(int uartidx);
    
/* uart irq handler */
void los_dev_uart_irqhandler(int uartidx);

#endif
