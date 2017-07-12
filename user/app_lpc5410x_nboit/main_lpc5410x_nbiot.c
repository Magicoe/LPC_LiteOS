#include "los_sys.h"
#include "los_tick.h"
#include "los_task.ph"
#include "los_config.h"

#include "los_bsp_led.h"
#include "los_bsp_key.h"
#include "los_bsp_uart.h"
#include "los_inspect_entry.h"
#include "los_demo_entry.h"

#include <string.h>

#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include <stdbool.h>

#include "lpcxpresso_5410x_rtc.h"       /* MGN: LiteOS do not define RTC API */

extern void LOS_EvbSetup(void);

static UINT32 g_uwboadTaskID;
LITE_OS_SEC_TEXT VOID LOS_BoadExampleTskfunc(VOID)
{
    while (1)
    {
        LOS_EvbLedControl(LOS_LED1, LED_ON); // LED1 is Blue, 2 is Green, 3 is Red
        LOS_TaskDelay(500);
        LOS_EvbLedControl(LOS_LED1, LED_OFF);
        LOS_TaskDelay(500);
        rtc_get((rtc_datetime_t *)&gRTCDate);
        /* print default time */
        PRINTF("Current datetime: %04d-%02d-%02d %02d:%02d:%02d\r\n",
               gRTCDate.year,
               gRTCDate.month,
               gRTCDate.day,
               gRTCDate.hour,
               gRTCDate.minute,
               gRTCDate.second);
    }
}

void LOS_BoadExampleEntry(void)
{
    UINT32 uwRet;
    TSK_INIT_PARAM_S stTaskInitParam;

    (VOID)memset((void *)(&stTaskInitParam), 0, sizeof(TSK_INIT_PARAM_S));
    stTaskInitParam.pfnTaskEntry = (TSK_ENTRY_FUNC)LOS_BoadExampleTskfunc;
    stTaskInitParam.uwStackSize = LOSCFG_BASE_CORE_TSK_IDLE_STACK_SIZE;
    stTaskInitParam.pcName = "BoardDemo";
    stTaskInitParam.usTaskPrio = 10;
    uwRet = LOS_TaskCreate(&g_uwboadTaskID, &stTaskInitParam);

    if (uwRet != LOS_OK)
    {
        return;
    }
    return;
}

/*****************************************************************************
Function    : main
Description : Main function entry
Input       : None
Output      : None
Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT
int main(void)
{
    UINT32 uwRet;
    /*
        add you hardware init code here
        for example flash, i2c , system clock ....
     */
    /* Init hardware*/
    /* attach 12 MHz clock to USART0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    
    /* Enable the asynchronous bridge */
    SYSCON->ASYNCAPBCTRL = 1;
    /* Use 12 MHz clock for some of the Ctimers */
    CLOCK_AttachClk(kIRC12M_to_ASYNC_APB);
    BOARD_InitPins();
    BOARD_BootClockPLL96M(); /* Rev B device can only support max core frequency to 96Mhz.
                                Rev C device can support 100Mhz,use BOARD_BootClockPLL100M() to boot core to 100Mhz. 
                                DEVICE_ID1 register in SYSCON shows the device version.
                                More details please refer to user manual and errata. */
                               
    /* MGN, udpate sys_clk_freq */
    SystemCoreClockUpdate();
    extern volatile unsigned int sys_clk_freq;
    sys_clk_freq = SystemCoreClock;
    
    gRTCDate.day    = SYS_SET_DAY;
    gRTCDate.month  = SYS_SET_MONTH;
    gRTCDate.year   = SYS_SET_YEAR;
    gRTCDate.hour   = SYS_SET_HOUR;
    gRTCDate.minute = SYS_SET_MIN;
    gRTCDate.second = SYS_SET_SEC;
    rtc_init((rtc_datetime_t *)&gRTCDate);
    
    /*Init LiteOS kernel */
    uwRet = LOS_KernelInit();
    if (uwRet != LOS_OK) {
        return LOS_NOK;
    }
    /* Enable LiteOS system tick interrupt */
    LOS_EnableTick();

    /*
        Notice: add your code here
        here you can create task for your function 
        do some hw init that need after systemtick init
     */
    LOS_EvbSetup();//init the device on the dev baord

    //LOS_Demo_Entry();

    //LOS_Inspect_Entry();

    LOS_BoadExampleEntry();
    
    LOS_OceanCon_NB_Sample();

    /* Kernel start to run */
    LOS_Start();
    for (;;);
    /* Replace the dots (...) with your own code. */
}
