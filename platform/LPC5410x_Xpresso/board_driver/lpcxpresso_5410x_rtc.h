
#ifndef _APP_RTC_H_
#define _APP_RTC_H_

#include "stdint.h"
#include "fsl_rtc.h"

#define LPC_ENABLE_RTC_ALARM    0 /* 0-disable alarm function. 1-enable */

#define SYS_SET_YEAR  ((((__DATE__[7] - '0') * 10 + (__DATE__[8] - '0')) * 10 + (__DATE__[9] - '0')) * 10 + (__DATE__[10] - '0'))
#define SYS_SET_MONTH (__DATE__ [2] == 'n' ? 0 : __DATE__ [2] == 'b' ? 1 : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 2 : 3) : __DATE__ [2] == 'y' ? 4 : __DATE__ [2] == 'n' ? 5 : __DATE__ [2] == 'l' ? 6 : __DATE__ [2] == 'g' ? 7 : __DATE__ [2] == 'p' ? 8 : __DATE__ [2] == 't' ? 9 : __DATE__ [2] == 'v' ? 10 : 11)
#define SYS_SET_DAY   ((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10 + (__DATE__ [5] - '0'))     

#define SYS_SET_HOUR  ((__TIME__ [0] == ' ' ? 0 : __TIME__ [0] - '0') * 10 + (__TIME__ [1] - '0'))  
#define SYS_SET_MIN   ((__TIME__ [3] == ' ' ? 0 : __TIME__ [3] - '0') * 10 + (__TIME__ [4] - '0'))  
#define SYS_SET_SEC   ((__TIME__ [6] == ' ' ? 0 : __TIME__ [6] - '0') * 10 + (__TIME__ [7] - '0'))  
    
extern volatile uint8_t gLPC5410xRTCBusyWait;
extern volatile rtc_datetime_t gRTCDate;

extern uint8_t rtc_init(rtc_datetime_t *date);
extern uint8_t rtc_get (rtc_datetime_t *date);
extern uint8_t rtc_set (rtc_datetime_t *date);
extern uint8_t rtc_deinit(void);

#endif
