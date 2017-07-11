#include "los_bsp_key.h"

#ifdef LOS_LPC5410X
    #include "lpcxpresso_5410x_key.h"
#endif

/*****************************************************************************
 Function    : LOS_EvbKeyInit
 Description : Key init
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void LOS_EvbKeyInit(void)
{
#ifdef LOS_LPC5410X
    key_init();
#endif
    return;
}

/*****************************************************************************
 Function    : LOS_EvbGetKeyVal
 Description : Get key value
 Input       : KeyNum
 Output      : None
 Return      : None
 *****************************************************************************/
uint8_t LOS_EvbGetKeyVal(int KeyNum)
{
#ifdef LOS_LPC5410X
    if(KeyNum > KEY_NUM)
    {
        return LOS_GPIO_ERR;
    }
    return key_value(KeyNum);
#else
    return LOS_GPIO_ERR;
#endif
}

