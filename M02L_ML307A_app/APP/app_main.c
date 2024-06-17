/*
 * app_main.c
 *
 *  Created on: Feb 23, 2022
 *      Author: idea
 */
#include "CONFIG.h"
#include "HAL.h"
#include "app_peripheral.h"
#include "app_central.h"
#include "app_port.h"
#include "app_sys.h"
#include "app_task.h"
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
const uint8_t MacAddr[6] =
    {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif

__HIGH_CODE
void Main_Circulation()
{
    while(1)
    {
        TMOS_SystemProcess();
       	portGpioWakeupIRQHandler();
        portWdtFeed();
    }
}



/*********************************************************************
 * @fn      main
 *
 * @brief   Ö÷º¯Êý
 *
 * @return  none
 */
int main(void)
{
    LSECFG_Capacitance(LSECap_26p);
    LSECFG_Current(LSE_RCur_200);
#if(defined(DCDC_ENABLE)) && (DCDC_ENABLE == TRUE)
    PWR_DCDCCfg(ENABLE);
#endif
    sleep_ext_init();
    myTaskPreInit();
    CH58X_BLEInit();
    HAL_Init();
    appPeripheralInit();
    //appCentralInit();
    myTaskInit();
    Main_Circulation();
}
