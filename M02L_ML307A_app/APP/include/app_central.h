/*
 * app_central.h
 *
 *  Created on: Jun 27, 2022
 *      Author: idea
 */

#ifndef APP_INCLUDE_APP_CENTRAL_H_
#define APP_INCLUDE_APP_CENTRAL_H_

#include "config.h"

#define APP_START_EVENT             				0x0001
#define APP_DISCOVER_SERVICES_EVENT                 0x0002
#define APP_DISCOVER_CHAR_EVENT						0x0004
#define APP_DISCOVER_NOTIFY_EVENT					0x0008
#define APP_NOTIFY_ENABLE_EVENT						0x0010
#define APP_WRITEDATA_EVENT							0x1000

#define SCAN_LIST_MAX_SIZE							15

typedef struct
{
    uint8 addr[B_ADDR_LEN];
    uint8 addrType;
    uint8 eventType;
    uint8 broadcaseName[31];
    int8  rssi;
} deviceScanInfo_s;


typedef struct
{
    uint8_t cnt;
    deviceScanInfo_s list[SCAN_LIST_MAX_SIZE];
} deviceScanList_s;

typedef struct
{
    uint8_t devAddr[B_ADDR_LEN]; //!< Device address of link
    uint8_t devAddrType;         //!< Device address type: @ref GAP_ADDR_TYPE_DEFINES
    uint8_t connRole;            //!< Connection formed as Master or Slave
    uint8_t clockAccuracy;       //!< Clock Accuracy
    uint8_t findIt;
    uint16_t connInterval;       //!< Connection Interval
    uint16_t connLatency;        //!< Connection Latency
    uint16_t connTimeout;        //!< Connection Timeout
    uint16_t connectionHandle;   //!< Connection Handle from controller used to ref the device
    uint16_t findUUID;
    uint16_t findStart;
    uint16_t findEnd;
    uint16_t notifyHandle;
    uint16_t writeHandle;

} centralConnInfo_s;

extern tmosTaskID appCentralTaskId;
void appCentralInit(void);

void centralStartDisc(void);
void centralEstablish(uint8_t *mac, uint8_t addrType);
void centralTerminate(void);
void centralExchangeMtc(void);
bStatus_t centralSendData(uint8 *data, uint8 len);
void centralStopDisc(void);
uint8 centralWrite(uint16 handle, uint8 *data, uint8 len);

void scanListAdd(deviceScanInfo_s *devInfo);
void scanListClear(void);
void scanListDisplay(void);


#endif /* APP_INCLUDE_APP_CENTRAL_H_ */
