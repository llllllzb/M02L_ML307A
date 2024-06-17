/*
 * app_central.c
 *
 *  Created on: Jun 27, 2022
 *      Author: idea
 */
#include "app_central.h"
#include "app_sys.h"
#include "app_task.h"
#include "app_param.h"
#include "app_instructioncmd.h"
#include "aes.h"
#include "app_protocol.h"
#define DISCOVER_SERVER_UUID		0xFFE0
#define DISCOVER_CHAR_UUID			0xFFE1


tmosTaskID appCentralTaskId = INVALID_TASK_ID;
static gapBondCBs_t appCentralGapbondCallBack;
static gapCentralRoleCB_t appCentralRoleCallBack;
static centralConnInfo_s  appCentralConnInfo;
static deviceScanList_s scanList;
static tmosEvents appCentralTaskEventProcess(tmosTaskID taskID, tmosEvents event);

/**
 * @brief   信号读取回调
 */
static void appCentralRssiReadCallBack(uint16_t connHandle, int8_t newRSSI)
{
    LogPrintf(DEBUG_ALL, "Handle(%d),RSSI:%d", connHandle, newRSSI);
}
/**
 * @brief   数据包长度更改回调
 */
static void appCentralHciDataLenChangeCallBack(uint16_t connHandle, uint16_t maxTxOctets, uint16_t maxRxOctets)
{
    LogPrintf(DEBUG_ALL, "Handle(%d),maxTxOctets:%d,maxRxOctets:%d", connHandle, maxTxOctets, maxRxOctets);
}
/**
 * @brief   扫描设备信息回调
 */
static void doDeviceInfoEvent(gapDeviceInfoEvent_t *pEvent)
{
    uint8 i, dataLen, cmd;
    deviceScanInfo_s scaninfo;
    tmos_memset(&scaninfo, 0, sizeof(deviceScanInfo_s));
    tmos_memcpy(scaninfo.addr, pEvent->addr, B_ADDR_LEN);
    scaninfo.rssi = pEvent->rssi;
    scaninfo.addrType = pEvent->addrType;
    scaninfo.eventType = pEvent->eventType;

    if (pEvent->pEvtData != NULL && pEvent->dataLen != 0)
    {
        for (i = 0; i < pEvent->dataLen; i++)
        {
            dataLen = pEvent->pEvtData[i];
            if ((dataLen + i + 1) > pEvent->dataLen)
            {
                return ;
            }
            cmd = pEvent->pEvtData[i + 1];
            switch (cmd)
            {
                //case GAP_ADTYPE_LOCAL_NAME_SHORT:
                case GAP_ADTYPE_LOCAL_NAME_COMPLETE:
                    if (dataLen > 30)
                    {
                        break;
                    }
                    tmos_memcpy(scaninfo.broadcaseName, pEvent->pEvtData + i + 2, dataLen - 1);
                    scaninfo.broadcaseName[dataLen - 1] = 0;
                    scanListAdd(&scaninfo);
                    break;
                default:
                    break;
            }

            i += dataLen;
        }
    }

}
/**
 * @brief   建立蓝牙链接回调
 */
static void doEstablishEvent(gapEstLinkReqEvent_t *pEvent)
{
    char debug[50];
    byteToHexString(pEvent->devAddr, debug, B_ADDR_LEN);
    debug[B_ADDR_LEN * 2] = 0;
    LogPrintf(DEBUG_ALL, "---------------------------------------");
    LogPrintf(DEBUG_ALL, "********Device Connected********");
    LogPrintf(DEBUG_ALL, "Mac: [%s] , AddrType: %d", debug, pEvent->devAddrType);
    LogPrintf(DEBUG_ALL, "connectionHandle: [%d]", pEvent->connectionHandle);
    LogPrintf(DEBUG_ALL, "connRole: [%d]", pEvent->connRole);
    LogPrintf(DEBUG_ALL, "connInterval: [%d]", pEvent->connInterval);
    LogPrintf(DEBUG_ALL, "connLatency: [%d]", pEvent->connLatency);
    LogPrintf(DEBUG_ALL, "connTimeout: [%d]", pEvent->connTimeout);
    LogPrintf(DEBUG_ALL, "clockAccuracy: [%d]", pEvent->clockAccuracy);
    LogPrintf(DEBUG_ALL, "---------------------------------------");
    memcpy(appCentralConnInfo.devAddr, pEvent->devAddr, B_ADDR_LEN);
    appCentralConnInfo.devAddrType = pEvent->devAddrType;
    appCentralConnInfo.connectionHandle = pEvent->connectionHandle;
    appCentralConnInfo.connRole = pEvent->connRole;
    appCentralConnInfo.connInterval = pEvent->connInterval;
    appCentralConnInfo.connLatency = pEvent->connLatency;
    appCentralConnInfo.connTimeout = pEvent->connTimeout;
    appCentralConnInfo.clockAccuracy = pEvent->clockAccuracy;

    appCentralConnInfo.findUUID = DISCOVER_SERVER_UUID;

    tmos_set_event(appCentralTaskId, APP_DISCOVER_SERVICES_EVENT);
}

/**
 * @brief   蓝牙链接断开回调
 */

static void doTerminatedEvent(gapTerminateLinkEvent_t *pEvent)
{
    LogPrintf(DEBUG_ALL, "---------------------------------------");
    LogPrintf(DEBUG_ALL, "********Device Terminate********");
    LogPrintf(DEBUG_ALL, "connectionHandle: [%d]", pEvent->connectionHandle);
    LogPrintf(DEBUG_ALL, "connRole: [%d]", pEvent->connRole);
    LogPrintf(DEBUG_ALL, "reason: [0x%02X]", pEvent->reason);
    LogPrintf(DEBUG_ALL, "---------------------------------------");
    appCentralConnInfo.connectionHandle = INVALID_CONNHANDLE;
}
/**
 * @brief   主机事件回调
 */
static void appCentralGapRoleEventCallBack(gapRoleEvent_t *pEvent)
{
    //LogPrintf(DEBUG_ALL, "gapRoleEvent==>0x%02X", pEvent->gap.opcode);
    switch (pEvent->gap.opcode)
    {
        case GAP_DEVICE_INIT_DONE_EVENT:
            break;
        case GAP_DEVICE_DISCOVERY_EVENT:
            LogPrintf(DEBUG_ALL, "Central==>discovery done");
            scanListDisplay();
            bleScanCallBack(&scanList);
            break;
        case GAP_ADV_DATA_UPDATE_DONE_EVENT:
            break;
        case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
            break;
        case GAP_END_DISCOVERABLE_DONE_EVENT:
            break;
        case GAP_LINK_ESTABLISHED_EVENT:
            doEstablishEvent(&pEvent->linkCmpl);
            break;
        case GAP_LINK_TERMINATED_EVENT:
            doTerminatedEvent(&pEvent->linkTerminate);
            break;
        case GAP_LINK_PARAM_UPDATE_EVENT:
            break;
        case GAP_RANDOM_ADDR_CHANGED_EVENT:
            break;
        case GAP_SIGNATURE_UPDATED_EVENT:
            break;
        case GAP_AUTHENTICATION_COMPLETE_EVENT:
            break;
        case GAP_PASSKEY_NEEDED_EVENT:
            break;
        case GAP_SLAVE_REQUESTED_SECURITY_EVENT:
            break;
        case GAP_DEVICE_INFO_EVENT:
            doDeviceInfoEvent(&pEvent->deviceInfo);
            break;
        case GAP_BOND_COMPLETE_EVENT:
            break;
        case GAP_PAIRING_REQ_EVENT:
            break;
        case GAP_DIRECT_DEVICE_INFO_EVENT:
            break;
        case GAP_PHY_UPDATE_EVENT:
            break;
        case GAP_EXT_ADV_DEVICE_INFO_EVENT:
            break;
        case GAP_MAKE_PERIODIC_ADV_DONE_EVENT:
            break;
        case GAP_END_PERIODIC_ADV_DONE_EVENT:
            break;
        case GAP_SYNC_ESTABLISHED_EVENT:
            break;
        case GAP_PERIODIC_ADV_DEVICE_INFO_EVENT:
            break;
        case GAP_SYNC_LOST_EVENT:
            break;
        case GAP_SCAN_REQUEST_EVENT:
            break;
    }
}
/**
 * @brief   主机初始化
 * @note
 * @param
 * @return
 */
void appCentralInit(void)
{
    GAPRole_CentralInit();
    appCentralTaskId = TMOS_ProcessEventRegister(appCentralTaskEventProcess);

    appCentralGapbondCallBack.pairStateCB = NULL;
    appCentralGapbondCallBack.passcodeCB = NULL;

    appCentralRoleCallBack.ChangCB = appCentralHciDataLenChangeCallBack;
    appCentralRoleCallBack.eventCB = appCentralGapRoleEventCallBack;
    appCentralRoleCallBack.rssiCB = appCentralRssiReadCallBack;

    appCentralConnInfo.connectionHandle = INVALID_CONNHANDLE;

    GAP_SetParamValue(TGAP_DISC_SCAN, 8000);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, 20);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, 100);
    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, 100);


    GATT_InitClient();
    GATT_RegisterForInd(appCentralTaskId);

    tmos_set_event(appCentralTaskId, APP_START_EVENT);
}

/**
 * @brief   gatt系统消息事件
 * @note
 * @param
 * @return
 */

static void doGattMsgEvent(gattMsgEvent_t *pMsgEvt)
{
    char debug[100];
    uint8 dataLen, infoLen, numOf, i;
    uint8 *pData;
    uint8_t uuid16[16];
    uint16 uuid, start, end, handle;
    LogPrintf(DEBUG_ALL, "gattMsgEvent==>Handle:[%d],Method:[0x%02X]", pMsgEvt->connHandle, pMsgEvt->method);
    switch (pMsgEvt->method)
    {
        //!< ATT Error Response
        case ATT_ERROR_RSP:
            LogPrintf(DEBUG_ALL, "Error Req:%#02X , Error Code:%#02X , Status:%#02X", pMsgEvt->msg.errorRsp.reqOpcode,
                      pMsgEvt->msg.errorRsp.errCode, pMsgEvt->hdr.status);
            break;
        //!< ATT Exchange MTU Request
        case ATT_EXCHANGE_MTU_REQ:
            break;
        //!< ATT Exchange MTU Response
        case ATT_EXCHANGE_MTU_RSP:
            LogPrintf(DEBUG_ALL, "Exchange MTU respon ==>Status:0x%X,RxMTU:%d", pMsgEvt->hdr.status,
                      pMsgEvt->msg.exchangeMTURsp.serverRxMTU);
            break;
        //!< ATT Find Information Request
        case ATT_FIND_INFO_REQ:
            break;
        //!< ATT Find Information Response
        case ATT_FIND_INFO_RSP:
            break;
        //!< ATT Find By Type Value Request
        case ATT_FIND_BY_TYPE_VALUE_REQ:
            break;
        //!< ATT Find By Type Value Response
        case ATT_FIND_BY_TYPE_VALUE_RSP:
            break;
        //!< ATT Read By Type Request
        case ATT_READ_BY_TYPE_REQ:
            break;
        //!< ATT Read By Type Response
        case ATT_READ_BY_TYPE_RSP:
            infoLen = pMsgEvt->msg.readByTypeRsp.len;
            numOf = pMsgEvt->msg.readByTypeRsp.numPairs;
            pData = pMsgEvt->msg.readByTypeRsp.pDataList;
            dataLen = infoLen * numOf;
            LogPrintf(DEBUG_ALL, "numGrps==>%d,Status:%02X", numOf, pMsgEvt->hdr.status);
            if (infoLen != 0)
            {
                //原始数据
                byteToHexString(pData, debug, dataLen);
                debug[dataLen * 2] = 0;
                LogPrintf(DEBUG_ALL, "Data:(%d)[%s]", dataLen, debug);
                //内容反转
                byteArrayInvert(pData, dataLen);
                byteToHexString(pData, debug, dataLen);
                debug[dataLen * 2] = 0;
                LogPrintf(DEBUG_ALL, "Data:(%d)[%s]", dataLen, debug);
                //180AFFFF0024 180F00230020 FEE7001F0017
                for (i = 0; i < numOf; i++)
                {
                    switch (infoLen)
                    {
                        /*
                        GATT_ReadUsingCharUUID
                        [00:00:00] Data:(4)[04000100]
                        [00:00:00] Data:(4)[00010004]
                        */
                        case 4:
                            handle = pData[4 * i + 2];
                            handle <<= 8;
                            handle |= pData[4 * i + 3];
                            LogPrintf(DEBUG_ALL, "findUUID[%04X]==>handle:0x%04X", appCentralConnInfo.findUUID, handle);
                            appCentralConnInfo.findIt = 1;
                            appCentralConnInfo.notifyHandle = handle;
                            break;
                        /*
                        [00:00:00] Data:(7)[02001A0300E1FF]
                        [00:00:00] Data:(7)[FFE100031A0002]
                        */
                        case 7:
                            uuid = pData[7 * i];
                            uuid <<= 8;
                            uuid |= pData[7 * i + 1];

                            handle = pData[7 * i + 2];
                            handle <<= 8;
                            handle |= pData[7 * i + 3];

                            LogPrintf(DEBUG_ALL, "CharUUID: [%04X],handle:0x%04X", uuid, handle);
                            if (uuid == appCentralConnInfo.findUUID)
                            {
                                appCentralConnInfo.findIt = 1;
                                appCentralConnInfo.writeHandle = handle;
                            }
                            break;
                    }
                }

            }
            if (pMsgEvt->hdr.status == bleProcedureComplete)
            {
                LogPrintf(DEBUG_ALL, "discover char done");
                if (appCentralConnInfo.findIt)
                {
                    appCentralConnInfo.findIt = 0;
                    switch (appCentralConnInfo.findUUID)
                    {
                        case GATT_CLIENT_CHAR_CFG_UUID:
                            LogPrintf(DEBUG_ALL, "find my notify UUID");
                            tmos_set_event(appCentralTaskId, APP_NOTIFY_ENABLE_EVENT);
                            //tmos_set_event(appCentralTaskId, APP_WRITEDATA_EVENT);
                            bleConnCallBack();
                            break;
                        case DISCOVER_CHAR_UUID:
                            LogPrintf(DEBUG_ALL, "find my char UUID");
                            tmos_set_event(appCentralTaskId, APP_DISCOVER_NOTIFY_EVENT);
                            break;
                        default:
                            LogPrintf(DEBUG_ALL, "hei,chekcout hear");
                            break;
                    }
                }
            }
            break;
        //!< ATT Read Request
        case ATT_READ_REQ:
            break;
        //!< ATT Read Response
        case ATT_READ_RSP:
            break;
        //!< ATT Read Blob Request
        case ATT_READ_BLOB_REQ:
            break;
        //!< ATT Read Blob Response
        case ATT_READ_BLOB_RSP:
            break;
        //!< ATT Read Multiple Request
        case ATT_READ_MULTI_REQ:
            break;
        //!< ATT Read Multiple Response
        case ATT_READ_MULTI_RSP:
            break;
        //!< ATT Read By Group Type Request
        case ATT_READ_BY_GRP_TYPE_REQ:
            break;
        //!< ATT Read By Group Type Response
        case ATT_READ_BY_GRP_TYPE_RSP:
            infoLen = pMsgEvt->msg.readByGrpTypeRsp.len;
            numOf = pMsgEvt->msg.readByGrpTypeRsp.numGrps;
            pData = pMsgEvt->msg.readByGrpTypeRsp.pDataList;
            dataLen = infoLen * numOf;
            LogPrintf(DEBUG_ALL, "numGrps==>%d,Status:%02X", numOf, pMsgEvt->hdr.status);
            if (infoLen != 0)
            {
                //原始数据
                byteToHexString(pData, debug, dataLen);
                debug[dataLen * 2] = 0;
                LogPrintf(DEBUG_ALL, "Data:(%d)[%s]", dataLen, debug);
                //内容反转
                byteArrayInvert(pData, dataLen);
                byteToHexString(pData, debug, dataLen);
                debug[dataLen * 2] = 0;
                LogPrintf(DEBUG_ALL, "Data:(%d)[%s]", dataLen, debug);
                //180AFFFF0024 180F00230020 FEE7001F0017
                for (i = 0; i < numOf; i++)
                {
                    switch (infoLen)
                    {
                        case 6:
                            uuid = pData[6 * i];
                            uuid <<= 8;
                            uuid |= pData[6 * i + 1];

                            end = pData[6 * i + 2];
                            end <<= 8;
                            end |= pData[6 * i + 3];

                            start = pData[6 * i + 4];
                            start <<= 8;
                            start |= pData[6 * i + 5];

                            LogPrintf(DEBUG_ALL, "ServUUID: [%04X],start:0x%04X,end:0x%04X", uuid, start, end);

                            //test
                            if (uuid == appCentralConnInfo.findUUID)
                            {
                                appCentralConnInfo.findStart = start;
                                appCentralConnInfo.findEnd = end;
                                appCentralConnInfo.findIt = 1;
                            }
                            break;
                        case 20:
                            memcpy(uuid16, pData + (20 * i), 16);
                            end = pData[20 * i + 16];
                            end <<= 8;
                            end |= pData[20 * i + 17];

                            start = pData[20 * i + 18];
                            start <<= 8;
                            start |= pData[20 * i + 19];
                            byteToHexString(uuid16, debug, 16);
                            debug[32] = 0;
                            LogPrintf(DEBUG_ALL, "ServUUID: [%s],start:0x%04X,end:0x%04X", debug, start, end);
                            break;
                    }
                }

            }
            if (pMsgEvt->hdr.status == bleProcedureComplete)
            {
                LogPrintf(DEBUG_ALL, "discover server done");
                if (appCentralConnInfo.findIt)
                {
                    appCentralConnInfo.findIt = 0;
                    tmos_set_event(appCentralTaskId, APP_DISCOVER_CHAR_EVENT);

                    LogPrintf(DEBUG_ALL, "find my server UUID");
                }
            }
            break;
        //!< ATT Write Request
        case ATT_WRITE_REQ:
            break;
        //!< ATT Write Response
        case ATT_WRITE_RSP:
            if (pMsgEvt->hdr.status == SUCCESS)
            {
                LogPrintf(DEBUG_ALL, "write respon success");
            }
            else
            {
                LogPrintf(DEBUG_ALL, "write respon fail , status 0x%X", pMsgEvt->hdr.status);
            }
            break;
        //!< ATT Prepare Write Request
        case ATT_PREPARE_WRITE_REQ:
            break;
        //!< ATT Prepare Write Response
        case ATT_PREPARE_WRITE_RSP:
            break;
        //!< ATT Execute Write Request
        case ATT_EXECUTE_WRITE_REQ:
            break;
        //!< ATT Execute Write Response
        case ATT_EXECUTE_WRITE_RSP:
            break;
        //!< ATT Handle Value Notification
        case ATT_HANDLE_VALUE_NOTI:
            pData = pMsgEvt->msg.handleValueNoti.pValue;
            dataLen = pMsgEvt->msg.handleValueNoti.len;
            byteToHexString(pData, debug, dataLen);
            debug[dataLen * 2] = 0;
            LogPrintf(DEBUG_ALL, "Notify(%d)[%s]", pMsgEvt->msg.handleValueNoti.handle, debug);
            char dencrypt[256];
            uint8_t dencryptlen;
            if(dencryptData(dencrypt, &dencryptlen, (char *)pData, dataLen)!=0)
            {
            	dencrypt[dencryptlen]=0;
                instructionParser((uint8_t *)dencrypt, dencryptlen, BLE_MODE, NULL);
            }
            break;
        //!< ATT Handle Value Indication
        case ATT_HANDLE_VALUE_IND:
            break;
        //!< ATT Handle Value Confirmation
        case ATT_HANDLE_VALUE_CFM:
            break;
    }
    GATT_bm_free(&pMsgEvt->msg, pMsgEvt->method);
}

/**
 * @brief   处理系统消息事件
 * @note
 * @param
 * @return
 */
static void doSysEventMsg(tmos_event_hdr_t *pMsg)
{
    switch (pMsg->event)
    {
        case GATT_MSG_EVENT:
            doGattMsgEvent((gattMsgEvent_t *)pMsg);
            break;
        default:
            LogPrintf(DEBUG_ALL, "Unprocessed Event 0x%02X", pMsg->event);
            break;
    }
}
/**
 * @brief   主机事件处理
 * @note
 * @param
 * @return
 */
static tmosEvents appCentralTaskEventProcess(tmosTaskID taskID, tmosEvents event)
{
    uint8_t *pMsg;
    uint8 data[10];
    bStatus_t status;
    attReadByTypeReq_t attReadByTypeReq;
    if (event & SYS_EVENT_MSG)
    {
        if ((pMsg = tmos_msg_receive(appCentralTaskId)) != NULL)
        {
            doSysEventMsg((tmos_event_hdr_t *) pMsg);
            tmos_msg_deallocate(pMsg);
        }
        return (event ^ SYS_EVENT_MSG);
    }
    if (event & APP_START_EVENT)
    {
        GAPRole_CentralStartDevice(appCentralTaskId, &appCentralGapbondCallBack, &appCentralRoleCallBack);
        return event ^ APP_START_EVENT;
    }
    if (event & APP_DISCOVER_SERVICES_EVENT)
    {
        status = GATT_DiscAllPrimaryServices(appCentralConnInfo.connectionHandle, appCentralTaskId);
        LogPrintf(DEBUG_ALL, "GATT_DiscAllPrimaryServices==>ret:0x%02X", status);
        return event ^ APP_DISCOVER_SERVICES_EVENT;
    }
    if (event & APP_DISCOVER_CHAR_EVENT)
    {
        appCentralConnInfo.findUUID = DISCOVER_CHAR_UUID;
        status = GATT_DiscAllChars(appCentralConnInfo.connectionHandle, appCentralConnInfo.findStart,
                                   appCentralConnInfo.findEnd,
                                   appCentralTaskId);
        LogPrintf(DEBUG_ALL, "GATT_DiscAllChars==>ret:0x%02X", status);
        return event ^ APP_DISCOVER_CHAR_EVENT;
    }
    if (event & APP_DISCOVER_NOTIFY_EVENT)
    {
        memset(&attReadByTypeReq, 0, sizeof(attReadByTypeReq_t));
        attReadByTypeReq.startHandle = appCentralConnInfo.findStart;
        attReadByTypeReq.endHandle = appCentralConnInfo.findEnd;
        attReadByTypeReq.type.len = ATT_BT_UUID_SIZE;
        attReadByTypeReq.type.uuid[0] = LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
        attReadByTypeReq.type.uuid[1] = HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
        appCentralConnInfo.findUUID = GATT_CLIENT_CHAR_CFG_UUID;
        status = GATT_ReadUsingCharUUID(appCentralConnInfo.connectionHandle, &attReadByTypeReq, appCentralTaskId);
        LogPrintf(DEBUG_ALL, "GATT_ReadUsingCharUUID==>ret:0x%02X", status);
        return event ^ APP_DISCOVER_NOTIFY_EVENT;
    }
    if (event & APP_NOTIFY_ENABLE_EVENT)
    {
        data[0] = 0x01;
        data[1] = 0x00;
        LogMessage(DEBUG_ALL, "enable notify");
        status = centralWrite(appCentralConnInfo.notifyHandle, data, 2);
        if (status == blePending)
        {
            LogPrintf(DEBUG_ALL, "blePending...");
            tmos_start_task(appCentralTaskId, APP_NOTIFY_ENABLE_EVENT, MS1_TO_SYSTEM_TIME(100));
        }
        return event ^ APP_NOTIFY_ENABLE_EVENT;
    }
    if (event & APP_WRITEDATA_EVENT)
    {
        uint8_t sendData[128];
        uint8_t aes[256],aesLen;
        sprintf(sendData, "SN:%s,%d,%.2f,%d", dynamicParam.SN, dynamicParam.startUpCnt, sysinfo.outsidevoltage, getBatteryLevel());
        LogPrintf(DEBUG_ALL, "try to send %s", sendData);
        encryptData(aes, &aesLen, sendData, strlen(sendData));
        status = centralSendData(aes,aesLen);
        if (status == blePending)
        {
            LogPrintf(DEBUG_ALL, "blePending...");
            tmos_start_task(appCentralTaskId, APP_WRITEDATA_EVENT, MS1_TO_SYSTEM_TIME(100));
        }
        return event ^ APP_WRITEDATA_EVENT;
    }
    return 0;
}

/**
 * @brief   启动扫描
 * @note
 * @param
 * @return
 */

void centralStartDisc(void)
{
    bStatus_t status;
    status = GAPRole_CentralStartDiscovery(DEVDISC_MODE_ALL, TRUE, FALSE);
    LogPrintf(DEBUG_ALL, "Central==>Discovery , status:0x%x", status);
    scanListClear();
}
/**
 * @brief   建立链接
 * @note
 * @param
 * @return
 */

void centralEstablish(uint8_t *mac, uint8_t addrType)
{
    bStatus_t status;
    status = GAPRole_CentralEstablishLink(TRUE, FALSE, addrType, mac);
    LogPrintf(DEBUG_ALL, "Central==>Establish , status:0x%x", status);
}

void centralTerminate(void)
{
    bStatus_t status;
    status = GAPRole_TerminateLink(appCentralConnInfo.connectionHandle);
    LogPrintf(DEBUG_ALL, "Central==>Terminate , status:0x%x", status);
}

void centralExchangeMtc(void)
{
    bStatus_t ret;
    attExchangeMTUReq_t pReq;
    pReq.clientRxMTU = BLE_BUFF_MAX_LEN - 4;
    ret = GATT_ExchangeMTU(appCentralConnInfo.connectionHandle, &pReq, appCentralTaskId);
    LogPrintf(DEBUG_ALL, "centralExchangeMTU==>Ret:0x%X", ret);
}

bStatus_t centralSendData(uint8 *data, uint8 len)
{
	bStatus_t status;
    status=centralWrite(appCentralConnInfo.writeHandle, data, len);
	return status;
}

void centralStopDisc(void)
{
    bStatus_t status;
    status = GAPRole_CentralCancelDiscovery();
    LogPrintf(DEBUG_ALL, "Central==>Stop Discovery , status:0x%x", status);
}


/**
 * @brief   主动写数据
 * @note
 * @param
 * @return
 */
uint8 centralWrite(uint16 handle, uint8 *data, uint8 len)
{
    attWriteReq_t req;
    uint8 ret = 0x01;
    req.handle = handle;
    req.cmd = 0;
    req.sig = 0;
    req.len = len;
    req.pValue = GATT_bm_alloc(appCentralConnInfo.connectionHandle, ATT_WRITE_REQ, req.len, NULL, 0);
    if (req.pValue != NULL)
    {
        tmos_memcpy(req.pValue, data, len);
        ret = GATT_WriteCharValue(appCentralConnInfo.connectionHandle, &req, appCentralTaskId);
        if (ret != SUCCESS)
        {
            GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
        }
    }
    LogPrintf(DEBUG_ALL, "centralWrite[%04X]==>Ret:0x%x", handle, ret);
    return ret;
}

/**
 * @brief   将扫描到的ble添加入队列中
 * @note
 * @param
 * @return
 */

void scanListAdd(deviceScanInfo_s *devInfo)
{
    uint8_t i, j;
    if (scanList.cnt >= SCAN_LIST_MAX_SIZE)
    {
        return;
    }
    for (i = 0; i < scanList.cnt; i++)
    {
        if (devInfo->rssi > scanList.list[i].rssi)
        {
            for (j = scanList.cnt; j > i; j--)
            {
                scanList.list[j] = scanList.list[j - 1];
            }
            tmos_memcpy(&scanList.list[i], devInfo, sizeof(deviceScanInfo_s));
            scanList.cnt++;
            return;
        }
    }
    //copy to last
    tmos_memcpy(&scanList.list[scanList.cnt++], devInfo, sizeof(deviceScanInfo_s));
}
/**
 * @brief   清空队列
 * @note
 * @param
 * @return
 */

void scanListClear(void)
{
    scanList.cnt = 0;
}

/**
 * @brief   显示队列
 * @note
 * @param
 * @return
 */

void scanListDisplay(void)
{
    char debug[20];
    uint8_t i;
    for (i = 0; i < scanList.cnt; i++)
    {
        byteToHexString(scanList.list[i].addr, debug, 6);
        debug[12] = 0;
        LogPrintf(DEBUG_ALL, "BLE(%02u)==>[%s][%s][%d][%d]", i + 1, debug, scanList.list[i].broadcaseName,
                  scanList.list[i].rssi,
                  scanList.list[i].addrType);
    }
}
