#include <app_protocol.h>
#include "app_net.h"

#include "app_db.h"
#include "app_gps.h"
#include "app_instructioncmd.h"
#include "app_kernal.h"
#include "app_param.h"
#include "app_sys.h"
#include "app_task.h"
#include "app_server.h"
#include "app_socket.h"
#include "app_jt808.h"

//联网相关结构体

static moduleState_s  moduleState;
static moduleCtrl_s moduleCtrl;
static cmdNode_s *headNode = NULL;


static void gpsUploadChangeFsm(uint8_t fsm);
static void gpsUploadSetSize(uint32_t size);

//指令表
const atCmd_s cmdtable[] =
{
    {AT_CMD, "ATE0"},
    {CPIN_CMD, "AT+CPIN"},
    {CSQ_CMD, "AT+CSQ"},
    {CGREG_CMD, "AT+CGREG"},
    {CEREG_CMD, "AT+CEREG"},
    {QIACT_CMD, "AT+QIACT"},
    {QIDEACT_CMD, "AT+QIDEACT"},
    {QIOPEN_CMD, "AT+QIOPEN"},
    {QISEND_CMD, "AT+QISEND"},
    {CIMI_CMD, "AT+CIMI"},
    {CGSN_CMD, "AT+CGSN"},
    {CCID_CMD, "AT+CCID"},
    {ICCID_CMD, "AT+ICCID"},
    {CMGF_CMD, "AT+CMGF"},
    {CMGR_CMD, "AT+CMGR"},
    {CMGD_CMD, "AT+CMGD"},
    {CMGS_CMD, "AT+CMGS"},
    {CPMS_CMD, "AT+CPMS"},
    {CNMI_CMD, "AT+CNMI"},
    {QSCLK_CMD, "AT+QSCLK"},
    {CFUN_CMD, "AT+CFUN"},
    {QICLOSE_CMD, "AT+QICLOSE"},
    {CGDCONT_CMD, "AT+CGDCONT"},
    {QCFG_CMD, "AT+QCFG"},
    {QICFG_CMD, "AT+QICFG"},
    {QPOWD_CMD, "AT+QPOWD"},
    {QGPS_CMD, "AT+QGPS"},
    {QGPSEND_CMD, "AT+QGPSEND"},
    {QGPSCFG_CMD, "AT+QGPSCFG"},
    {QGPSGNMEA_CMD, "AT+QGPSGNMEA"},
    {QISTATE_CMD, "AT+QISTATE"},
    {QAGPS_CMD, "AT+QAGPS"},
    {QURCCFG_CMD, "AT+QURCCFG"},
    {CGATT_CMD, "AT+CGATT"},
    {QIDNSIP_CMD, "AT+QIDNSIP"},
    {QIMUX_CMD, "AT+QIMUX"},
    {QISACK_CMD, "AT+QISACK"},
    {QINDI_CMD, "AT+QINDI"},
    {QIRD_CMD, "AT+QIRD"},
    {ATA_CMD, "ATA"},
    {CIPMUX_CMD, "AT+CIPMUX"},
    {CIPSTART_CMD, "AT+CIPSTART"},
    {CIPCLOSE_CMD, "AT+CIPCLOSE"},
    {CIPSEND_CMD, "AT+CIPSEND"},
    {CIPQSEND_CMD, "AT+CIPQSEND"},
    {CIPRXGET_CMD, "AT+CIPRXGET"},

    {CIPSTATUS_CMD, "AT+CIPSTATUS"},
    {CFGRI_CMD, "AT+CFGRI"},
    {WIFISCAN_CMD, "AT+WIFISCAN"},
    {CFG_CMD, "AT+CFG"},
	{CIICR_CMD, "AT+CMD"},
	//中移
	{CGACT_CMD, "AT+CGACT"},
	{MIPOPEN_CMD, "AT+MIPOPEN"},
	{MIPCLOSE_CMD, "AT+MIPCLOSE"},
	{MIPSEND_CMD, "AT+MIPSEND"},
	{MIPRD_CMD, "AT+MIPRD"},
	{MIPSACK_CMD, "AT+MIPSACK"},
	{MADC_CMD, "AT+MADC"},
	{MLPMCFG_CMD, "AT+MLPMCFG"},
	{MLBSCFG_CMD, "AT+MLBSCFG"},
	{MLBSLOC_CMD, "AT+MLBSLOC"},
	{CMGL_CMD, "AT+CMGL"},
	{MWIFISCANSTART_CMD, "AT+MWIFISCANSTART"},
	{MWIFISCANSTOP_CMD, "AT+MWIFISCANSTOP"},
	{MCHIPINFO_CMD, "AT+MCHIPINFO"},
	{MCFG_CMD, "AT+MCFG"},
	{AUTHREQ_CMD, "AT*AUTHReq"},
	{MIPCALL_CMD, "AT+MIPCALL"},
	{MCCID_CMD, "AT+MCCID"},
};

/**************************************************
@bref		创建指令输出队列，用于顺序输出
@param
@return
@note
**************************************************/

uint8_t createNode(char *data, uint16_t datalen, uint8_t currentcmd)
{
    cmdNode_s *nextnode;
    cmdNode_s *currentnode;
    //如果链表头未创建，则创建链表头。
    WAKEMODULE;
    if (currentcmd == MWIFISCANSTART_CMD)
    {
		wakeUpByInt(1, 28);
    }
    else
    {
		wakeUpByInt(1, 8);
    }
    if (headNode == NULL)
    {
        headNode = malloc(sizeof(cmdNode_s));
        if (headNode != NULL)
        {
            headNode->currentcmd = currentcmd;
            headNode->data = NULL;
            headNode->data = malloc(datalen);
            if (headNode->data != NULL)
            {
                memcpy(headNode->data, data, datalen);
                headNode->datalen = datalen;
                headNode->nextnode = NULL;
                return 1;
            }
            else
            {
                free(headNode);
                headNode = NULL;
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
    currentnode = headNode;
    do
    {
        nextnode = currentnode->nextnode;
        if (nextnode == NULL)
        {
            nextnode = malloc(sizeof(cmdNode_s));
            if (nextnode != NULL)
            {

                nextnode->currentcmd = currentcmd;
                nextnode->data = NULL;
                nextnode->data = malloc(datalen);
                if (nextnode->data != NULL)
                {
                    memcpy(nextnode->data, data, datalen);
                    nextnode->datalen = datalen;
                    nextnode->nextnode = NULL;
                    currentnode->nextnode = nextnode;
                    nextnode = nextnode->nextnode;
                }
                else
                {
                    free(nextnode);
                    nextnode = NULL;
                    return 0;
                }
            }
            else
            {
                return 0;
            }
        }
        currentnode = nextnode;
    }
    while (nextnode != NULL);

    return 1;
}

/**************************************************
@bref		数据队列输出
@param
@return
@note
**************************************************/

void outputNode(void)
{
    static uint8_t lockFlag = 0;
    static uint8_t sleepTick = 0;
    static uint8_t tickRange = 50;
    cmdNode_s *nextnode;
    cmdNode_s *currentnode;
    if (lockFlag)
    {
        if (sysinfo.lockTick++ >= tickRange)
        {
            lockFlag = 0;
            sysinfo.lockTick = 0;
            LogMessage(DEBUG_ALL, "outputNode==>Unlock");
        }
        return ;
    }
    if (headNode == NULL)
    {
        if (sleepTick > 0)
        {
            sleepTick--;
        }
        else
        {
        	/* 
        		模组关机状态时DTR脚要置低;
        		如果置高的话模组会一直有1.5V左右的电,这样会导致模组可能会关不了机,也开不了机,需要整机断电才行;
        		另外模组关机的时候最好不要阻止这里发完AT指令,不然会一直占着堆空间
        	*/
        	if (moduleState.powerState == 0)	//关机或者正在开关机 DTR都拉低
        	{
            	WAKEMODULE;
            }
            else 
            {
				SLEEPMODULE;
            }
        }
        return ;
    }
    sleepTick = 2;
    currentnode = headNode;
    if (currentnode != NULL)
    {
        nextnode = currentnode->nextnode;
        moduleState.cmd = currentnode->currentcmd;

        //数据发送
        portUartSend(&usart3_ctl, (uint8_t *)currentnode->data, currentnode->datalen);
        if (currentnode->data[0] != 0X78 && currentnode->data[0] != 0x79 && currentnode->data[0] != 0x7E)
        {
            LogMessageWL(DEBUG_ALL, currentnode->data, currentnode->datalen);
        }
        free(currentnode->data);
        free(currentnode);
        if (currentnode->currentcmd == QICLOSE_CMD || currentnode->currentcmd == CMGS_CMD || 
        		currentnode->currentcmd == MWIFISCANSTART_CMD)
        {
            lockFlag = 1;
            if (currentnode->currentcmd == QICLOSE_CMD)
            {
                tickRange = 20;
            }
            else if (currentnode->currentcmd == MWIFISCANSTART_CMD)
            {
				tickRange = 125;
            }
            else
            {
                tickRange = 10;
            }
            LogMessage(DEBUG_ALL, "outputNode==>Lock");
        }
    }
    headNode = nextnode;

}

/**************************************************
@bref		模组指令发送
@param
@return
@note
**************************************************/

uint8_t  sendModuleCmd(uint8_t cmd, char *param)
{
    uint8_t i;
    int16_t cmdtype = -1;
    char sendData[256];
    for (i = 0; i < sizeof(cmdtable) / sizeof(cmdtable[0]); i++)
    {
        if (cmd == cmdtable[i].cmd_type)
        {
            cmdtype = i;
            break;
        }
    }
    if (cmdtype < 0)
    {
        snprintf(sendData, 255, "sendModuleCmd==>No cmd");
        LogMessage(DEBUG_ALL, sendData);
        return 0;
    }
    if (param != NULL && strlen(param) <= 240)
    {
        if (param[0] == '?')
        {
            snprintf(sendData, 255, "%s?\r\n", cmdtable[cmdtype].cmd);

        }
        else
        {
            snprintf(sendData, 255, "%s=%s\r\n", cmdtable[cmdtype].cmd, param);
        }
    }
    else if (param == NULL)
    {
        snprintf(sendData, 255, "%s\r\n", cmdtable[cmdtype].cmd);
    }
    else
    {
        return 0;
    }
    createNode(sendData, strlen(sendData), cmd);
    return 1;
}

void sendModuleDirect(char *param)
{
    createNode(param, strlen(param), 0);
}

/**************************************************
@bref		初始化模块相关使用结构体
@param
@return
@note
**************************************************/

static void moduleInit(void)
{
    memset(&moduleState, 0, sizeof(moduleState_s));
}

/**************************************************
@bref		是否开机按键
@param
@return
@note
**************************************************/
static void modulePressReleaseKey(void)
{
    PWRKEY_HIGH;
    moduleState.powerState = 1;
    LogPrintf(DEBUG_ALL, "PowerOn Done");
}
/**************************************************
@bref		按下开机按键
@param
@return
@note
**************************************************/

static void modulePressPowerKey(void)
{
    PWRKEY_LOW;
    startTimer(25, modulePressReleaseKey, 0);
}
/**************************************************
@bref		模组开机
@param
@return
@note
**************************************************/

void modulePowerOn(void)
{
	moduleReqSet(MODULE_REQUEST_OPEN);
}

/**************************************************
@bref		关机完成
@param
@return
@note
**************************************************/

static void modulePowerOffDone(void)
{
	LogMessage(DEBUG_ALL, "modulePowerOff Done");
	moduleInit();
	POWER_OFF;
}

/**************************************************
@bref		释放关机按键
@param
@return
@note
**************************************************/
static void modulePowerOffRelease(void)
{
	LogMessage(DEBUG_ALL, "modulePowerOffRelease");
	PWRKEY_HIGH;
	startTimer(60, modulePowerOffDone, 0);
}	


/**************************************************
@bref		按下关机按键
@param
@return
@note
**************************************************/
static void modulePowerOffProcess(void)
{
    PWRKEY_LOW;
	startTimer(37, modulePowerOffRelease, 0);
	LogMessage(DEBUG_ALL, "modulePowerOffProcess");
}


/**************************************************
@bref		模组关机
@param
@return
@note
关模组的时间不宜太长，太长可能会因为休眠之前模组关机
同时又立马重新唤醒开启模组，导致最终的结果就是关机
**************************************************/

void modulePowerOff(void)
{
	moduleReqSet(MODULE_REQUEST_CLOSE);

}

/**************************************************
@bref		释放复位按键
@param
@return
@note
**************************************************/

static void moduleReleaseRstkey(void)
{
    moduleState.powerState = 1;
    RSTKEY_HIGH;
}
/**************************************************
@bref		模组复位
@param
@return
@note
**************************************************/

void moduleReset(void)
{
	//moduleState.powerState == 0表示设备处于关机或者处于开关机状态
	//如果已经处于开关机状态，就没必要有这个请求了
	if (moduleState.powerState != 0)
		moduleReqSet(MODULE_REQUEST_RESET);
}

/**************************************************
@bref		模组断电重启
@param
@return
@note
**************************************************/
void moduleShutdownStartup(void)
{
	moduleReqSet(MODULE_REQUESR_SHUTDOWN_OPEN);
}

/**************************************************
@bref		模组开关状态获取
@param
@return    0：关机完毕
		   1：开机完毕
		   2：正在进行开关机
@note
**************************************************/

uint8_t getModulePwrState(void)
{
	/* 请求是关闭且模组电源状态机是关闭状态 */
	if (sysinfo.moduleReq == 0 && sysinfo.moduleFsm == MODULE_FSM_CLOSE_DONE)
		return 0;
	if (sysinfo.moduleReq == 0 && sysinfo.moduleFsm == MODULE_FSM_OPEN_DONE)
	    return 1;
	return 2;
}

/**************************************************
@bref		模组电源请求设置
@param
@return
@note
**************************************************/

void moduleReqSet(uint8_t req)
{
	sysinfo.moduleReq = req;
	LogPrintf(DEBUG_ALL, "moduleReqSet==>%d", sysinfo.moduleReq);
}

/**************************************************
@bref		模组电源请求清除
@param
@return
@note
**************************************************/
void moduleReqClear(void)
{
	if (sysinfo.moduleReq != 0)
	{
		sysinfo.moduleReq = 0;
		LogPrintf(DEBUG_ALL, "moduleReqClear");
	}
}

/**************************************************
@bref		模组电源请求查询
@param
@return
@note
**************************************************/

uint8_t moduleReqGet(void)
{
	return sysinfo.moduleReq;
}

/**************************************************
@bref		模组电源状态机切换
@param
@return
@note
**************************************************/

static void moduleFsmChange(uint8_t fsm)
{
	sysinfo.moduleFsm = fsm;
	sysinfo.moduleFsmTick = 0;
}

/**************************************************
@bref		模组电源状态机
@param
@return
@note
sysinfo.moduleReq为开机、关机、复位、断电复位的请求
这4个请求均为互斥，状态机接收到请求后开始执行并清除掉这个请求
在处理完一个请求之前来第二个请求时，会保存此次请求，直到完成当前请求
当前请求处理完成之后，会判断下一个请求是否合理，才选择是否执行
比如当完成关机请求后，来了一个复位请求，不合理，则清除此次请求

**************************************************/
void moduleRequestTask(void)
{
	switch (sysinfo.moduleFsm)
	{
	case MODULE_FSM_CLOSE_DONE:
		if (sysinfo.moduleReq == MODULE_REQUEST_OPEN)
		{
			LogMessage(DEBUG_ALL, "modulePowerOn");
		    moduleInit();
		    sysinfo.moduleRstFlag = 1;
		    portUartCfg(APPUSART3, 1, 115200, moduleRecvParser);
		    POWER_ON;
		    PWRKEY_HIGH;
		    RSTKEY_HIGH;
		    moduleState.gpsFileHandle = 1;
		    moduleCtrl.scanMode = 0;
		    socketDelAll();
			moduleFsmChange(MODULE_FSM_OPEN_ING1);
		}
		else
		{
			//把模组唤醒tick清除 让mcu更快进入休眠
		    sysinfo.ringWakeUpTick = 0;
		    sysinfo.cmdTick = 0;
		    sysinfo.irqTick = 0;
		}
		moduleReqClear();
		break;
	case MODULE_FSM_OPEN_ING1:
		if (++sysinfo.moduleFsmTick > 5)	//500ms
		{
			PWRKEY_LOW;
			moduleFsmChange(MODULE_FSM_OPEN_ING2);
		}
		break;
	case MODULE_FSM_OPEN_ING2:
		if (++sysinfo.moduleFsmTick > 25)	//2500ms
		{
			PWRKEY_HIGH;
			moduleState.powerState = 1;
			LogMessage(DEBUG_ALL, "modulePowerOnDone");
			moduleFsmChange(MODULE_FSM_OPEN_DONE);
		}
		break;
	case MODULE_FSM_OPEN_ING3:
		
		moduleFsmChange(MODULE_FSM_OPEN_DONE);
		break;
	case MODULE_FSM_OPEN_DONE:
		if (sysinfo.moduleReq == MODULE_REQUEST_CLOSE)
		{
			LogMessage(DEBUG_ALL, "modulePowerOff");
		    portUartCfg(APPUSART3, 0, 115200, NULL);
		    RSTKEY_HIGH;
		    PWRKEY_HIGH;
		    WAKEMODULE;
		    moduleInit();
		    sysinfo.moduleRstFlag = 1;
		    socketDelAll();
		    moduleFsmChange(MODULE_FSM_CLOSE_ING);
		}
		else if (sysinfo.moduleReq == MODULE_REQUEST_RESET)
		{
			RSTKEY_HIGH;
			moduleInit();
			moduleFsmChange(MODULE_FSM_RESET_ING1);
		}
		else if (sysinfo.moduleReq == MODULE_REQUESR_SHUTDOWN_OPEN)
		{
			LogMessage(DEBUG_ALL, "module shut down");
		    portUartCfg(APPUSART3, 0, 115200, NULL);
		    RSTKEY_HIGH;
		    PWRKEY_HIGH;
		    WAKEMODULE;
		    moduleInit();
		    
		    sysinfo.moduleRstFlag = 1;
		    socketDelAll();
			moduleFsmChange(MODULE_FSM_SHUTDOWN_ING);
		}
		moduleReqClear();
		break;
	case MODULE_FSM_CLOSE_ING:
	    if (++sysinfo.moduleFsmTick > 5)	//500ms
		{
			PWRKEY_LOW;
			moduleFsmChange(MODULE_FSM_CLOSE_ING2);
		}
		break;
	case MODULE_FSM_CLOSE_ING2:
		if (++sysinfo.moduleFsmTick > 35)	//3500ms
		{
			PWRKEY_HIGH;
			POWER_OFF;
			LogMessage(DEBUG_ALL, "modulePowerOffDone");
	    	moduleFsmChange(MODULE_FSM_CLOSE_DONE);
		}
		break; 
	case MODULE_FSM_CLOSE_ING3:

		break;
	case MODULE_FSM_RESET_ING1:
		if (++sysinfo.moduleFsmTick > 5)	//500ms
		{
			LogMessage(DEBUG_ALL, "modulePowerReset");
			RSTKEY_LOW;
			moduleFsmChange(MODULE_FSM_RESET_ING2);
		}
		break;
	case MODULE_FSM_RESET_ING2:
		if (++sysinfo.moduleFsmTick > 10)	//1000ms
		{
			LogMessage(DEBUG_ALL, "modulePowerResetDone");
			RSTKEY_HIGH;
			moduleState.powerState = 1;
			moduleFsmChange(MODULE_FSM_OPEN_DONE);
		}
		break;
	case MODULE_FSM_SHUTDOWN_ING:
		LogMessage(DEBUG_ALL, "module shut down done");
		POWER_OFF;
		moduleFsmChange(MODULE_FSM_SHUTDOWN_WAIT);
		break;
	case MODULE_FSM_SHUTDOWN_WAIT:
		if (++sysinfo.moduleFsmTick > 300)
		{
			moduleFsmChange(MODULE_FSM_SHUTDOWN_UP);
		}
		if ((sysinfo.moduleFsmTick % 10) == 0)
			LogPrintf(DEBUG_ALL, "module shut down wait %ds...", sysinfo.moduleFsmTick / 10);
		break;
	case MODULE_FSM_SHUTDOWN_UP:
		LogMessage(DEBUG_ALL, "module shut down up");
	    moduleInit();
	    sysinfo.moduleRstFlag = 1;
	    portUartCfg(APPUSART0, 1, 115200, moduleRecvParser);
	    POWER_ON;
	    PWRKEY_HIGH;
	    RSTKEY_HIGH;
	    moduleState.gpsFileHandle = 1;
	    moduleCtrl.scanMode = 0;
	    socketDelAll();
		moduleFsmChange(MODULE_FSM_OPEN_ING1);
		break;
	default:
		moduleFsmChange(MODULE_FSM_CLOSE_DONE);
		break;
	}
}

/**************************************************
@bref		切换联网状态机
@param
@return
@note
**************************************************/
static void changeProcess(uint8_t fsm)
{
    moduleState.fsmState = fsm;
    moduleState.fsmtick = 0;
    if (moduleState.fsmState != NORMAL_STATUS)
    {
        ledStatusUpdate(SYSTEM_LED_NETOK, 0);
    }
}

/**************************************************
@bref		创建socket
@param
@return
@note
**************************************************/

void openSocket(uint8_t link, char *server, uint16_t port)
{
    char param[100];
    sprintf(param, "%d,\"TCP\",\"%s\",%d,60,2,0", link, server, port);
    //sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(MIPOPEN_CMD, param);
}

/**************************************************
@bref		关闭socket
@param
@return
@note
**************************************************/

void closeSocket(uint8_t link)
{
    char param[10];
    sprintf(param, "%d", link);
    //sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(MIPCLOSE_CMD, param);
}

/**************************************************
@bref		apn配置
@param
@return
@note
**************************************************/

static void netSetCgdcong(char *apn)
{
    char param[100];
    sprintf(param, "1,\"IP\",\"%s\"", apn);
    sendModuleCmd(CGDCONT_CMD, param);
}

/**************************************************
@bref		apn配置
@param
@return
@note
**************************************************/

static void netSetApn(char *apnname, char *apnpassword)
{
    char param[100];
    sprintf(param, "1,1,%s,%s", apnname, apnpassword);
    sendModuleCmd(AUTHREQ_CMD, param);
}


/**************************************************
@bref		模组进入飞行模式
@param
@return
@note
**************************************************/

static void moduleEnterFly(void)
{
    sendModuleCmd(CFUN_CMD, "0");
}

/**************************************************
@bref		模组进入正常模式
@param
@return
@note
**************************************************/

static void moduleExitFly(void)
{
    sendModuleCmd(CFUN_CMD, "1");
}

/**************************************************
@bref		发送socket读取缓存指令
@param
@return
@note
**************************************************/

static void qirdCmdSend(uint8_t link, uint8_t index)
{
    char param[10];
    sprintf(param, "%d,1024", link);
    moduleState.curQirdId = link;
    sendModuleCmd(MIPRD_CMD, param);
}

/**************************************************
@bref		读取缓存
@param
@return
@note
**************************************************/

static void queryRecvBuffer(void)
{
    char param[10];
    if (moduleState.normalLinkQird)
    {
        qirdCmdSend(NORMAL_LINK, moduleState.normalLinkQird);

    }
    else if (moduleState.agpsLinkQird)
    {
        qirdCmdSend(AGPS_LINK, moduleState.agpsLinkQird);

    }
    else if (moduleState.bleLinkQird)
    {
        qirdCmdSend(BLE_LINK, moduleState.bleLinkQird);

    }
    else if (moduleState.jt808LinkQird)
    {
        qirdCmdSend(JT808_LINK, moduleState.jt808LinkQird);

    }
    else if (moduleState.hideLinkQird)
    {
        qirdCmdSend(HIDDEN_LINK, moduleState.hideLinkQird);

    }
}

/**************************************************
@bref		网络请求设置
@param
@return
@note
**************************************************/

void netRequestSet(void)
{
	sysinfo.netRequest = 1;
	LogPrintf(DEBUG_ALL, "netRequestSet==>OK");
}

/**************************************************
@bref		网络请求清除
@param
@return
@note
**************************************************/

void netRequestClear(void)
{
//	if (sysinfo.netRequest)
//	{
//		if (sysparam.MODE != MODE4)
//		{
//			gpsinfo_s newgps;
//			centralPointGet(&newgps);
//			updateHistoryGpsTime(&newgps);
//			protocolSend(NORMAL_LINK, PROTOCOL_12, &newgps);
//			jt808SendToServer(TERMINAL_POSITION,   &newgps);
//		}	
//		sysinfo.netRequest = 0;
//		LogPrintf(DEBUG_ALL, "netRequestClear==>OK");
//	}
	sysinfo.netRequest = 0;
}

/**************************************************
@bref		初始化重新搜网时间
@param
@return
@note		刚上电的时候用
**************************************************/

void noNetTimeInit(void)
{
	sysinfo.noNetTime = 1;
}

/**************************************************
@bref		更新重新搜网时间
@param
@return
@note
**************************************************/

void updateNoNetTime(void)
{
	sysinfo.noNetTime = 1;
	//sysinfo.noNetTime = sysinfo.noNetTime > 5 ? 1 : sysinfo.noNetTime;
	LogPrintf(DEBUG_ALL, "%s==>check net gap:%d", __FUNCTION__, sysinfo.noNetTime);
}

/**************************************************
@bref		联网准备任务
@param
@return
@note
**************************************************/

void netConnectTask(void)
{
    if (moduleState.powerState == 0)
    {
        return;
    }

    moduleState.powerOnTick++;
    switch (moduleState.fsmState)
    {
        case AT_STATUS:
            if (moduleState.atResponOK)
            {
                moduleCtrl.atCount = 0;
                moduleState.atResponOK = 0;
                moduleState.cpinResponOk = 0;
                changeProcess(CPIN_STATUS);
            }
            else
            {
                if (moduleState.fsmtick % 2 == 0)
                {
                    moduleState.powerOnTick = 0;
                    sendModuleCmd(AT_CMD, NULL);
                }
                if (moduleState.fsmtick >= 30)
                {
                    moduleCtrl.atCount++;
                    if (moduleCtrl.atCount >= 2)
                    {
                        moduleCtrl.atCount = 0;
                        modulePowerOn();
                    }
                    else
                    {
                        moduleReset();
                    }
                }
                break;
            }
        case CPIN_STATUS:
            if (moduleState.cpinResponOk)
            {
                moduleState.cpinResponOk = 0;
                moduleState.csqOk = 0;
                netSetCgdcong((char *)sysparam.apn);
                netSetApn((char *)sysparam.apnuser, (char *)sysparam.apnpassword);
                sendModuleCmd(CIMI_CMD, NULL);
                sendModuleCmd(CGSN_CMD, "1");
                changeProcess(CSQ_STATUS);
            }
            else
            {
                if (moduleState.fsmtick % 2 == 0)
                {
                    sendModuleCmd(CPIN_CMD, "?");
                }
                if (moduleState.fsmtick >= 30)
                {
                    moduleReset();
                }
                break;
            }
        case CSQ_STATUS:
            if (moduleState.csqOk)
            {
                moduleState.csqOk = 0;
                moduleState.cgregOK = 0;
                moduleState.ceregOK = 0;
                moduleCtrl.csqCount = 0;
                sendModuleCmd(CEREG_CMD, "2");
                sendModuleCmd(CPMS_CMD, "\"ME\",\"ME\",\"ME\"");	/*修改短信存储位置*/
	        	sendModuleCmd(CNMI_CMD, "2,2");						/*第二个参数表示缓存在ME中, 不立即上报*/
	        	sendModuleCmd(CMGF_CMD, "1");						/*TEXT模式*/
                changeProcess(CGREG_STATUS);
                netResetCsqSearch();
            }
            else
            {
                sendModuleCmd(CSQ_CMD, NULL);
                if (moduleCtrl.csqTime == 0)
                {
                    moduleCtrl.csqTime = 90;
                }
                if (moduleState.fsmtick >= moduleCtrl.csqTime)
                {
                    moduleCtrl.csqCount++;
                    if (moduleCtrl.csqCount >= 3)
                    {
                        moduleCtrl.csqCount = 0;
                        //3次搜索不到网络时，如果没有gps请求，则关机
                        if (sysinfo.gpsRequest != 0)
                        {
							moduleReset();
                        }
                        else
                        {
                            modeTryToStop();
                        }
                    }
                    else
                    {
                        moduleEnterFly();
                        startTimer(80, moduleExitFly, 0);
                    }
                    changeProcess(AT_STATUS);
                }
                break;
            }
        case CGREG_STATUS:
            if (moduleState.ceregOK || moduleState.cgregOK)
            {
                moduleCtrl.cgregCount = 0;
                moduleState.ceregOK = 0;
                moduleState.cgregOK = 0;
                changeProcess(CONFIG_STATUS);
            }
            else
            {
                //sendModuleCmd(CGREG_CMD, "?");
                sendModuleCmd(CEREG_CMD, "?");
                if (moduleState.fsmtick >= 90)
                {
                    moduleCtrl.cgregCount++;
                    if (moduleCtrl.cgregCount >= 2)
                    {
                        moduleCtrl.cgregCount = 0;
                        //2次注册不上基站时，如果没有gps请求，则关机
                        if (sysinfo.gpsRequest != 0)
                        {
                            moduleReset();
                        }
                        else
                        {
                            modeTryToStop();
                        }

                        LogMessage(DEBUG_ALL, "Register timeout,try to skip");
                    }
                    else
                    {
                        //sendModuleCmd(CGATT_CMD, "1");
                        changeProcess(AT_STATUS);
                    }
                }
                break;
            }
        case CONFIG_STATUS:
//        	sendModuleCmd(CPMS_CMD, "\"ME\",\"ME\",\"ME\"");	/*修改短信存储位置*/
//        	sendModuleCmd(CNMI_CMD, "2,2");						/*第二个参数表示缓存在ME中, 不立即上报*/
//        	sendModuleCmd(CMGF_CMD, "1");						/*TEXT模式*/
			sendModuleCmd(CEREG_CMD, "2");
			sendModuleCmd(CGSN_CMD, "1");
			sendModuleCmd(CIMI_CMD, NULL);
			sendModuleCmd(MCCID_CMD, NULL);
			sendModuleCmd(MCFG_CMD, "ri,1");
			queryTemperture();
			queryBatVoltage();
            if (sysparam.MODE == MODE4 && sysinfo.netRequest == 0)
            {
				moduleSleepCtl(1);
				changeProcess(OFFLINE_STATUS);
            }
            else
            {
				changeProcess(QIACT_STATUS);
            }
            break;
        case QIACT_STATUS:
            if (moduleState.qipactOk)
            {
                moduleState.qipactOk = 0;
                moduleCtrl.qipactCount = 0;
                changeProcess(NORMAL_STATUS);
            }
            else
            {
                sendModuleCmd(MIPCALL_CMD, "1,1");
                sendModuleCmd(MIPCALL_CMD, "?");
                if (moduleState.fsmtick >= 45)
                {
                    LogMessage(DEBUG_ALL, "try QIPACT again");
                    moduleState.qipactSet = 0;
                    moduleState.fsmtick = 0;
                    moduleCtrl.qipactCount++;
                    if (moduleCtrl.qipactCount >= 3)
                    {
                        moduleCtrl.qipactCount = 0;
                        moduleReset();
                    }
                    else
                    {
                        changeProcess(CPIN_STATUS);
                    }
                }
                break;
            }
        case NORMAL_STATUS:
            socketSchedule();
            queryRecvBuffer();
            break;
        case OFFLINE_STATUS:
			if (sysparam.MODE != MODE4 || sysinfo.netRequest != 0)
			{
				gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
				changeProcess(CPIN_STATUS);
			}
        	break;
        default:
            changeProcess(AT_STATUS);
            break;
    }
    moduleState.fsmtick++;
}


/**************************************************
@bref		AT+CSQ	指令解析
@param
@return
@note
**************************************************/

static void csqParser(uint8_t *buf, uint16_t len)
{
    int index, indexa, datalen;
    uint8_t *rebuf;
    uint16_t  relen;
    char restore[5];
    index = my_getstrindex((char *)buf, "+CSQ:", len);
    if (index >= 0)
    {
        rebuf = buf + index;
        relen = len - index;
        indexa = getCharIndex(rebuf, relen, ',');
        if (indexa > 6)
        {
            datalen = indexa - 6;
            if (datalen > 5)
                return;
            memset(restore, 0, 5);
            strncpy(restore, (char *)rebuf + 6, datalen);
            moduleState.rssi = atoi(restore);
            if (moduleState.rssi >= 6 && moduleState.rssi <= 31)
                moduleState.csqOk = 1;
        }
    }
}

/**************************************************
@bref		AT+CREG	指令解析
@param
@return
@note
**************************************************/

static void cgregParser(uint8_t *buf, uint16_t len)
{
    int index, datalen;
    uint8_t *rebuf;
    uint16_t  relen, i;
    char restore[50];
    uint8_t cnt;
    uint8_t type = 0;
   	uint16_t lac;
    uint32_t cid;
    index = my_getstrindex((char *)buf, "+CGREG:", len);
    if (index < 0)
    {
        type = 1;
        index = my_getstrindex((char *)buf, "+CEREG:", len);
    }
    if (index >= 0)
    {
        rebuf = buf + index;
        relen = len - index;
        datalen = 0;
        cnt = 0;
        restore[0] = 0;
        for (i = 0; i < relen; i++)
        {
            if (rebuf[i] == ',' || rebuf[i] == '\r' || rebuf[i] == '\n')
            {
                if (restore[0] != 0)
                {
                    restore[datalen] = 0;
                    cnt++;
                    datalen = 0;
                    switch (cnt)
                    {
                        case 2:
                            if (restore[0] == '1' || restore[0] == '5')
                            {
                                if (type)
                                {
                                    moduleState.ceregOK = 1;
                                }
                                else
                                {
                                    moduleState.cgregOK = 1;
                                }
                            }
                            else
                            {
                                return ;
                            }
                            break;
                        case 3:
                            lac = strtoul(restore + 1, NULL, 16);
                            //LogPrintf(DEBUG_ALL, "LAC=%s,0x%X", restore, moduleState.lac);
                            break;
                        case 4:
                            cid = strtoul(restore + 1, NULL, 16);
                            LogPrintf(DEBUG_ALL, "current lac:0x%x cid:0x%x  last lac:0x%x cid:0x%x ", 
                            		lac, cid, moduleState.lac, moduleState.cid);
                            //要实时检测基站信息的变化
                            if (((moduleState.lac != lac && 0 != moduleState.lac) ||
                            	 (moduleState.cid != cid && 0 != moduleState.cid)))
                            {
								LogPrintf(DEBUG_ALL, "LBS different, lbs wifireq");
								lbsRequestSet(DEV_EXTEND_OF_MY);
								wifiRequestSet(DEV_EXTEND_OF_MY);
                            }
                           	moduleState.lac = lac;
                           	moduleState.cid = cid;
                            LogPrintf(DEBUG_ALL, "LAC=0x%X CID=0x%X",  moduleState.lac, moduleState.cid);
                            break;

                    }
                    restore[0] = 0;
                }
            }
            else
            {
                restore[datalen] = rebuf[i];
                datalen++;
                if (datalen >= 50)
                {
                    return ;
                }

            }
        }
    }
}

/**************************************************
@bref		AT+CIMI	指令解析
@param
@return
@note
	460064814034016
其中"460"是中国的MCC，"06"是中国联通的MNC。接下来的10位数字"4814034016"是该移动用户的唯一标识符，由移动网络分配给该用户。
**************************************************/

static void cimiParser(uint8_t *buf, uint16_t len)
{
    int16_t index;
    uint8_t *rebuf;
    uint16_t  relen;
    uint8_t i;
    rebuf = buf;
    relen = len;
    index = getCharIndex(rebuf, relen, '\n');
    if (index < 0)
    {
        return;
    }
    rebuf = rebuf + index + 1;
    relen = relen - index - 1;
    index = getCharIndex(rebuf, relen, '\r');
    if (index == 15)
    {
        for (i = 0; i < index; i++)
        {
            moduleState.IMSI[i] = rebuf[i];
        }
        moduleState.IMSI[index] = 0;
        moduleState.mcc = (moduleState.IMSI[0] - '0') * 100 + (moduleState.IMSI[1] - '0') * 10 + moduleState.IMSI[2] - '0';
        moduleState.mnc = (moduleState.IMSI[3] - '0') * 10 + moduleState.IMSI[4] - '0';
        LogPrintf(DEBUG_ALL, "IMSI:%s,MCC=%d,MNC=%02d", moduleState.IMSI, moduleState.mcc, moduleState.mnc);
    }
}


/**************************************************
@bref		AT+ICCID	指令解析
@param
@return
@note
**************************************************/

static void iccidParser(uint8_t *buf, uint16_t len)
{
    int16_t index, indexa;
    uint8_t *rebuf;
    uint16_t  relen;
    uint8_t snlen, i;
    char debug[70];
    index = my_getstrindex((char *)buf, "+ICCID:", len);
    if (index >= 0)
    {
        rebuf = buf + index;
        relen = len - index;
        indexa = getCharIndex(rebuf, relen, '\r');
        if (indexa > 8)
        {
            snlen = indexa - 8;
            if (snlen == 20)
            {
                for (i = 0; i < snlen; i++)
                {
                    moduleState.ICCID[i] = rebuf[i + 8];
                }
                moduleState.ICCID[snlen] = 0;
                sprintf(debug, "ICCID:%s", moduleState.ICCID);
                LogMessage(DEBUG_ALL, debug);
            }
        }
    }

}


/**************************************************
@bref		短信接收
@param
@return
@note
**************************************************/


static void cmtiParser(uint8_t *buf, uint16_t len)
{
    uint8_t i;
    int16_t index;
    uint8_t *rebuf;
    char restore[5];
    index = my_getstrindex((char *)buf, "+CMTI:", len);
    if (index >= 0)
    {
        rebuf = buf + index;
        index = getCharIndex(rebuf, len, ',');
        if (index < 0)
            return;
        rebuf = rebuf + index + 1;
        index = getCharIndex(rebuf, len, '\r');
        if (index > 5 || index < 0)
            return ;
        for (i = 0; i < index; i++)
        {
            restore[i] = rebuf[i];
        }
        restore[index] = 0;
        LogPrintf(DEBUG_ALL, "Message index=%d", atoi(restore));
        sendModuleCmd(CMGR_CMD, restore);
    }
}

/**************************************************
@bref		CMGR	指令解析
@param
@return
@note
**************************************************/

static void cmgrParser(uint8_t *buf, uint16_t len)
{
    int index;
    uint8_t *rebuf;
    uint8_t *numbuf;
    uint16_t  relen, i, renumlen;
    char restore[100];
    insParam_s insparam;
    //找到特定字符串在buf的位置
    index = my_getstrindex((char *)buf, "+CMGR:", len);
    if (index >= 0)
    {
        //得到特定字符串的开始位置和剩余长度
        rebuf = buf + index;
        relen = len - index;
        //识别手机号码
        index = getCharIndexWithNum(rebuf, relen, '"', 3);
        if (index < 0)
            return;
        numbuf = rebuf + index + 1;
        renumlen = relen - index - 1;
        index = getCharIndex(numbuf, renumlen, '"');
        if (index > 100 || index < 0)
            return ;
        for (i = 0; i < index; i++)
        {
            restore[i] = numbuf[i];
        }
        restore[index] = 0;

        if (index > sizeof(moduleState.messagePhone))
            return ;
        strcpy((char *)moduleState.messagePhone, restore);
        LogPrintf(DEBUG_ALL, "Tel:%s", moduleState.messagePhone);
        //得到第一个\n的位置
        index = getCharIndex(rebuf, len, '\n');
        if (index < 0)
            return;
        //偏移到内容处
        rebuf = rebuf + index + 1;
        //得到从内容处开始的第一个\n，测试index就是内容长度
//        index = getCharIndex(rebuf, len, '"');
//        if (index > 100 || index < 0)
//            return ;
        for (i = 0; i < index; i++)
        {
            restore[i] = rebuf[i];
        }
        restore[index] = 0;
        LogPrintf(DEBUG_ALL, "Message:%s", restore);
        insparam.telNum = moduleState.messagePhone;

        instructionParser((uint8_t *)restore, index, SMS_MODE, &insparam);
    }
}

/**************************************************
@bref		AT+QISEND	指令解析
@param
@return
@note
+QISEND: 212,212,0

**************************************************/

static void qisendParser(uint8_t *buf, uint16_t len)
{
    int index;
    uint8_t *rebuf;
    uint8_t i, datalen, cnt, sockId;
    uint16_t relen;
    char restore[51];

    index = my_getstrindex((char *)buf, "ERROR", len);
    if (index >= 0)
    {
        //不能很好区分到底是哪条链路出现错误
        socketResetConnState();
        moduleSleepCtl(0);
        changeProcess(CGREG_STATUS);
        return ;
    }
    index = my_getstrindex((char *)buf, "+QISEND:", len);
    if (index < 0)
    {
        return ;
    }

    rebuf = buf + index + 9;
    relen = len - index - 9;
    index = getCharIndex(rebuf, relen, '\n');
    datalen = 0;
    cnt = 0;
    if (index > 0 && index < 50)
    {
        restore[0] = 0;
        for (i = 0; i < index; i++)
        {
            if (rebuf[i] == ',' || rebuf[i] == '\r' || rebuf[i] == '\n')
            {
                if (restore[0] != 0)
                {
                    restore[datalen] = 0;
                    cnt++;
                    datalen = 0;
                    switch (cnt)
                    {
                        case 1:
                            moduleState.tcpTotal = atoi(restore);
                            break;
                        case 2:
                            moduleState.tcpAck = atoi(restore);
                            break;
                        case 3:
                            moduleState.tcpNack = atoi(restore);
                            break;
                    }
                    restore[0] = 0;
                }
            }
            else
            {
                restore[datalen] = rebuf[i];
                datalen++;
                if (datalen >= 50)
                {
                    return ;
                }

            }
        }
    }
}





/**************************************************
@bref		MWIFISCANINFO	指令解析
@param
@return
@note
+MWIFISCANINFO: 1,,"EC41180C8209",,-71,4,
+MWIFISCANINFO: 2,,"4022301AF801",,-79,1,
+MWIFISCANINFO: 3,,"F88C21A2C6E9",,-79,11,
+MWIFISCANINFO: 4,,"086BD10B5060",,-80,11,
+MWIFISCANINFO: 5,,"F46D2F7F0EA8",,-84,11,
+MWIFISCANINFO: 6,,"C4AD34C70D01",,-88,9,
+MWIFISCANINFO: 0


**************************************************/
void mwifiscaninfoParser(uint8_t *buf, uint16_t len)
{
	int index;
	uint8_t *rebuf, i;
	int16_t relen;
	char restore[20];
	uint8_t numb;
	WIFIINFO wifiList = { 0 };
	rebuf = buf;
	relen = len;
	index = my_getstrindex((char *)rebuf, "+MWIFISCANINFO:", relen);
	wifiList.apcount = 0;
	while (index >= 0)
	{
		rebuf += index + 16;
		relen -= index + 16;
		index = getCharIndex(rebuf, relen, ',');
		if (index < 0 || index > 2)
		{
			tmos_memcpy(restore, rebuf, 1);
			restore[1] = 0;
			numb = atoi(restore);
			if (numb == 0 && wifiList.apcount == 0 && sysinfo.wifiExtendEvt != 0)
			{
				wifiRspSuccess();
				sysinfo.wifiExtendEvt = 0;
				//lbsRequestSet(DEV_EXTEND_OF_MY);
			}
			break;
		}
		tmos_memcpy(restore, rebuf, index);
		restore[index] = 0;
		numb = atoi(restore);
		index = getCharIndex(rebuf, relen, '"');
		rebuf += index + 1;
		relen -= index + 1;
		index = getCharIndex(rebuf, relen, '"');
		if (numb != 0 && wifiList.apcount < WIFIINFOMAX)
		{
			memcpy(restore, rebuf, index);
			restore[index] = 0;
			LogPrintf(DEBUG_ALL, "WIFI(%d):[%s]", numb, restore);
			wifiList.ap[wifiList.apcount].signal = 0;
			/*	*/
			if (strncmp(restore, "000000000000", 12) == 0 || strncmp(restore, "FFFFFFFFFFFF", 12) == 0)
			{
				LogPrintf(DEBUG_ALL, "WIFI mac error:%s", restore);
			}
			else
			{
				changeHexStringToByteArray(wifiList.ap[wifiList.apcount].ssid, restore, 6);
				wifiList.apcount++;
			}
		}
		index = getCharIndex(rebuf, relen, '\r');
		rebuf += index;
		relen -= index;
		index = my_getstrindex((char *)rebuf, "+MWIFISCANINFO:", relen);
	}
	if (wifiList.apcount != 0)
	{
		//取消扫不到wifi上报lbs的逻辑
//		if (wifiList.apcount < 3 && sysinfo.wifiExtendEvt != 0)
//		{
//			lbsRequestSet(DEV_EXTEND_OF_MY);
//		}
//		else 
//		{
			if (sysinfo.wifiExtendEvt & DEV_EXTEND_OF_MY)
			{
				protocolSend(NORMAL_LINK, PROTOCOL_F3, &wifiList);
			}
			if (sysinfo.wifiExtendEvt & DEV_EXTEND_OF_BLE)
			{
				protocolSend(BLE_LINK, PROTOCOL_F3, &wifiList);
			}
//			lbsRequestClear();
//		}
		sysinfo.wifiExtendEvt = 0;
		wifiRspSuccess();
	}
}


//+CGSN:864606060177986
static void cgsnParser(uint8_t *buf, uint16_t len)
{
    int16_t index;
    uint8_t *rebuf;
    uint16_t  relen;
    uint8_t i;
    rebuf = buf;
    relen = len;
	index = my_getstrindex((char *)rebuf, "+CGSN:", relen);
	if (index < 0)
		return;
	rebuf += index + 7;
	relen -= index - 7;
	index = getCharIndex(rebuf, relen, '\r');
    if (index >= 0 && index <= 20)
    {
        for (i = 0; i < index; i++)
        {
            moduleState.IMEI[i] = rebuf[i];
        }
        moduleState.IMEI[index] = 0;
        LogPrintf(DEBUG_ALL, "module IMEI [%s]", moduleState.IMEI);
        if (tmos_memcmp(moduleState.IMEI, dynamicParam.SN, 15) == FALSE)
        {
            tmos_memset(dynamicParam.SN, 0, sizeof(dynamicParam.SN));
            strncpy(dynamicParam.SN, moduleState.IMEI, 15);
            jt808CreateSn(dynamicParam.jt808sn, dynamicParam.SN + 3, 12);
            dynamicParam.jt808isRegister = 0;
            dynamicParam.jt808AuthLen = 0;
            dynamicParamSaveAll();
        }
    }

}


/**************************************************
@bref		+CGATT	指令解析
@param
@return
@note
	+CGATT: 1

	OK
**************************************************/

static void cgattParser(uint8_t *buf, uint16_t len)
{
    uint8_t ret;
    int index;
    uint8_t *rebuf;
    int16_t relen;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+CGACT: ", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index;
    relen -= index;
    ret = rebuf[10] - '0';
    //LogPrintf(DEBUG_ALL, "ret:%d", ret);
    if (ret == 1)
    {
        moduleState.qipactOk = 1;
    }
    else
    {
        moduleState.qipactOk = 0;
    }
}

/**************************************************
@bref       +MIPCALL  指令解析
@param
@return
@note
+MIPCALL: 1,1,"10.165.173.87"

**************************************************/

static void mipcallParser(uint8_t *buf, uint16_t len)
{
    uint8_t ret;
    int index;
    uint8_t *rebuf;
    int16_t relen;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+MIPCALL:", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index;
    relen -= index;
    ret = rebuf[12] - '0';
    if (ret == 1)
    {
        moduleState.qipactOk = 1;
    }
    
    else 
    {
        moduleState.qipactOk = 0;
        changeProcess(CPIN_STATUS);
    }
}
/**************************************************
@bref		+RECEIVE	指令解析
@param
@return
@note
+RECEIVE: 0, 16
xx?		\0U?

**************************************************/

uint8_t receiveParser(uint8_t *buf, uint16_t len)
{
    char *rebuf;
    char resbuf[513];
    int index, relen, recvLen;
    int sockId, debugLen;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+RECEIVE:", relen);
    if (index < 0)
        return 0;
    while (index >= 0)
    {
        rebuf += index + 10;
        relen -= index + 10;
        index = getCharIndex(rebuf, relen, ',');
        if (index >= 1 && index <= 2)
        {
            memcpy(resbuf, rebuf, index);
            resbuf[index] = 0;
            sockId = atoi(resbuf);
            rebuf += index + 2;
            relen -= index + 2;
            index = getCharIndex(rebuf, relen, '\r');
            if (index >= 0 && index <= 5)
            {
                memcpy(resbuf, rebuf, index);
                resbuf[index] = 0;
                recvLen = atoi(resbuf);
                rebuf += index + 2;
                relen -= index + 2;
                if (relen >= recvLen)
                {
                    debugLen = recvLen > 256 ? 256 : recvLen;
                    byteToHexString(rebuf, resbuf, debugLen);
                    resbuf[debugLen * 2] = 0;
                    LogPrintf(DEBUG_ALL, "TCP RECV (%d)[%d]:%s", sockId, recvLen, resbuf);
                    socketRecv(sockId, rebuf, recvLen);
                    rebuf += recvLen;
                    relen -= recvLen;
                }
                else
                {
                    LogMessage(DEBUG_ALL, "TCP data lost");
                    return 1;
                }

            }
        }
        index = my_getstrindex((char *)rebuf, "+RECEIVE:", relen);
    }
    return 0;
}


/**************************************************
@bref		MADC	指令解析
@param
@return
@note
	+MADC: 1203

	OK
**************************************************/

void madcParser(uint8_t *buf, uint16_t len)
{
    char *rebuf;
    int index, relen;
    ITEM item;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+MADC:", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index + 7;
    relen -= index + 7;
    stringToItem(&item, rebuf, relen);
    if (item.item_cnt == 2)
    {
        sysinfo.insidevoltage = atoi(item.item_data[0]) / 100.0 * 2.03;
        LogPrintf(DEBUG_ALL, "batttery voltage %.2f", sysinfo.insidevoltage);
    }
}


/**************************************************
@bref		QIRD	指令解析
@param
@return
@note
+QIRD: 47.107.25.39:9998,TCP,10
xx馶0
?

OK

**************************************************/

static uint8_t qirdParser(uint8_t *buf, uint16_t len)
{
    char *rebuf;
    char resbuf[513];
    int index, relen, recvLen;
    int debugLen;
    uint8_t ret = 0;
    rebuf = buf;
    relen = len;
    index = my_getstrindex(rebuf, "+QIRD:", relen);
    if (index < 0)
    {
        return ret;
    }
    while (index >= 0)
    {
        rebuf += index + 7;
        relen -= index + 7;
        index = getCharIndex(rebuf, relen, '\r');
        if (index >= 0 && index <= 5)
        {
            memcpy(resbuf, rebuf, index);
            resbuf[index] = 0;
            recvLen = atoi(resbuf);
            rebuf += index + 2;
            relen -= index + 2;
            if (relen >= recvLen)
            {
                if (recvLen == 0)
                {
                    switch (moduleState.curQirdId)
                    {
                        case NORMAL_LINK:
                            moduleState.normalLinkQird = 0;
                            break;
                        case BLE_LINK:
                            moduleState.bleLinkQird = 0;
                            break;
                        case JT808_LINK:
                            moduleState.jt808LinkQird = 0;
                            break;
                        case HIDDEN_LINK:
                            moduleState.hideLinkQird = 0;
                            break;
                        case AGPS_LINK:
                            moduleState.agpsLinkQird = 0;
                            break;
                    }
                    LogPrintf(DEBUG_ALL, "Link[%d] recv Done", moduleState.curQirdId);
                }
                else
                {
                    debugLen = recvLen > 256 ? 256 : recvLen;
                    byteToHexString(rebuf, resbuf, debugLen);
                    resbuf[debugLen * 2] = 0;
                    LogPrintf(DEBUG_ALL, "TCP RECV (%d)[%d]:%s", moduleState.curQirdId, recvLen, resbuf);
                    ret = 0;
                    socketRecv(moduleState.curQirdId, rebuf, recvLen);
                }

                rebuf += recvLen;
                relen -= recvLen;
            }
            else
            {
                ret = 1;
                LogMessage(DEBUG_ALL, "TCP data lost");
            }

        }

        index = my_getstrindex((char *)rebuf, "+QIRD:", relen);
    }
    return ret;
}


/**************************************************
@bref		PDP	指令解析
@param
@return
@note
+PDP DEACT


**************************************************/
void deactParser(uint8_t *buf, uint16_t len)
{
    int index;
    index = my_getstrindex(buf, "+PDP DEACT", len);
    if (index < 0)
    {
        return;
    }
    socketDelAll();
    changeProcess(CPIN_STATUS);
}


/**************************************************
@bref		CMT	指令解析
@param
@return
@note
+CMT: "1064899195049",,"2022/09/05 17:46:53+32"
PARAM

**************************************************/

void cmtParser(uint8_t *buf, uint16_t len)
{
    uint8_t *rebuf;
    char restore[130];
    int relen;
    int index;
    insParam_s insparam;
    rebuf = buf;
    relen = len;
    index = my_getstrindex(rebuf, "+CMT:", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index;
    relen -= index;
    while (index >= 0)
    {
        rebuf += 7;
        relen -= 7;
        index = getCharIndex(rebuf, relen, '\"');
        if (index < 0 || index >= 20)
        {
            return;
        }
        memcpy(moduleState.messagePhone, rebuf, index);
        moduleState.messagePhone[index] = 0;
        LogPrintf(DEBUG_ALL, "TEL:%s", moduleState.messagePhone);
        index = getCharIndex(rebuf, relen, '\n');
        if (index < 0)
        {
            return;
        }
        rebuf += index + 1;
        relen -= index + 1;
        index = getCharIndex(rebuf, relen, '\r');
        if (index < 0 || index >= 128)
        {
            return ;
        }
        memcpy(restore, rebuf, index);
        restore[index] = 0;
        LogPrintf(DEBUG_ALL, "Content:[%s]", restore);
        insparam.telNum = moduleState.messagePhone;
        instructionParser((uint8_t *)restore, index, SMS_MODE, &insparam);
    }
}

void ringParser(uint8_t *buf, uint16_t len)
{
    int index;
    index = my_getstrindex(buf, "RING", len);
    if (index < 0)
    {
        return;
    }
    sendModuleCmd(ATA_CMD, NULL);
}

/**************************************************
@bref		MCCID	指令解析
@param
@return
@note
	+MCCID: 89860620040007789511

**************************************************/
static void mccidParser(uint8_t *buf, uint16_t len)
{
	int16_t index, indexa;
    uint8_t *rebuf;
    uint16_t  relen;
    uint8_t i;
    char debug[70];
    index = my_getstrindex((char *)buf, "+MCCID:", len);
    if (index >= 0)
    {
        rebuf = buf + index + 8;
        relen = len - index - 8;
        indexa = getCharIndex(rebuf, relen, '\r');
        if (indexa > 8)
        {
        	LogPrintf(DEBUG_ALL, "indexa:%d", indexa);
            if (indexa == 20)
            {
                for (i = 0; i < indexa; i++)
                {
                    moduleState.ICCID[i] = rebuf[i];
                }
                moduleState.ICCID[indexa] = 0;
                sprintf(debug, "ICCID:%s", moduleState.ICCID);
                LogMessage(DEBUG_ALL, debug);
            }
        }
    }

}

/**************************************************
@bref		MIPOPEN	指令解析
@param
@return
@note
	+MIPOPEN: 0,0
	+MIPOPEN: 0,571
**************************************************/

void mipopenParser(uint8_t *buf, uint16_t len)
{
	int index;
	int relen;
    uint8_t *rebuf;
    uint8_t result, link;
    ITEM item;
    
    rebuf = buf;
    relen = len;
	index = my_getstrindex(rebuf, "+MIPOPEN:", relen);
	while (index >= 0)
	{
		rebuf += index + 10;
		relen -= index + 10;
		index = getCharIndex(rebuf, relen, '\r');
		stringToItem(&item, rebuf, relen);
		link = atoi(item.item_data[0]);
		result = atoi(item.item_data[1]);
		if (result == 0)
		{
			socketSetConnState(link, SOCKET_CONN_SUCCESS);
			if (link == NORMAL_LINK)
			{
				moduleCtrl.qiopenCount = 0;
			}
		}
		else
		{
			socketSetConnState(link, SOCKET_CONN_ERR);
			if (link == NORMAL_LINK)
			{
				moduleCtrl.qiopenCount++;
				if (moduleCtrl.qiopenCount >= 4)
				{
					moduleReset();
					moduleCtrl.qiopenCount = 0;
				}
			}
			 
		}
		index = my_getstrindex(rebuf, "+MIPOPEN:", relen);
	}
	
}

uint8_t isAgpsDataRecvComplete(void)
{
	return moduleState.agpsLinkQird;
}

/**************************************************
@bref		模组端urc数据接收解析器
@param
@return
@note
+MIPURC: "rtcp",0,2
+MIPURC: "disconn",0,1


OK
**************************************************/

uint8_t mipurcParser(uint8_t *buf, uint16_t len)
{
    int index;
    char restore[513];
    uint8_t *rebuf, type, link, numb, ret = 0;
    int16_t relen;
    uint16_t readLen, unreadLen, debugLen;
    ITEM item;
    rebuf = buf;
    relen = len;

    index = my_getstrindex(rebuf, "+MIPURC:", relen);
    if (index < 0)
    	return 0 ;
    while (index >= 0)
    {
		rebuf += index + 10;
		relen -= index + 10;
        if (my_strpach(rebuf, "rtcp"))
        {
			rebuf += 6;
			relen -= 6;
			index = getCharIndex(rebuf, relen, '\r');
			tmos_memcpy(restore, rebuf, index);
			restore[index] = 0;
			stringToItem(&item, restore, index);
			link = atoi(item.item_data[0]);
			numb = atoi(item.item_data[1]);
			LogPrintf(DEBUG_ALL, "Socket[%d] recv data", link);
			switch (link)
			{
				case NORMAL_LINK:
					moduleState.normalLinkQird = 1;
					break;
				case BLE_LINK:
					moduleState.bleLinkQird = 1;
					break;
				case JT808_LINK:
					moduleState.jt808LinkQird = 1;
					break;
				case HIDDEN_LINK:
					moduleState.hideLinkQird = 1;
					break;
				case AGPS_LINK:
					moduleState.agpsLinkQird = 1;
					break;
			}

        }
        else if (my_strpach(rebuf, "disconn"))
        {
			rebuf += 9;
			relen -= 9;
			index = getCharIndex(rebuf, relen, '\r');
			tmos_memcpy(restore, rebuf, index);
			restore[index] = 0;
			stringToItem(&item, restore, index);
			link = atoi(item.item_data[0]);
			LogPrintf(DEBUG_ALL, "Socket[%d] close", link);
			socketSetConnState(link, SOCKET_CONN_IDLE);
			rebuf += index;
			relen -= index;
        }

        index = my_getstrindex(rebuf, "+MIPURC:", relen);
    }

    return ret;
}
/**************************************************
@bref		模组端数据接收解析器
@param
@return
@note
+MIPRD: 0,2,10,xx\0\0萓


+MIPRD: 0,1,10,xx馶0gs


+MIPRD: 0,0,16,xx?"\0

OK
**************************************************/

static void miprdParser(uint8_t *buf, uint16_t len)
{
	int index;
	uint8_t link, reindex;
	uint8_t *rebuf;
	uint16_t relen;
	uint8_t restore[512];
	uint16_t debuglen, rxlen;
	rebuf = buf;
	relen = len;
	index = my_getstrindex(rebuf, "+MIPRD:", relen);
	if (index < 0)
		return;
	while (index >= 0)
	{
		rebuf += index + 8;
		relen -= index + 8;
		index = getCharIndex(rebuf, relen, ',');
		if (index < 0)
		{
			LogMessage(DEBUG_ALL, "No follow-up data1");
			return ;
		}
		tmos_memcpy(restore, rebuf, index);
		restore[index] = 0;
		link = atoi(restore);
		rebuf += index + 1;
		relen -= index + 1;

		index = getCharIndex(rebuf, relen, ',');
		if (index < 0)
		{
			LogMessage(DEBUG_ALL, "No follow-up data2");
			return ;
		}
		tmos_memcpy(restore, rebuf, index);
		restore[index] = 0;
		reindex = atoi(restore);
		if (reindex == 0)
		{
			switch (link)
			{
				case NORMAL_LINK:
					moduleState.normalLinkQird = 0;
					break;
				case BLE_LINK:
					moduleState.bleLinkQird = 0;
					break;
				case JT808_LINK:
					moduleState.jt808LinkQird = 0;
					break;
				case HIDDEN_LINK:
					moduleState.hideLinkQird = 0;
					break;
				case AGPS_LINK:
					moduleState.agpsLinkQird = 0;
					break;
			}
			LogPrintf(DEBUG_ALL, "Socket[%d] recv Done", link);
		}
		rebuf += index + 1;
		relen -= index + 1;

		index = getCharIndex(rebuf, relen, ',');
		if (index < 0)
		{
			LogMessage(DEBUG_ALL, "No follow-up data3");
			return ;
		}
		tmos_memcpy(restore, rebuf, index);
		restore[index] = 0;
		rxlen = atoi(restore);
		rebuf += index + 1;
		relen -= index + 1;
		index = my_getstrindex(rebuf, "\r\n\r\n", relen);
		
		if (index >= 0)
			index +=2;
		else
			LogMessage(DEBUG_ALL, "No follow-up data4");
		debuglen = rxlen > 256 ? 256 : rxlen;
		tmos_memcpy(restore, rebuf, debuglen);
		restore[debuglen] = 0;
		LogPrintf(DEBUG_ALL, "TCP RECV (%d)[%d]:%s", link, rxlen, restore);
		socketRecv(link, rebuf, rxlen);
		rebuf += index;
		relen -= index;
		index = my_getstrindex(rebuf, "+MIPRD:", relen);
	}
}



/**************************************************
@bref		模组端数据接收解析器
@param
@return
@note
+MIPSACK: <sent>,<acked>,<nack>,<received>
+MIPSACK: 81,81,0,46

**************************************************/

void mipsackParser(uint8_t *buf, uint16_t len)
{
    int index;
    ITEM item;
    uint8_t *rebuf;
    int16_t relen;
    rebuf = buf;
    relen = len;
	static uint8_t cnt;

	index = my_getstrindex(rebuf, "ERROR", relen);
	if (index >= 0)
	{
		cnt++;
		if (cnt >= 5)
		{
			moduleReset();
			cnt = 0;
		}
	}
	
    
    index = my_getstrindex(rebuf, "+MIPSACK:", relen);
    if (index < 0)
    {
        return;
    }

    rebuf += 10;
    relen -= 10;

    if (relen < 0)
    {
        return;
    }
    cnt = 0;
    stringToItem(&item, rebuf, relen);
    moduleState.tcpTotal = atoi(item.item_data[0]);
    moduleState.tcpAck = atoi(item.item_data[1]);
    moduleState.tcpNack = atoi(item.item_data[2]);
    LogPrintf(DEBUG_ALL, "Total send:%d,Ack:%d,NAck:%d; Total recvive:%d", moduleState.tcpTotal, moduleState.tcpAck, moduleState.tcpNack, atoi(item.item_data[3]));

}

/**************************************************
@bref		短消息解析接收器
@param
@return
@note
+CMGL: 1,"REC READ","1064899192026",,"23/05/15,15:01:27+32"
STATUS

+CMGL: 3,"REC READ","1064899192026",,"23/05/15,15:01:51+32"
VERSION

+CMGL: 4,"REC READ","1064899192026",,"23/05/15,15:04:49+32"
SETBLEMAC

**************************************************/
static void cmglParser(uint8_t *buf, uint16_t len)
{
	int index;
	uint8_t *rebuf;
	uint16_t relen;
	uint8_t smsInd;
	uint8_t restore[50];
	ITEM item;
	
	rebuf = buf;
	relen = len;
	index = my_getstrindex(rebuf, "+CMGL:", relen);
	if (index < 0)
		return;

	while (index >= 0)
	{
		rebuf += index + 7;
		relen -= index + 7;
		index = getCharIndex(rebuf, relen, '\r');
		if (index > 0)
		{
			stringToItem(&item, rebuf, relen);
			smsInd = atoi(item.item_data[0]);
			tmos_memcpy(restore, item.item_data[1], index);
			if (tmos_memcmp(restore, "\"REC READ\"", strlen(restore)) == TRUE)
			{
				LogPrintf(DEBUG_ALL, "SMS[%d] has read, delete it", smsInd);
				deleteMessage(smsInd);
			}

		}
		
		
		index = my_getstrindex(rebuf, "+CMGL:", relen);
	}
	
	
}

/**************************************************
@bref		模组异常复位检测
@param
@return
@note
**************************************************/
static void moduleRstDetector(uint8_t * buf, uint16_t len)
{
	int index;
	if (moduleState.powerState != 1)
	{
		return;
	}

	index = my_getstrindex((char *)buf, "+MATREADY", len);
	if (index >= 0)
	{
		if (sysinfo.moduleRstFlag == 1)
		{
			sysinfo.moduleRstFlag = 0;
			LogMessage(DEBUG_ALL, "ignore module abnormal reset");
			return;
		}


	}
}


/**************************************************
@bref		模组端数据接收解析器
@param
@return
@note
**************************************************/

void moduleRecvParser(uint8_t *buf, uint16_t bufsize)
{
    static uint16_t len = 0;
    static uint8_t dataRestore[MODULE_RX_BUFF_SIZE + 1];
    
    if (bufsize == 0)
        return;
    if (len + bufsize > MODULE_RX_BUFF_SIZE)
    {
        len = 0;
        bufsize %= MODULE_RX_BUFF_SIZE;
        LogMessage(DEBUG_ALL, "UartRecv Full!!!");
    }
    memcpy(dataRestore + len, buf, bufsize);
    len += bufsize;
    dataRestore[len] = 0;
//        uint8_t debug[1000];
//        byteToHexString(dataRestore, debug, len);
//        debug[len * 2]=0;
//        LogMessage(DEBUG_ALL, "<<<<<<<<<");
//        LogMessageWL(DEBUG_ALL, (char *)debug, len * 2);
    if (dataRestore[len - 1] != '\n')
    {
        if (dataRestore[2] != '>')
        {
            return;
        }
    }
    LogPrintf(DEBUG_ALL, "--->>>---0x%X [%d]", dataRestore, len);
    LogMessageWL(DEBUG_ALL, (char *)dataRestore, len);
    LogMessage(DEBUG_ALL, "---<<<---");
//        uint8_t debug[1000];
//        byteToHexString(dataRestore, debug, len);
//        debug[len * 2]=0;
//        LogMessage(DEBUG_ALL, "<<<<<<<<<");
//        LogMessageWL(DEBUG_ALL, (char *)debug, len * 2);

    /*****************************************/

    moduleRstDetector(dataRestore, len);
    moduleRspSuccess();
    cmtiParser(dataRestore, len);
    cmgrParser(dataRestore, len);
    mipopenParser(dataRestore, len);
    mipurcParser(dataRestore, len);
    miprdParser(dataRestore, len);
    cmglParser(dataRestore, len);
    mwifiscaninfoParser(dataRestore, len);
    mipcallParser(dataRestore, len);
	cmtParser(dataRestore, len);

    /*****************************************/
    switch (moduleState.cmd)
    {
        case AT_CMD:
            if (distinguishOK((char *)dataRestore))
            {
                moduleState.atResponOK = 1;
            }
            break;
        case CPIN_CMD:
            if (my_strstr((char *)dataRestore, "+CPIN: READY", len))
            {
                moduleState.cpinResponOk = 1;
            }
            break;
        case CSQ_CMD:
            csqParser(dataRestore, len);
            break;
        case CGREG_CMD:
        case CEREG_CMD:
            cgregParser(dataRestore, len);
            break;
        case CIMI_CMD:
            cimiParser(dataRestore, len);
            break;
        case MCCID_CMD:
            mccidParser(dataRestore, len);
            break;
        case CGSN_CMD:
            cgsnParser(dataRestore, len);
            break;
        case MADC_CMD:
            madcParser(dataRestore, len);
            break;
        case MIPSACK_CMD:
            mipsackParser(dataRestore, len);
            break;
//        case MIPCALL_CMD:
//            mipcallParser(dataRestore, len);
//            break;
        case MIPRD_CMD:
        	if (my_strstr((char *)dataRestore, "+CME ERROR:", len))
            {
                switch (moduleState.curQirdId)
                {
                    case NORMAL_LINK:
                        moduleState.normalLinkQird = 0;
                        break;
                    case BLE_LINK:
                        moduleState.bleLinkQird = 0;
                        break;
                    case JT808_LINK:
                        moduleState.jt808LinkQird = 0;
                        break;
                    case HIDDEN_LINK:
                        moduleState.hideLinkQird = 0;
                        break;
                    case AGPS_LINK:
                        moduleState.agpsLinkQird = 0;
                        break;
                }
                LogPrintf(DEBUG_ALL, "Link[%d] recv err", moduleState.curQirdId);
            }
        	break;
        default:
            break;
    }
    moduleState.cmd = 0;
    len = 0;
}

/*--------------------------------------------------------------*/

/**************************************************
@bref		重置信号搜索时长
@param
@return
@note
**************************************************/

void netResetCsqSearch(void)
{
    moduleCtrl.csqTime = 90;
}

/**************************************************
@bref		socket发送数据
@param
@return
@note
**************************************************/

int socketSendData(uint8_t link, uint8_t *data, uint16_t len)
{
    int ret = 0;
    char param[10];

    if (socketGetConnStatus(link) == 0)
    {
        //链路未链接
        return 0;
    }

    sprintf(param, "%d,%d", link, len);
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(MIPSEND_CMD, param);
    createNode((char *)data, len, CIPSEND_CMD);
    if (link == NORMAL_LINK || link == JT808_LINK)
    {
        moduleState.tcpNack = len;
    }
    return len;
}
/**************************************************
@bref		模组睡眠控制
@param
@return
@note
**************************************************/
void moduleSleepCtl(uint8_t onoff)
{
    char param[5];
    if (onoff == 0)
    {
        return;
    }
    else
    {
	    sprintf(param, "\"sleepmode\",2,0", onoff);
	    sendModuleCmd(MLPMCFG_CMD, param);
    }
}

/**************************************************
@bref		获取CSQ
@param
@return
@note
**************************************************/

void moduleGetCsq(void)
{
    sendModuleCmd(CSQ_CMD, NULL);
}

/**************************************************
@bref		获取基站
@param
@return
@note
**************************************************/

void moduleGetLbs(void)
{
	sendModuleCmd(CEREG_CMD, "2");
    sendModuleCmd(CEREG_CMD, "?");
}
/**************************************************
@bref		获取WIFIscan
@param
@return
@note
**************************************************/

void moduleGetWifiScan(void)
{
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(MWIFISCANSTART_CMD, NULL);
}

/**************************************************
@bref		停止WIFIscan
@param
@return
@note
**************************************************/

void moduleStopWifiScan(void)
{
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(MWIFISCANSTOP_CMD, NULL);
}

/**************************************************
@bref		发送短消息
@param
@return
@note
**************************************************/

void sendMessage(uint8_t *buf, uint16_t len, char *telnum)
{
    char param[60];
    sprintf(param, "\"%s\"", telnum);
    sendModuleCmd(CMGF_CMD, "1");
    sendModuleCmd(CMGS_CMD, param);
    LogPrintf(DEBUG_ALL, "len:%d", len);
    buf[len] = 0x1A;
    createNode((char *)buf, len + 1, CMGS_CMD);
}

/**************************************************
@bref       删除所有短消息
@param
@return
@note
**************************************************/
void deleteAllMessage(void)
{
    sendModuleCmd(CMGD_CMD, "0,4");
}

/**************************************************
@bref		删除某条短消息
@param
@return
@note
**************************************************/


void deleteMessage(uint8_t index)
{
	char param[50];
	sprintf(param, "%d", index);
    sendModuleCmd(CMGD_CMD, param);
}

/**************************************************
@bref		查询短信列表
@param
@return
@note
**************************************************/

void queryMessageList(void)
{
	char param[50];
	sprintf(param, "\"ALL\"");
	sendModuleCmd(CMGL_CMD, param);
}

/**************************************************
@bref		查询数据是否发送完毕
@param
@return
@note
**************************************************/

void querySendData(uint8_t link)
{
    char param[5];
    sprintf(param, "%d", link);
    sendModuleCmd(MIPSACK_CMD, param);
}


/**************************************************
@bref		查询模组电池电压
@param
@return
@note
**************************************************/

void queryBatVoltage(void)
{
    //sendModuleCmd(MADC_CMD, "0");
}

/**************************************************
@bref		查询模组温度
@param
@return
@note
**************************************************/

void queryTemperture(void)
{
	sendModuleCmd(MCHIPINFO_CMD, "\"temp\"");
}

/**************************************************
@bref		读取信号值
@param
@return
@note
**************************************************/

uint8_t getModuleRssi(void)
{
    return moduleState.rssi;
}

/**************************************************
@bref		读取IMSI
@param
@return
@note
**************************************************/

uint8_t *getModuleIMSI(void)
{
    return moduleState.IMSI;
}
/**************************************************
@bref		读取IMEI
@param
@return
@note
**************************************************/

uint8_t *getModuleIMEI(void)
{
    return moduleState.IMEI;
}



/**************************************************
@bref		读取ICCID
@param
@return
@note
**************************************************/

uint8_t *getModuleICCID(void)
{
    return moduleState.ICCID;
}

/**************************************************
@bref		读取MCC
@param
@return
@note
**************************************************/

uint16_t getMCC(void)
{
    return moduleState.mcc;
}

/**************************************************
@bref		读取MNC
@param
@return
@note
**************************************************/

uint8_t getMNC(void)
{
    return moduleState.mnc;
}

/**************************************************
@bref		读取LAC
@param
@return
@note
**************************************************/

uint16_t getLac(void)
{
    return moduleState.lac;
}

/**************************************************
@bref		读取CID
@param
@return
@note
**************************************************/

uint32_t getCid(void)
{
    return moduleState.cid;
}

/**************************************************
@bref		读取未发送字节数，判断是否发送成功
@param
@return
@note
**************************************************/

uint32_t getTcpNack(void)
{
    return moduleState.tcpNack;
}

/**************************************************
@bref       查询模组版本
@param
@return
@note
**************************************************/

char *getQgmr(void)
{
//    return moduleState.qgmr;
}


/**************************************************
@bref		切换模式4回调函数
@param
@return
@note
**************************************************/
void changeMode4Callback(void)
{
	changeProcess(CPIN_STATUS);
}

/**************************************************
@bref		模式4模组是否搜网完成
@param
@return
@note
**************************************************/
uint8_t isModuleOfflineStatus(void)
{
	if (moduleState.fsmState == OFFLINE_STATUS)
		return 1;
	return 0;
}


/**************************************************
@bref		模组是否达到联网状态
@param
@return
@note
**************************************************/

uint8_t isModuleRunNormal(void)
{
    if (moduleState.fsmState == NORMAL_STATUS)
        return 1;
    return 0;
}

/**************************************************
@bref		模组达到正常开机
@param
@return
@note
**************************************************/

uint8_t isModulePowerOnOk(void)
{
    if (moduleState.powerOnTick > 10)
        return 1;
    return 0;
}
/**************************************************
@bref		模组是否关机
@param
@return
@note
**************************************************/
uint8_t isModulePowerOff(void)
{
	if (moduleState.powerState == 0)
		return 1;
	return 0;
}


/**************************************************
@bref		挂断电话
@param
@return
@note
**************************************************/

void stopCall(void)
{
    sendModuleDirect("ATH\r\n");
}
/**************************************************
@bref		拨打电话
@param
@return
@note
**************************************************/

void callPhone(char *tel)
{
    char param[50];
    sprintf(param, "ATD%s;\r\n", tel);
    LogPrintf(DEBUG_ALL, "Call %s", tel);
    sendModuleDirect(param);

}


