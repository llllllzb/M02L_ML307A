#include "app_port.h"

#include "CH583SFR.h"
#include "SLEEP.h"

#include "app_kernal.h"
#include "app_mir3da.h"
#include "app_param.h"
#include "app_sys.h"
#include "app_task.h"
#include "app_db.h"
#include "app_net.h"
//IIC
#define SCL_OUTHIGH    GPIOB_ModeCfg(SCL_PIN,GPIO_ModeIN_Floating)//输出高电平时，设置外输入，由外部上拉为高电平
#define SCL_OUTLOW     {GPIOB_ModeCfg(SCL_PIN,GPIO_ModeOut_PP_5mA);GPIOB_ResetBits(SCL_PIN);}//设为输出模式，并输出低电平
#define SCL_READ       sclReadInput()//设为输入模式，并读电平

#define SDA_OUTHIGH    GPIOB_ModeCfg(SDA_PIN,GPIO_ModeIN_Floating)
#define SDA_OUTLOW     {GPIOB_ModeCfg(SDA_PIN,GPIO_ModeOut_PP_5mA);GPIOB_ResetBits(SDA_PIN);}
#define SDA_READ       sdaReadInput()
/*--------------------------------------------------------------*/
/*
 * UART
 */
uint8_t uart0RxBuff[APPUSART0_BUFF_SIZE];
uint8_t uart0trigB;

uint8_t uart1RxBuff[APPUSART1_BUFF_SIZE];
uint8_t uart1trigB;

uint8_t uart2RxBuff[APPUSART2_BUFF_SIZE];
uint8_t uart2trigB;

uint8_t uart3RxBuff[APPUSART3_BUFF_SIZE];
uint8_t uart3trigB;

UART_RXTX_CTL usart0_ctl = {0};
UART_RXTX_CTL usart1_ctl = {0};
UART_RXTX_CTL usart2_ctl = {0};
UART_RXTX_CTL usart3_ctl = {0};



/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/**
 * @brief               将接收的串口数据塞入对应的串口缓冲区
 * @param
 *      uartctl         串口操作符
 *      buf             已接收数据
 *      len             数据长度
 * @return
 *      -1              缓冲区满
 *      0               成功
 */
static int8_t pushUartRxData(UART_RXTX_CTL *uartctl, uint8_t *buf, uint16_t len)
{
    uint16_t spare, sparetoend;
    uint8_t id = 5;
    if (uartctl->rxend > uartctl->rxbegin)
    {
        spare = uartctl->rxbufsize - (uartctl->rxend - uartctl->rxbegin);
    }
    else if (uartctl->rxbegin > uartctl->rxend)
    {
        spare = uartctl->rxbegin - uartctl->rxend;
    }
    else
    {
        spare = uartctl->rxbufsize;
    }
    if (len > spare - 1)
    {
    	if (uartctl == &usart0_ctl)
    	{
    		id = 0;
    	}
    	else if (uartctl == &usart1_ctl)
    	{
    		id = 1;
    	}
    	else if (uartctl == &usart2_ctl)
    	{
			id = 2;
    	}
    	else if (uartctl == &usart3_ctl)
    	{
			id = 3;
    	}
        LogPrintf(DEBUG_ALL, "pushUartRxData[%d]==>no space", id);
        return -1;
    }

    if (uartctl->rxend >= uartctl->rxbegin)
    {
        sparetoend = uartctl->rxbufsize - uartctl->rxend;
        if (sparetoend > len)
        {
            memcpy(uartctl->rxbuf + uartctl->rxend, buf, len);
            uartctl->rxend = (uartctl->rxend + len) % uartctl->rxbufsize;
        }
        else
        {
            memcpy(uartctl->rxbuf + uartctl->rxend, buf, sparetoend);
            uartctl->rxend = (uartctl->rxend + sparetoend) % uartctl->rxbufsize;
            memcpy(uartctl->rxbuf + uartctl->rxend, buf + sparetoend,
                   len - sparetoend);
            uartctl->rxend += (len - sparetoend);
        }
    }
    else
    {
        memcpy(uartctl->rxbuf + uartctl->rxend, buf, len);
        uartctl->rxend += len;
    }
    return 0;
}

/**
 * @brief               查找串口接收队列中是否有数据
 * @param
 *      uartctl         串口操作符
 * @return  none
 */
static void postUartRxData(UART_RXTX_CTL *uartctl)
{
    uint16_t end, rxsize;
    if (uartctl->rxbegin == uartctl->rxend)
    {
        return;
    }
    end = uartctl->rxend;
    if (end > uartctl->rxbegin)
    {
        rxsize = end - uartctl->rxbegin;
        if (uartctl->rxhandlefun != NULL)
        {
            uartctl->rxhandlefun(uartctl->rxbuf + uartctl->rxbegin, rxsize);
        }
        uartctl->rxbegin = end;
    }
    else
    {
        rxsize = uartctl->rxbufsize - uartctl->rxbegin;
        if (uartctl->rxhandlefun != NULL)
        {
            uartctl->rxhandlefun(uartctl->rxbuf + uartctl->rxbegin, rxsize);
        }
        uartctl->rxbegin = 0;
        if (uartctl->rxhandlefun != NULL)
        {
            uartctl->rxhandlefun(uartctl->rxbuf + uartctl->rxbegin, end);
        }
        uartctl->rxbegin = end;
    }
}
/**
 * @brief               轮询串口操作符，处理接收数据
 * @param
 * @return  none
 */
void pollUartData(void)
{
    postUartRxData(&usart0_ctl);
    postUartRxData(&usart1_ctl);
    postUartRxData(&usart2_ctl);
    postUartRxData(&usart3_ctl);
}
/**
 * @brief               串口初始化
 * @param
 *      type            串口号，见UARTTYPE
 *      onoff           开关
 *      baudrate        波特率
 *      rxhandlefun     接收回调
 * @note
 *      UART0_TX:PB7     UART0_RX:PB4
 *      UART1_TX:PB13    UART1_RX:PB12
 *      UART2_TX:PB23    UART2_RX:PB22
 *      UART3_TX:PA5     UART3_RX:PA4
 * @return  none
 */
void portUartCfg(UARTTYPE type, uint8_t onoff, uint32_t baudrate,
                 void (*rxhandlefun)(uint8_t *, uint16_t len))
{

    if (onoff == 0)
    {
        LogPrintf(DEBUG_ALL, "Close uart%d", type);
    }

    switch (type)
    {
        case APPUSART0:
            memset(&usart0_ctl, 0, sizeof(UART_RXTX_CTL));
            usart0_ctl.rxbuf = uart0RxBuff;
            usart0_ctl.rxbufsize = APPUSART0_BUFF_SIZE;
            usart0_ctl.rxhandlefun = rxhandlefun;
            usart0_ctl.txhandlefun = UART0_SendString;

            if (onoff)
            {
                usart0_ctl.init = 1;
                GPIOB_SetBits(GPIO_Pin_7);
                GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PU);
                GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
                UART0_DefInit();
                UART0_BaudRateCfg(baudrate);
                if (rxhandlefun != NULL)
                {
                    UART0_ByteTrigCfg(UART_7BYTE_TRIG);
                    uart0trigB = 7;
                    UART0_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                    PFIC_EnableIRQ(UART0_IRQn);
                }
            }
            else
            {
                usart0_ctl.init = 0;
                UART0_Reset();
                GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PD);
                GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeIN_PD);
                UART0_INTCfg(DISABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                PFIC_DisableIRQ(UART0_IRQn);
            }
            break;
        case APPUSART1:

            usart1_ctl.rxbegin = 0;
            usart1_ctl.rxend = 0;
            usart1_ctl.rxbuf = uart1RxBuff;
            usart1_ctl.rxbufsize = APPUSART1_BUFF_SIZE;
            usart1_ctl.rxhandlefun = rxhandlefun;
            usart1_ctl.txhandlefun = UART1_SendString;

            if (onoff)
            {
                usart1_ctl.init = 1;
                GPIOB_SetBits(GPIO_Pin_13); //Tx default high
                GPIOB_ModeCfg(GPIO_Pin_12, GPIO_ModeIN_PU);  //Rx default input high
                GPIOB_ModeCfg(GPIO_Pin_13, GPIO_ModeOut_PP_5mA);  // Set tx as output

                GPIOPinRemap(ENABLE, RB_PIN_UART1);

                UART1_DefInit(); //
                UART1_BaudRateCfg(baudrate);
                if (rxhandlefun != NULL)
                {

                    UART1_ByteTrigCfg(UART_7BYTE_TRIG);
                    uart1trigB = 7;
                    UART1_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                    PFIC_EnableIRQ(UART1_IRQn);
                }
            }
            else
            {
                usart1_ctl.init = 0;
                UART1_Reset();
                GPIOB_ModeCfg(GPIO_Pin_12, GPIO_ModeIN_PD);
                GPIOB_ModeCfg(GPIO_Pin_13, GPIO_ModeIN_PD);
                UART1_INTCfg(DISABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                PFIC_DisableIRQ(UART1_IRQn);
            }
            break;
        case APPUSART2:

            usart2_ctl.rxbegin = 0;
            usart2_ctl.rxend = 0;
            usart2_ctl.rxbuf = uart2RxBuff;
            usart2_ctl.rxbufsize = APPUSART2_BUFF_SIZE;
            usart2_ctl.rxhandlefun = rxhandlefun;
            usart2_ctl.txhandlefun = UART2_SendString;

            if (onoff)
            {
                usart2_ctl.init = 1;
                GPIOPinRemap(ENABLE, RB_PIN_UART2);
                GPIOB_SetBits(GPIO_Pin_23); //Tx default high
                GPIOB_ModeCfg(GPIO_Pin_22, GPIO_ModeIN_PU);  //Rx default input high
                GPIOB_ModeCfg(GPIO_Pin_23, GPIO_ModeOut_PP_5mA);  // Set tx as output
                UART2_DefInit(); //
                UART2_BaudRateCfg(baudrate);
                if (rxhandlefun != NULL)
                {
                    UART2_ByteTrigCfg(UART_7BYTE_TRIG);
                    uart2trigB = 7;
                    UART2_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                    PFIC_EnableIRQ(UART2_IRQn);
                }
            }
            else
            {
                usart2_ctl.init = 0;
                UART2_Reset();
                GPIOB_ModeCfg(GPIO_Pin_22, GPIO_ModeIN_PD);
                GPIOB_ModeCfg(GPIO_Pin_23, GPIO_ModeIN_PD);
                UART2_INTCfg(DISABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                PFIC_DisableIRQ(UART2_IRQn);
            }
            break;
        case APPUSART3:

            usart3_ctl.rxbegin = 0;
            usart3_ctl.rxend = 0;
            usart3_ctl.rxbuf = uart3RxBuff;
            usart3_ctl.rxbufsize = APPUSART3_BUFF_SIZE;
            usart3_ctl.rxhandlefun = rxhandlefun;
            usart3_ctl.txhandlefun = UART3_SendString;

            if (onoff)
            {
                usart3_ctl.init = 1;
                GPIOA_SetBits(GPIO_Pin_5); //Tx default high
                GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PU);  //Rx default input high
                GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeOut_PP_5mA);  // Set tx as output
                UART3_DefInit(); //
                UART3_BaudRateCfg(baudrate);
                if (rxhandlefun != NULL)
                {
                    UART3_ByteTrigCfg(UART_7BYTE_TRIG);
                    uart3trigB = 7;
                    UART3_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                    PFIC_EnableIRQ(UART3_IRQn);
                }
            }
            else
            {
                usart3_ctl.init = 0;
                UART3_Reset();
                GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PD);
                GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeIN_PD);
                UART3_INTCfg(DISABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                PFIC_DisableIRQ(UART3_IRQn);
            }
            break;
        default:
            break;
    }
    if (onoff)
    {
        LogPrintf(DEBUG_ALL, "Open uart%d==>%d", type, baudrate);
    }

}
/**
 * @brief               串口发送数据
 * @param
 *      uartctl         串口操作符
 *      buf             待发送数据
 *      len             数据长度
 * @return  none
 */
void portUartSend(UART_RXTX_CTL *uartctl, uint8_t *buf, uint16_t len)
{
    if (uartctl->txhandlefun != NULL && uartctl->init == 1)
    {
        uartctl->txhandlefun(buf, len);
    }
}
/*
 * UART0 中断接收
 * */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void UART0_IRQHandler(void)
{
    UINT8V i;
    UINT8V linestate;
    uint8_t rxbuff[7];
    switch (UART0_GetITFlag())
    {
        case UART_II_LINE_STAT:        // 线路状态错误
        {
            linestate = UART0_GetLinSTA();
            (void) linestate;
            break;
        }

        case UART_II_RECV_RDY:          // 数据达到设置触发点
            for (i = 0; i != uart0trigB; i++)
            {
                rxbuff[i] = UART0_RecvByte();
            }
            pushUartRxData(&usart0_ctl, rxbuff, uart0trigB);
            break;

        case UART_II_RECV_TOUT:         // 接收超时，暂时一帧数据接收完成
            i = UART0_RecvString(rxbuff);
            pushUartRxData(&usart0_ctl, rxbuff, i);
            break;

        case UART_II_THR_EMPTY:         // 发送缓存区空，可继续发送
            break;

        case UART_II_MODEM_CHG:         // 只支持串口0
            break;

        default:
            break;
    }
}
/*
 * UART1 中断接收
 * */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void UART1_IRQHandler(void)
{
    UINT8V linestate;
    UINT8V i;
    uint8_t rxbuff[7];
    switch (UART1_GetITFlag())
    {
        case UART_II_LINE_STAT:        // 线路状态错误
        {
            linestate = UART1_GetLinSTA();
            (void) linestate;
            break;
        }

        case UART_II_RECV_RDY:          // 数据达到设置触发点
            for (i = 0; i != uart1trigB; i++)
            {
                rxbuff[i] = UART1_RecvByte();
            }
            pushUartRxData(&usart1_ctl, rxbuff, uart1trigB);
            break;

        case UART_II_RECV_TOUT:         // 接收超时，暂时一帧数据接收完成
            i = UART1_RecvString(rxbuff);
            pushUartRxData(&usart1_ctl, rxbuff, i);
            break;

        case UART_II_THR_EMPTY:         // 发送缓存区空，可继续发送
            break;

        case UART_II_MODEM_CHG:         // 只支持串口0
            break;

        default:
            break;
    }
}
/*
 * UART2 中断接收
 * */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void UART2_IRQHandler(void)
{
    UINT8V i;
    UINT8V linestate;
    uint8_t rxbuff[7];
    switch (UART2_GetITFlag())
    {
        case UART_II_LINE_STAT:        // 线路状态错误
        {
            linestate = UART2_GetLinSTA();
            (void) linestate;
            break;
        }

        case UART_II_RECV_RDY:          // 数据达到设置触发点
            for (i = 0; i != uart2trigB; i++)
            {
                rxbuff[i] = UART2_RecvByte();
            }
            pushUartRxData(&usart2_ctl, rxbuff, uart2trigB);
            break;

        case UART_II_RECV_TOUT:         // 接收超时，暂时一帧数据接收完成
            i = UART2_RecvString(rxbuff);
            pushUartRxData(&usart2_ctl, rxbuff, i);
            break;

        case UART_II_THR_EMPTY:         // 发送缓存区空，可继续发送
            break;

        case UART_II_MODEM_CHG:         // 只支持串口0
            break;

        default:
            break;
    }
}
/*
 * UART3 中断接收
 * */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void UART3_IRQHandler(void)
{
    UINT8V i;
    UINT8V linestate;
    uint8_t rxbuff[7];
    switch (UART3_GetITFlag())
    {
        case UART_II_LINE_STAT:
        {
            linestate = UART3_GetLinSTA();
            (void) linestate;
            break;
        }

        case UART_II_RECV_RDY:
            for (i = 0; i != uart3trigB; i++)
            {
                rxbuff[i] = UART3_RecvByte();
            }
            pushUartRxData(&usart3_ctl, rxbuff, uart3trigB);
            break;

        case UART_II_RECV_TOUT:
            i = UART3_RecvString(rxbuff);
            pushUartRxData(&usart3_ctl, rxbuff, i);
            break;

        case UART_II_THR_EMPTY:
            break;

        case UART_II_MODEM_CHG:
            break;

        default:
            break;
    }
}


__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void GPIOA_IRQHandler(void)
{
    uint16_t iqr;
    iqr = GPIOA_ReadITFlagPort();
    if (iqr & RING_PIN)
    {
        wakeUpByInt(0, 8);
        GPIOA_ClearITFlagBit(RING_PIN);
    }


}

__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void GPIOB_IRQHandler(void)
{
    uint16_t iqr;
    iqr = GPIOB_ReadITFlagPort();
    if (iqr & GSINT_PIN)
    {
        motionInfo.tapInterrupt++;
        sysinfo.doMotionFlag = 1;
        GPIOB_ClearITFlagBit(GSINT_PIN);
       	if (GPIOB_ReadPortPin(GSINT_PIN)) 
		{
        	GPIOB_ResetBits(GSINT_PIN);
	    }
	    else
	    {
	        GPIOB_SetBits(GSINT_PIN);
	    }
    }

}

/**
 * @brief   gpio 中断唤醒
 * @param
 * @return	唤醒+触发报警逻辑
 */
__attribute__((section(".highcode")))
void portGpioWakeupIRQHandler(void)
{
	/*LDR触发*/
	if (sysparam.ldrEn)
	{
		if (sysinfo.ldrIrqFlag)
		{
			/*开启任务*/
			if (sysinfo.kernalRun == 0)
			{
				wakeUpByInt(2, 3);
				tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
			}
		}
	}
	if (sysinfo.doMotionFlag)
	{
//		portDebugUartCfg(1);
//		LogMessage(DEBUG_ALL, "devic moving..");

		/*开启任务*/
		if (sysinfo.kernalRun == 0)
		{
			wakeUpByInt(2, 3);
			tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
		}
		sysinfo.doMotionFlag = 0;
		
	}
}




/**
 * @brief   模组GPIO初始化
 * @param
 * @return
 */
void portModuleGpioCfg(uint8_t state)
{
    if (state)
    {
        GPIOA_ModeCfg(POWER_PIN, GPIO_ModeOut_PP_5mA);
		GPIOA_ModeCfg(RST_PIN, GPIO_ModeOut_PP_5mA);
        GPIOA_ModeCfg(DTR_PIN, GPIO_ModeOut_PP_5mA);
        GPIOA_ModeCfg(RING_PIN, GPIO_ModeIN_Floating);
        GPIOA_ITModeCfg(RING_PIN, GPIO_ITMode_FallEdge);
        PFIC_EnableIRQ(GPIO_A_IRQn);
        PORT_PWRKEY_H;
        PORT_RSTKEY_H;
        PORT_SUPPLY_OFF;
    }
    else
    {
        R16_PA_INT_EN &= ~RING_PIN;
        GPIOA_ModeCfg(DTR_PIN, GPIO_ModeIN_PD);
        GPIOA_ModeCfg(POWER_PIN, GPIO_ModeIN_PD);
		GPIOA_ModeCfg(RST_PIN, GPIO_ModeIN_PD);
        GPIOA_ModeCfg(RING_PIN, GPIO_ModeIN_PD);
    }
}

/**
 * @brief   LED GPIO初始化
 * @param
 * @return
 */
void portLedGpioCfg(uint8_t onoff)
{
	if (onoff)
	{
	    GPIOB_ModeCfg(LED1_PIN, GPIO_ModeOut_PP_5mA);
	    LED1_OFF;
	    LED2_OFF;
    }
    else
    {
		GPIOB_ModeCfg(LED1_PIN, GPIO_ModeIN_PD);
    }
}

/**
 * @brief   GPS GPIO初始化
 * @param
 * @return
 */
void portGpsGpioCfg(uint8_t onoff)
{

}

/**
 * @brief   MIC GPIO初始化
 * @param
 * @return
 */
void portMicGpioCfg(void)
{
//	GPIOB_ModeCfg(MICPWR_PIN, GPIO_ModeOut_PP_5mA);
//	MICPWR_OFF;
}

/**
 * @brief   LDR GPIO初始化
 * @param
 * @return
 */
void portLdrGpioCfg(uint8_t onoff)
{

}



/**
 * @brief   GPIO 重置
 * @param
 * @return
 */
void portGpioSetDefCfg(void)
{
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PD);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PD);
}

/**
 * @brief   系统复位
 * @param
 * @return
 */
void portSysReset(void)
{
    LogMessage(DEBUG_ALL, "system reset");
    dbSaveRelease();
    saveGpsHistory();
    SYS_ResetExecute();
}
/**
 * @brief   gsensor 电源控制
 * @param
 *      onoff
 * @return
 */
void portGsensorPwrCtl(uint8 onoff)
{
    if (onoff)
    {
        GPIOB_ModeCfg(GSPWR_PIN, GPIO_ModeOut_PP_5mA);
        GPIOB_ResetBits(GSPWR_PIN);
    }
    else
    {
        GPIOB_ModeCfg(GSPWR_PIN, GPIO_ModeIN_Floating);
        GPIOB_SetBits(GSPWR_PIN);
    }
}
/**
 * @brief   zt09串口开关
 * @param
 *      onoff
 * @return 针对ZT09串口特殊处理
 */
void portDebugUartCfg(uint8_t onoff)
{
	if (onoff)
	{
		portUartCfg(APPUSART2, 1, 115200, doDebugRecvPoll);
	}
	else
	{
		if (sysinfo.sleep && sysinfo.kernalRun == 0)
		{
//			portUartCfg(APPUSART2, 0, 115200, NULL);
	    }
	}
}

/**
 * @brief   IIC 引脚初始化
 * @param
 *      onoff
 * @return
 */
void portIICCfg(void)
{
    SCL_OUTHIGH;
    SDA_OUTHIGH;
}


/**
 * @brief   IIC SCL
 * @param
 * @return
 */
uint8_t sclReadInput(void)
{
    GPIOB_ModeCfg(SCL_PIN, GPIO_ModeIN_Floating);
    if (GPIOB_ReadPortPin(SCL_PIN))
    {
        return 1;
    }
    return 0;
}
/**
 * @brief   IIC SDA
 * @param
 * @return
 */
uint8_t sdaReadInput(void)
{
    GPIOB_ModeCfg(SDA_PIN, GPIO_ModeIN_Floating);
    if (GPIOB_ReadPortPin(SDA_PIN))
    {
        return 1;
    }
    return 0;
}
/**
 * @brief   IIC 产生起始信号
 * @param
 * @return
 */
static void iicStart(void)
{
    SDA_OUTHIGH;
    SCL_OUTHIGH;
    DelayUs(5);
    SDA_OUTLOW
    ;
    DelayUs(5);
}
/**
 * @brief   IIC 产生停止信号
 * @param
 * @return
 */
static void iicStop(void)
{
    SCL_OUTLOW;
    SDA_OUTLOW;
    DelayUs(5);
    SCL_OUTHIGH;
    ;
    DelayUs(5);
    SDA_OUTHIGH;
    DelayUs(5);
}
/**
 * @brief   IIC 发送一个字节
 * @param
 *      data
 * @return
 */
static uint8_t iicSendOneByte(uint8_t data)
{
    int8_t i, ret = 0;
    for (i = 7; i >= 0; i--)
    {
        SCL_OUTLOW;
        DelayUs(5);
        if (data & (1 << i))
        {
            SDA_OUTHIGH;
        }
        else
        {
            SDA_OUTLOW;
        }
        DelayUs(5);
        SCL_OUTHIGH;
        DelayUs(5);
    }
    SCL_OUTLOW;
    DelayUs(5);
    SDA_OUTHIGH;
    DelayUs(5);
    SCL_OUTHIGH;
    if (SDA_READ == 0)
    {
        ret = 1;
    }
    SCL_OUTLOW;
    DelayUs(5);
    return ret;
}
/**
 * @brief   IIC 读取一个字节
 * @param
 * @return
 */
static uint8_t iicReadOneByte(uint8_t ack)
{
    uint8_t readByte = 0;
    int8_t i;
    for (i = 7; i >= 0; i--)
    {
        SCL_OUTLOW;
        DelayUs(5);
        SCL_OUTHIGH;
        DelayUs(5);
        if (SDA_READ)
        {
            readByte |= (1 << i);
        }
        DelayUs(5);
    }
    SCL_OUTLOW;
    DelayUs(5);
    if (ack)
    {
        SDA_OUTLOW;
    }
    else
    {
        SDA_OUTHIGH;
    }
    DelayUs(5);
    SCL_OUTHIGH;
    DelayUs(5);
    SCL_OUTLOW;
    SDA_OUTHIGH;
    DelayUs(5);
    return readByte;
}
/**
 * @brief   IIC 读取多个字节
 * @param
 * @return
 */
uint8_t iicReadData(uint8_t addr, uint8_t regaddr, uint8_t *data, uint8_t len)
{
    uint8_t i, ret = 0;
    if (data == NULL)
        return ret;
    iicStart();
    addr &= ~0x01;
    ret = iicSendOneByte(addr);
    iicSendOneByte(regaddr);
    iicStart();
    addr |= 0x01;
    iicSendOneByte(addr);
    for (i = 0; i < len; i++)
    {
        if (i == (len - 1))
        {
            data[i] = iicReadOneByte(0);
        }
        else
        {
            data[i] = iicReadOneByte(1);
        }
    }
    iicStop();
    return ret;
}
/**
 * @brief   IIC 写多个字节
 * @param
 * @return
 */
uint8_t iicWriteData(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t ret = 0;
    iicStart();
    addr &= ~0x01;
    ret = iicSendOneByte(addr);
    iicSendOneByte(reg);
    iicSendOneByte(data);
    iicStop();
    return ret;
}
/**
 * @brief   gsensor控制
 * @param
 *      onoff
 * @return
 */
void portGsensorCtl(uint8_t onoff)
{
    portIICCfg();
    if (onoff && sysinfo.gsensorOnoff == 0)
    {
        sysinfo.gsensorOnoff = 1;
        GSPWR_ON;
		DelayMs(10);
        mir3da_init();

		PWR_PeriphWakeUpCfg( ENABLE, RB_SLP_GPIO_WAKE, Long_Delay );
        GPIOB_ModeCfg(GSINT_PIN, GPIO_ModeIN_PU);
        GPIOB_ITModeCfg(GSINT_PIN, GPIO_ITMode_FallEdge);
        if (GPIOB_ReadPortPin(GSINT_PIN)) 
		{
        	GPIOB_ResetBits(GSINT_PIN);
	    }
	    else
	    {
	        GPIOB_SetBits(GSINT_PIN);
	    }
        PFIC_EnableIRQ(GPIO_B_IRQn);
        LogPrintf(DEBUG_ALL, "gsensor %s", onoff ? "On" : "Off");
    }
    else if (onoff == 0 && sysinfo.gsensorOnoff == 1)
    {
        sysinfo.gsensorOnoff = 0;
        GSPWR_OFF;
        GPIOB_ModeCfg(GSINT_PIN, GPIO_ModeIN_PD);
        R16_PB_INT_EN &= ~GSINT_PIN;
        terminalAccoff();
        motionClear();
        gpsRequestClear(GPS_REQUEST_ACC_CTL); //如果acc off要关闭gps的acc请求
        LogPrintf(DEBUG_ALL, "gsensor %s", onoff ? "On" : "Off");
    }
    
}


/**
 * @brief   RTC 配置
 * @param
 * @return
 */
void portRtcCfg(void)
{
    LClk32K_Select(Clk32K_LSI);
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
    R8_SLP_WAKE_CTRL |= RB_SLP_RTC_WAKE;// RTC唤醒
    R8_RTC_MODE_CTRL |= (RB_RTC_TMR_EN | RB_RTC_TMR_MODE); //定时模式

    R8_SAFE_ACCESS_SIG = 0;
    PFIC_EnableIRQ(RTC_IRQn);
}

/**
 * 
 * @brief 获取RTC时间偏正值
 * @return int32_t
 * @date 2023-06-13
 * @author lvvv
 * 
 * @details 
 * @note 
 * @par 修改日志:
 * <table>
 * <tr><th>Date 			<th>Author		   <th>Description
 * <tr><td>2023-6-13	    <td>lvvv 			<td>新建
 * </table>
 */

int32_t portGetRtcOffset(void)
{
    return dynamicParam.rtcOffset;
}


/**
 * 
 * @brief 获取偏正后的RTC时间
 * @return 
 * @date 2023-06-13
 * @author lvvv
 * 
 * @details 
 * @note 
 * @par 修改日志:
 * <table>
 * <tr><th>Date 			<th>Author		   <th>Description
 * <tr><td>2023-6-13	   <td>lvvv 			  <td>新建
 * </table>
 */

uint32_t portGetTimestamp(void)
{
    uint32_t  t;
    uint16_t  day, sec2, t32k;

    day = R32_RTC_CNT_DAY & 0x3FFF;
    sec2 = R16_RTC_CNT_2S; 
    
    t32k = RTC_GetCycle32k();						//获取当前32K的振荡周期

    t = day*86400+(sec2<<1) + ((t32k<0x8000)?0:1);	//这一步是把当前运行的RTC时间全部转换为以1S为单位
    												//sec2<<1 等价于sec2*2
    												//0x8000等于32768, t32k>0x8000表示振荡周期已经满1s
    t += portGetRtcOffset();

    return t;
}

/**
 * 
 * @brief Timestamp转换时间
 * @return 
 * @date 2023-06-13
 * @author lvvv
 * 
 * @details 
 * @note 
 * @par 修改日志:
 * <table>
 * <tr><th>Date 			<th>Author		   <th>Description
 * <tr><td>2023-6-13	   	<td>lvvv 			<td>新建
 * </table>
 */

void portConvertTimestampToTime(system_date_s *tm, uint32_t sec_time)
{
  // calculate the time less than a day - hours, minutes, seconds
  {
    uint32_t day = sec_time % DAY;
    tm->second = day % 60UL;
    tm->minute = (day % 3600UL) / 60UL;
    tm->hour = day / 3600UL;
    //LogPrintf(DEBUG_ALL, "sec_time:%d h:%d, m:%d, s:%d", sec_time, tm->hour, tm->minute, tm->second);
  }
  // Fill in the calendar - day, month, year
  {
    uint16_t numDays = sec_time / DAY;
    tm->year = BEGYEAR;
    
    while ( numDays >= (IsLeapYear(tm->year) ? 366 : 365) )
    {
      numDays -= YearLength( tm->year );
      tm->year++;
    }

    tm->month = 0;
    while ( numDays >= monthLength( IsLeapYear( tm->year ), tm->month ) )
    {
      numDays -= monthLength( IsLeapYear( tm->year ), tm->month );
      tm->month++;
    }
 
    tm->date = numDays;
  }
  //offset
  tm->date +=1;
  tm->month +=1;
}

/**
 * 
 * @brief  时间转换成timestamp
 * @return  timestamp
 * @date 2023-06-13
 * @author lvvv
 * 
 * @details 
 * @note 
 * @par 修改日志:
 * <table>
 * <tr><th>Date 			<th>Author		   <th>Description
 * <tr><td>2023-6-13	   	<td>lvvv 			<td>新建
 * </table>
 */
uint32_t portConvertTimeToTimestamp(system_date_s *tm)
{
	uint32_t seconds;

  //offset
  tm->date -=1;
  tm->month -=1;
 
  /* Seconds for the partial day */
  seconds = (((tm->hour * 60UL) + tm->minute) * 60UL) + tm->second;
 
  /* Account for previous complete days */
  {
    /* Start with complete days in current month */
    uint16_t days = tm->date;
 
    /* Next, complete months in current year */
    {
      int8_t month = tm->month;
      while ( --month >= 0 )
      {
        days += monthLength( IsLeapYear( tm->year ), month );
      }
    }
 
    /* Next, complete years before current year */
    {
      uint16_t year = tm->year;
      while ( --year >= BEGYEAR )
      {
        days += YearLength( year );
      }
    }
 
    /* Add total seconds before partial day */
    seconds += (days * DAY);
  }
 
  return ( seconds );
}

/**
 * 
 * @brief 保存timestamp
 * @return 
 * @date 2023-06-13
 * @author lvvv
 * 
 * @details 
 *  set local rtc cnt in seconds
 *	the max timestamp in hardware is (0x3fff*86400)+(86400-1) about 44 year
 *	when use this in ble, TMOS_TimerInit must be executed after it. 
 *	and this will cause ble connection lost
 * @note 
 * @par 修改日志:
 * <table>
 * <tr><th>Date 			<th>Author		   <th>Description
 * <tr><td>2023-6-13	   	<td>lvvv 			<td>新建
 * </table>
 */
void portSetTimestamp(uint32_t timestamp)
{
	
    uint32_t  t;
    uint16_t  day, sec2, t32k;

    day = R32_RTC_CNT_DAY & 0x3FFF;
    sec2 = R16_RTC_CNT_2S;

    t32k = RTC_GetCycle32k();

    t = day*86400+(sec2<<1) + ((t32k<0x8000)?0:1);

    int32_t offset = timestamp - t;

	dynamicParam.rtcOffset = offset;
	dynamicParamSaveAll();
}


/**
 * 
 * @brief 获取偏正后的RTC时间
 * @return 
 * @date 2023-06-13
 * @author lvvv
 * 
 * @details 
 * @note 
 * @par 修改日志:
 * <table>
 * <tr><th>Date 			<th>Author		   <th>Description
 * <tr><td>2023-6-13	   <td>lvvv 			  <td>新建
 * </table>
 */

void portGetRtcDateTime(uint16_t *year, uint8_t *month, uint8_t *date, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
	system_date_s	datetime;
	uint32_t timestamp = portGetTimestamp();
	portConvertTimestampToTime(&datetime, timestamp);																
	*year   = datetime.year;
	*month  = datetime.month;
	*date   = datetime.date;
	*hour   = datetime.hour;
	*minute = datetime.minute;
	*second = datetime.second;
}

/**
 * 
 * @brief 修改RTCoffset值
 * @return 
 * @date 2023-06-13
 * @author lvvv
 * 
 * @details 更改时间只需要调节offset值即可,不需要改变rtc硬件寄存器的时间,解决使用蓝牙协议栈更新RTC时间会复位的问题
 * @note 
 * @par 修改日志:
 * <table>
 * <tr><th>Date 			<th>Author		   <th>Description
 * <tr><td>2023-6-13	    <td>lvvv 			  <td>新建
 * </table>
 */

 void portUpdateRtcOffset(uint8_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minute, uint8_t second)
 {
 	uint16_t a;
 	uint8_t b, c, d, e, f;
	uint32_t sec1, sec2;
	system_date_s datetime;
	uint32_t timestamp;
	portGetRtcDateTime(&a, &b, &c, &d, &e, &f);
    sec1 = d * 3600 + e * 60 + f;
    sec2 = hour * 3600 + minute * 60 + second;
    if ((year + 2000) != a || b != month || c != date || abs(sec1 - sec2) >= 10)
    {
		datetime.year   = year + 2000;
		datetime.month  = month;
		datetime.date   = date;
		datetime.hour   = hour;
		datetime.minute = minute;
		datetime.second = second;
		timestamp = portConvertTimeToTimestamp(&datetime);
		portSetTimestamp(timestamp);
		LogPrintf(DEBUG_ALL, "update time==>%02d/%02d/%02d-%02d:%02d:%02d", datetime.year, datetime.month, datetime.date, datetime.hour, datetime.minute, datetime.second);
    }
    else
    {
		LogPrintf(DEBUG_ALL, "current Datetime==>%02d/%02d/%02d %02d:%02d:%02d", a, b, c, d, e, f);
    }
 }



/**
 * @brief   更新RTC时间
 * @param
 * @note    使用蓝牙协议栈时，切勿更改rtc时间
 * @return
 */
void portUpdateRtcDateTime(uint8_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minute, uint8_t second)
{
    uint16_t a, b, c, d, e, f;
    uint32_t sec1, sec2;
    RTC_GetTime(&a, &b, &c, &d, &e, &f);
    if (sysinfo.logLevel == DEBUG_FACTORY)
    {
        return ;
    }

    sec1 = d * 3600 + e * 60 + f;
    sec2 = hour * 3600 + minute * 60 + second;
    if ((year + 2000) != a || b != month || c != date || abs(sec1 - sec2) >= 60)
    {
        RTC_InitTime(year + 2000, month, date, hour, minute, second);
        LogPrintf(DEBUG_ALL, "update time==>%02d/%02d/%02d-%02d:%02d:%02d", year + 2000, month, date, hour, minute, second);
        portSysReset();
    }
    else
    {
        LogPrintf(DEBUG_ALL, "current Datetime==>%02d/%02d/%02d %02d:%02d:%02d", a, b, c, d, e, f);
    }
}

/**
 * @brief   设置闹铃时间
 * @param
 * @return
 */
static void portUpdateAlarmTime(uint8_t date, uint8_t hour, uint8_t minutes)
{
    sysinfo.alarmDate = date;
    sysinfo.alarmHour = hour;
    sysinfo.alarmMinute = minutes;
    LogPrintf(DEBUG_ALL, "SetTimeAlarm==>Date:%02d,Time:%02d:%02d", date, hour, minutes);
}

/**
  * @brief  设置下一次时间点
  * @param  None
  * @retval None
  */

int portSetNextAlarmTime(void)
{
    unsigned short  rtc_mins, next_ones;
    unsigned char next_date, set_nextdate = 1;
    uint16_t  YearToday;      /*当前年*/
    uint8_t  MonthToday;     /*当前月*/
    uint8_t  DateToday;      /*当前日*/
    int i;
    //RTC_AlarmTypeDef rtc_a;
    uint16_t year;
    uint8_t  month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);

    //1、读取当前时间点的总分钟数
    rtc_mins = (hour & 0x1F) * 60;
    rtc_mins += (minute & 0x3f);
    //2、读取当前年月
    YearToday = year; //计算当前年，从2000年开始算起
    MonthToday = month;
    DateToday = date;
    //3、根据当前月，计算下个月日期
    if (MonthToday == 4 || MonthToday == 6 || MonthToday == 9 || MonthToday == 11)
    {
        next_date = (DateToday + sysparam.gapDay) % 30; //当前日期加上间隔日，计算下一次的时间点
        if (next_date == 0)
            next_date = 30;
    }
    else if (MonthToday == 2)
    {
        //4、如果是2月，判断是不是闰年
        if (((YearToday % 100 != 0) && (YearToday % 4 == 0)) || (YearToday % 400 == 0))  //闰年
        {
            next_date = (DateToday + sysparam.gapDay) % 29;
            if (next_date == 0)
                next_date = 29;
        }
        else
        {
            next_date = (DateToday + sysparam.gapDay) % 28;
            if (next_date == 0)
                next_date = 28;
        }
    }
    else
    {
        next_date = (DateToday + sysparam.gapDay) % 31;
        if (next_date == 0)
            next_date = 31;
    }
    next_ones = 0xFFFF;
    //5、查找闹铃表里面是否有在当前时间点之后的时间
    for (i = 0; i < 5; i++)
    {
        if (sysparam.AlarmTime[i] == 0xFFFF)
            continue;
        if (sysparam.AlarmTime[i] > rtc_mins)   //跟当前时间比对
        {
            next_ones = sysparam.AlarmTime[i];  //得到新的时间
            if (next_ones < 1440) 
            {
                set_nextdate = 0;
            }
            break;
        }
    }

    if (next_ones == 0xFFFF)  //没有配置时间
    {
        //Set Current Alarm Time
        next_ones = sysparam.AlarmTime[0];
        if (next_ones == 0xFFFF)
        {
            next_ones = 720; //默认12:00
        }
    }
    //6、设置下次上报的日期和时间
    if (set_nextdate)
        portUpdateAlarmTime(next_date, (next_ones / 60) % 24, next_ones % 60);
    else
        portUpdateAlarmTime(date, (next_ones / 60) % 24, next_ones % 60);
    return 0;
}


/**
  * @brief  设置MODE4下一次唤醒时间
  * @param  None
  * @retval None
  */

int portSetNextMode4AlarmTime(void)
{
	if (sysparam.mode4Alarm == 0xFFFF)
		return 0;
	sysinfo.mode4alarmHour = (sysparam.mode4Alarm / 60) % 24;
	sysinfo.mode4alarmMinute = sysparam.mode4Alarm % 60;
//    unsigned short  rtc_mins, next_ones;
//    unsigned char next_date, set_nextdate = 1;
//    uint16_t  YearToday;      /*当前年*/
//    uint8_t  MonthToday;     /*当前月*/
//    uint8_t  DateToday;      /*当前日*/
//    int i;
//    uint16_t year;
//    uint8_t  month;
//    uint8_t date;
//    uint8_t hour;
//    uint8_t minute;
//    uint8_t second;
//
//    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
//
//    //1、读取当前时间点的总分钟数
//    rtc_mins = (hour & 0x1F) * 60;
//    rtc_mins += (minute & 0x3f);
//    //2、读取当前年月
//    YearToday = year; //计算当前年，从2000年开始算起
//    MonthToday = month;
//    DateToday = date;
//    //3、根据当前月，计算下个月日期
//    if (MonthToday == 4 || MonthToday == 6 || MonthToday == 9 || MonthToday == 11)
//    {
//        next_date = (DateToday + 1) % 30; //当前日期加上间隔日，计算下一次的时间点
//        if (next_date == 0)
//            next_date = 30;
//    }
//    else if (MonthToday == 2)
//    {
//        //4、如果是2月，判断是不是闰年
//        if (((YearToday % 100 != 0) && (YearToday % 4 == 0)) || (YearToday % 400 == 0))  //闰年
//        {
//            next_date = (DateToday + 1) % 29;
//            if (next_date == 0)
//                next_date = 29;
//        }
//        else
//        {
//            next_date = (DateToday + 1) % 28;
//            if (next_date == 0)
//                next_date = 28;
//        }
//    }
//    else
//    {
//        next_date = (DateToday + 1) % 31;
//        if (next_date == 0)
//            next_date = 31;
//    }
//    next_ones = 0xFFFF;
//	//5、比对mode4时间表
//	if (sysparam.mode4Alarm == 0xFFFF)
//	{
//		return 0;
//	}
//	else 
//	{
//		//今天
//		if (sysparam.mode4Alarm > rtc_mins)
//		{
//			next_ones = sysparam.mode4Alarm;
//			if (next_date < 1440)
//			{
//				set_nextdate = 0;
//			}
//		}
//		//明天
//		else
//		{
//			set_nextdate = 1;
//		}	
//	}
//	if (set_nextdate)
//	{
//		sysinfo.mode4alarmDate = next_date;
//		sysinfo.mode4alarmHour = (next_ones / 60) % 24;
//		sysinfo.mode4alarmMinute = (next_ones / 60) % 24;
//	}
//	else
//	{
//		sysinfo.mode4alarmDate = date;
//		sysinfo.mode4alarmHour = (next_ones / 60) % 24;
//		sysinfo.mode4alarmMinute = (next_ones / 60) % 24;
//	}
//	return 1;
}


/**
 * @brief   设置下次唤醒时间
 * @param
 * @return
 */
void portSetNextWakeUpTime(void)
{
    uint16_t  YearToday;      /*当前年*/
    uint8_t  MonthToday;     /*当前月*/
    uint8_t  DateToday;      /*当前日*/
    uint8_t  date = 0, next_date;
    uint16_t totalminutes;
    uint16_t year;
    uint8_t  month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    portGetRtcDateTime(&year, &month, &day, &hour, &minute, &second);
    YearToday = year; //计算当前年，从2000年开始算起
    MonthToday = month;
    DateToday = day;
    totalminutes = hour * 60 + minute;
    totalminutes += sysparam.gapMinutes;
    if (totalminutes >= 1440)
    {
        date = 1;
        totalminutes -= 1440;
    }
    //3、根据当前月，计算下个月日期
    if (MonthToday == 4 || MonthToday == 6 || MonthToday == 9 || MonthToday == 11)
    {
        next_date = (DateToday + date) % 30; //当前日期加上间隔日，计算下一次的时间点
        if (next_date == 0)
            next_date = 30;
    }
    else if (MonthToday == 2)
    {
        //4、如果是2月，判断是不是闰年
        if (((YearToday % 100 != 0) && (YearToday % 4 == 0)) || (YearToday % 400 == 0))  //闰年
        {
            next_date = (DateToday + date) % 29;
            if (next_date == 0)
                next_date = 29;
        }
        else
        {
            next_date = (DateToday + date) % 28;
            if (next_date == 0)
                next_date = 28;
        }
    }
    else
    {
        next_date = (DateToday + date) % 31;
        if (next_date == 0)
            next_date = 31;
    }
    portUpdateAlarmTime(next_date, (totalminutes / 60) % 24, totalminutes % 60);
}

/**
 * @brief   低功耗配置
 * @param
 * @return
 */
void portLowPowerCfg(void)
{
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
    R8_SLP_WAKE_CTRL |= RB_SLP_RTC_WAKE;    //RTC唤醒
    R8_SLP_WAKE_CTRL |= RB_SLP_GPIO_WAKE;   //gpio中断唤醒
    R8_SLP_WAKE_CTRL &= ~RB_SLP_BAT_WAKE;   //关闭电池低压唤醒
    R8_SAFE_ACCESS_SIG = 0;
}

/**
 * @brief   adc相关配置
 * @param
 * @return
 */
void portAdcCfg(uint8_t onoff)
{
    //PA9  channel 3
    if (onoff)
    {
    	sysinfo.adcOnoff = 1;
        GPIOA_ModeCfg(VCARD_ADCPIN, GPIO_ModeIN_Floating);
    }
    else
    {
    	sysinfo.adcOnoff = 0;
        GPIOA_ModeCfg(VCARD_ADCPIN, GPIO_ModeIN_PU);
    }
}
/**
 * @brief   读取ADC电压
 * @param
 *      channel
 * @return
 *      float adc电压值
 */
float portGetAdcVol(ADC_SingleChannelTypeDef channel)
{
#define ADC_DET_CNT		7
    float value;
    float x[ADC_DET_CNT];
    uint8_t i;
    ADC_ChannelCfg(channel);
    DelayUs(1);
    ADC_ExtSingleChSampInit(SampleFreq_8, ADC_PGA_0);
    ADC_ExcutSingleConver();
    value = (ADC_ExcutSingleConver() / 2048.0) * 1.05;
    if (value >= 2.0)
    {
        ADC_ExtSingleChSampInit(SampleFreq_8, ADC_PGA_1_2);
        ADC_ExcutSingleConver();
        value = (ADC_ExcutSingleConver() / 1024.0 - 1) * 1.05;
    }
    
    for (i = 0; i < ADC_DET_CNT; i++)
    {
	    ADC_ExtSingleChSampInit(SampleFreq_8, ADC_PGA_0);
	    ADC_ExcutSingleConver();
	    x[i] = (ADC_ExcutSingleConver() / 2048.0) * 1.05;
	    if (value >= 2.0)
	    {
	        ADC_ExtSingleChSampInit(SampleFreq_8, ADC_PGA_1_2);
	        ADC_ExcutSingleConver();
	        x[i] = (ADC_ExcutSingleConver() / 1024.0 - 1) * 1.05;
	    }	 
    }
    sort(x, ADC_DET_CNT);
    value = 0.0;
    for (i = 1; i < ADC_DET_CNT - 1; i++)
    {
		value += x[i];
    }
    value /= 5.0;
    return value;
}
/**
 * @brief   使能睡眠
 * @param
 * @return
 */
void portSleepEn(void)
{
    sleepSetState(1);
}
/**
 * @brief   禁止睡眠
 * @param
 * @return
 */
void portSleepDn(void)
{
    sleepSetState(0);
}

/**
 * @brief   使能看门狗
 * @param
 * @return
 */
void portWdtCfg(void)
{
    WWDG_SetCounter(0);
    WWDG_ResetCfg(ENABLE);
}

/**
 * @brief   喂狗
 * @param
 * @return
 */
void portWdtFeed(void)
{
    WWDG_SetCounter(0);
}
/**
 * @brief   关闭看门狗
 * @param
 * @return
 */
void portWdtCancel(void)
{
	WWDG_ResetCfg(DISABLE);
}


/**
 * @brief   修改系统主频
 * @param
 		1	高主频
 		0	低主频
 * @return
 */

void portFsclkChange(uint8_t type)
{
//    if (type)
//    {
//    	LogMessage(DEBUG_ALL, "portFsclkChange==>60MHz");
//        SetSysClock(CLK_SOURCE_PLL_60MHz);
//    }
//    else
//    {
//    	LogMessage(DEBUG_ALL, "portFsclkChange==>16MHz");
//        SetSysClock(CLK_SOURCE_HSE_16MHz);
//    }
//    //主频改变，修改波特率计算值
//    UART0_BaudRateCfg(115200);
//    UART1_BaudRateCfg(115200);
//    UART2_BaudRateCfg(115200);
//    UART3_BaudRateCfg(115200);
}
