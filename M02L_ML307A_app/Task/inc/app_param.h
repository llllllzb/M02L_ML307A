#ifndef APP_EEPROM_H
#define APP_EEPROM_H
#include <stdint.h>

#include "app_sys.h"

/*
 * DataFlash 分布
 * [0~1)        Total:1KB   bootloader
 * [1~9)        Total:8KB   systemparam
 * [9~10)		Total:1KB	dynamicparam
 * [10~30)      Total:20KB  database
 * [30~31)		Total:1KB	databaseInfo
 * [31~32)      Total:1KB   blestack
 */

//目前预留前1K字节给bootloader，所以app从0x400开始
#define BOOT_USER_PARAM_ADDR    0x00
#define APP_USER_PARAM_ADDR     0x400  //实际是0x00070000+APP_USER_PARAM_ADDR
#define APP_DYNAMIC_PARAM_ADDR	0x2400 //实际是0x00070000+APP_DYNAMIC_PARAM_ADDR
#define APP_PARAM_FLAG          0x2C
#define BOOT_PARAM_FLAG         0xB0
#define OTA_PARAM_FLAG          0x30

#define EEPROM_VERSION									"M02L_ML307A_V1.0.1"


#define JT808_PROTOCOL_TYPE			8
#define ZT_PROTOCOL_TYPE			0



typedef struct
{
    uint8_t VERSION;        /*当前软件版本号*/
    uint8_t updateStatus;
    uint8_t apn[50];
    uint8_t apnuser[50];
    uint8_t apnpassword[50];
    uint8_t SN[20];
    uint8_t updateServer[50];
    uint8_t codeVersion[50];
    uint16_t updatePort;
} bootParam_s;

/*存储在EEPROM中的数据*/
typedef struct
{
    uint8_t VERSION;        /*当前软件版本号*/
    uint8_t MODE;           /*系统工作模式*/
    uint8_t ldrEn;
    uint8_t gapDay;
    uint8_t heartbeatgap;
    uint8_t ledctrl;
    uint8_t accdetmode;
    uint8_t poitype;
    uint8_t accctlgnss;
    uint8_t apn[50];
    uint8_t apnuser[50];
    uint8_t apnpassword[50];
    uint8_t lowvoltage;
    uint8_t fence;	

    uint8_t Server[50];
    uint8_t agpsServer[50];
    uint8_t agpsUser[50];
    uint8_t agpsPswd[50];
    uint8_t protocol;



    uint8_t hiddenServOnoff;
    uint8_t hiddenServer[50];
    uint8_t jt808Server[50];
    uint8_t bleServer[50];

    int8_t utc;

    uint16_t gpsuploadgap;	//该参数在m02不带gps的设备上是以分钟为单位的
    uint16_t AlarmTime[5];  /*每日时间，0XFFFF，表示未定义，单位分钟*/
    uint16_t gapMinutes;    /*模式三间隔周期，单位分钟*/
	
    uint8_t gsdettime;
    uint8_t gsValidCnt;
    uint8_t gsInvalidCnt;

    uint16_t agpsPort;
    uint16_t jt808Port;
    uint16_t hiddenPort;
    uint16_t ServerPort;
    uint16_t bleServerPort;

    float  adccal;
    float  accOnVoltage;
    float accOffVoltage;


	uint8_t cm;

	uint8_t sosalm;
	uint8_t tiltalm;		/*作为Gsensor和倾斜检测的切换标志，1：倾斜版本， 0：Gsensor版本*/
	uint8_t leavealm;
	
	uint8_t bleLinkFailCnt;
	uint8_t tiltThreshold;

	uint8_t otaParamFlag;
	uint8_t agpsen;
	uint8_t debug;

	uint16_t mode4Alarm;
	uint8_t smsreply;
	uint8_t bf;
	uint8_t batsel;

	float pdop;
	uint8_t fixmode;
	uint8_t bleen;
	uint16_t mode23_acc_on_min;
} systemParam_s;

/*存在EEPROM里的动态参数*/
typedef struct
{
	uint8_t SN[20];
    uint8_t jt808sn[7];
    uint8_t jt808isRegister;
    uint8_t jt808AuthCode[50];
    uint8_t jt808AuthLen;

    uint8_t bleLinkCnt;     
    uint16_t runTime;       /*模式二的工作时间，单位分钟*/
    uint16_t startUpCnt;
    uint16_t noNmeaRstCnt;
    uint8_t debug;
    int32_t rtcOffset;
    double saveLat;
	double saveLon;
}dynamicParam_s;



typedef enum{
	ALARM_TYPE_NONE,
	ALARM_TYPE_GPRS,
	ALARM_TYPE_GPRS_SMS,
	ALARM_TYPE_GPRS_SMS_TEL,
}AlarmType_e;

extern systemParam_s sysparam;
extern bootParam_s bootparam;
extern dynamicParam_s dynamicParam;

void bootParamSaveAll(void);
void bootParamGetAll(void);

void paramSaveAll(void);
void paramGetAll(void);

void dynamicParamSaveAll(void);
void dynamicParamGetAll(void);

void paramInit(void);
void paramDefaultInit(uint8_t type);


#endif

