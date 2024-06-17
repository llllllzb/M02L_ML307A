#ifndef APP_GPS
#define APP_GPS

#include <stdint.h>

#define GPSFIXED	1
#define GPSINVALID	0

#define GPSITEMCNTMAX	24
#define GPSITEMSIZEMAX	20

#define GPSFIFOSIZE	7


typedef struct
{
    unsigned char item_cnt;
    char item_data[GPSITEMCNTMAX][GPSITEMSIZEMAX];
} GPSITEM;


typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    int8_t hour;
    uint8_t minute;
    uint8_t second;
} datetime_s;

typedef struct
{
    datetime_s datetime;//����ʱ��
    char fixstatus; //��λ״̬
    double latitude;//γ��
    double longtitude;//����
    double speed;		//�ٶ�
    uint16_t course;//�����
    float pdop;
	float hight;
    uint32_t gpsticksec;

    char NS; //�ϱ�γ
    char EW; //������
    char fixmode;
	char fixAccuracy;
    unsigned char used_star;//��������
    unsigned char gpsviewstart;//�ɼ�����
    unsigned char beidouviewstart;
    unsigned char glonassviewstart;
    uint8_t hadupload;
    uint8_t beidouCn[30];
    uint8_t gpsCn[30];
} gpsinfo_s;

typedef enum
{
    NMEA_RMC = 1,
    NMEA_GSA,
    NMEA_GGA,
    NMEA_GSV,
} NMEATYPE;

/*------------------------------------------------------*/

typedef struct{
	uint8_t currentindex;
	gpsinfo_s gpsinfohistory[GPSFIFOSIZE];
	gpsinfo_s lastfixgpsinfo;
}GPSFIFO;

/*------------------------------------------------------*/

typedef struct{
	datetime_s datetime;//����ʱ��
	double latitude;//γ��
	double longtitude;//����
	uint32_t gpsticksec;
	uint8_t init;
}LASTUPLOADGPSINFO;



void nmeaParser(uint8_t *buf, uint16_t len);
double latitude_to_double(double latitude, char Direction);
double longitude_to_double(double longitude, char Direction);
/*------------------------------------------------------*/
datetime_s changeUTCTimeToLocalTime(datetime_s utctime,int8_t localtimezone);
void updateLocalRTCTime(datetime_s *datetime);

/*------------------------------------------------------*/

void gpsClearCurrentGPSInfo(void);
gpsinfo_s *getCurrentGPSInfo(void);
gpsinfo_s *getLastFixedGPSInfo(void);
GPSFIFO *getGSPfifo(void);
/*------------------------------------------------------*/
void gpsUploadPointToServer(void);

/*------------------------------------------------------*/

void hdGpsColdStart(void);
void hdGpsHotStart(void);
void hdGpsGsvCtl(uint8_t onoff);

int32_t gnss_inject_location(int32_t latitude, int32_t longitude, float altitude, float accuracy);

int32_t gnss_eph_inject_data(uint8_t *data, int32_t length);


#endif