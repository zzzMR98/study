#ifndef IMU531_H
#define IMU531_H

//#include "m_navin.hh"
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <math.h>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*POSIX 终端控制定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/stat.h>
#include <serial/serial.h>
#include <string>

#define CV_PI_1	3.14159265

#define PORT 9001
#define BUFSIZE 2048

//9001

//192.168.0.254 Nport IMU
//192.168.0.253 Nport ECU

//192.168.0.111 Computer
//192.168.0.112 Computer

union longbyte				//a special kind of class
{
    double _double;
  unsigned int _long;
  char _char[4];
};


union UucCharOrShortInt		//有符号短整形
{
  unsigned char ucData[2];
  short int shintData;
};


struct SFONSData
{

    double dHeading;
    double dPitch;
    double dRoll;

    double dAccx;
    double dAccy;
    double dAccz;

    double dArx;	//Pear
    double dAry;
    double dArz;

    double dVelN;	//Pear
    double dVelSky;
    double dVelE;

    double dFOSNLat;	//Pear
    double dFOSNLng;
    double dFOSNAltitude;

    double dFOSNState;

    double dGPSLat;	//Pear
    double dGPSLng;
    double dGPSDop;

    double dGPSCounter;

    //	double dState;																	Pear注释掉的
    //	char GPSState;

    //	unsigned char ucStarNum;														Pear注释掉的
    //	unsigned char ucFONSState;

    //	unsigned char ucStaThreshold;		//ECU通信状态阈值								Pear注释掉的
    unsigned char ucStaTimes;		//ECU通信状态计数
    //	unsigned char ucStaOK;			//ECU通信状态										Pear注释掉的

    //double				dTimeStamp;			//IMU时间戳
    //	int                	 dTimeStamp;	 											Pear注释掉的

    //	double				m_AHRSHeading;			//IMU发来的Heading                 Pear注释掉的
    //	double				dIPitch;			//IMU发来的Pitch
    //	double				dIRoll;				//IMU发来的Roll
    //
    //	double				dGyroX;				//IMU发来的绕X向Angular rate
    //	double				dGyroY;				//IMU发来的绕Y向Angular rate
    //	double				dGyroZ;				//IMU发来的绕Z向Angular rate
    //
    //	double				dVdd;				//IMU发来的电压
    //	double				dTemp;				//IMU发来的平均温度

    //	unsigned char ucOStaThreshold;		//IMU通信状态阈值								Pear注释掉的
    //	unsigned char ucOStaTimes;			//IMU通信状态计数
    //	unsigned char ucOStaOK;			//IMU通信状态

    //	//*****新加  fons ASCII 码协议部分 												Pear注释掉的
    //	double              dRunTime;           //FONS 上电运行时间
    //	double              dUTC;                //FONS utc世界协调时间
    //	double              dEastV;             // FONS  东向速度
    //	double              dNorthV;            // FONS  北向速度
    //	double              dAltitudeV;         // 天向速度

    //坐标解算之后的，与协议无关
    double east;
    double north;
    unsigned int utc_time;
};
class Imu_531
{
  public:
    Imu_531(); //构造函数，赋予初值
    ~Imu_531(); //析构函数，释放
    bool ImuInit(int port);
	bool SerialInit(const std::string& comport, int baudrate, int timeout_ms, int toUDPPort = 9001);
    void UpdateBySerial();
    void GetImuData();
    void Update();
    SFONSData m_sFONSData;
    void ParseImu(char* ch,int len);
  private:



    unsigned int sum;
    unsigned int check_sum;
    unsigned char Fhead[3];
    bool FheadFound;
    bool FDataPacketCorr;
    unsigned int FbyteCount;
    short Fheading_angle; //拼接变量，表示有多少个180/(2^15-1)单位
    UucCharOrShortInt Fheading;
    longbyte gps_temp;

    double heading_angle;  //转换成角度
    unsigned char Rx_FOSNData[62];

    void ParseImu(char ch);
    void ParseImu_ASCII(char ch);

    sockaddr_in myaddr; /* our address */
    sockaddr_in remaddr; /* remote address */
    sockaddr_in toaddr; /// send data from serial port for data playback
    socklen_t addrlen; /* length of addresses */
    int recvlen; /* # bytes received */
    int fd; /* our socket */
    unsigned char buf[BUFSIZE]; /* receive buffer */

    char            m_cFONSData[150];                   //���ڽ���FONS��ASCII��ʽ��ͨѶ���
    char            m_cFONSTemp[150];

    //imu date save
    std::fstream data_backup;
    int fdSerial;
	serial::Serial ser;
	
};

void Position_Trans_From_ECEF_To_UTM(double latitude,double longitude,double e0, double n0, double *e, double *n);

#endif
