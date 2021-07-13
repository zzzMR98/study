#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <math.h>
#include <fstream>
#include <unistd.h>
#include  <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <serial/serial.h>


#define BUFSIZE 4096
typedef char                 UINT;
typedef unsigned char                 UCHAR;
/******************GPS  Model output***************************************/
 struct struct_GPS_IMU
{
  bool GPFPDflag;
  bool GTIMUflag;
	double dLongitude;
	double dLatitude;
	double daltitude;
	
	double droll;
	double dpitch;
	double dheading;
	
	double droX;
	double droY;
	double droZ;
	
	double AccX;
	double AccY;
	double AccZ;
	
	double dVelE;
	double dVelN;
	double dVelSky;
	
	int GPSWeek;
	double GPSTime;
	double HDOP;//水平精度因子
// 	int DGPSState;//0:初始化，1：单点定位；2：码差分；3：无效PPS；4：固定解；5：浮点解；6：正在估算；7：人工输入固定值；8：模拟模式；9：WAAS差分。
// 	char	DGPSState_VTG;//A自主定位；D差分；E估算。
// 	char	DGPSState_HDT;
	double	lTimeStamp=0;			// TimeStamp(Unit:ms)
	double UTCtime;
	int UTCdate;
	int year = 0;
        int month = 0;
        int day = 0;
        int hour = 0;
        int min = 0;
        double second = 0;
	double gravity;
// 	char status[5];
// 	std::string statusstr;
};

class CAnalysisGPS_IMU
{
   public:
	 struct_GPS_IMU   Data_struct;
    CAnalysisGPS_IMU(); 
    ~CAnalysisGPS_IMU(); 
    bool UDPInit(int port);
    bool SerialInit(const std::string& comport, int baudrate, int timeout_ms);
    void Update();
    bool check(char buf[]);
//     void UpdateChar(int len,unsigned char* buffer);
//     void stringtochar(std::string str,char result[4096]);

  private:
    void DataGPGGA(char Data[],int n);
    void DataGPRMC(char Data[],int n);
    void DataGPFPD(char Data[],int n);
    void DataGPHPD(char Data[],int n);
    void DataGTIMU(char Data[],int n);
  
    char m_cGPSDataFromReceiver[200];
    bool CreatNetSucess;


    sockaddr_in myaddr; /* our address */
    sockaddr_in remaddr; /* remote address */
    socklen_t addrlen; /* length of addresses */

    int recvlen; /* # bytes received */
    int fd; /* our socket */
//     unsigned char buf1[BUFSIZE]; /* receive buffer */
    
    serial::Serial ser;
//    std::fstream data_backup;
};
