
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <math.h>
#include <fstream>
#include <unistd.h>

#include  <iostream>


#define BUFSIZE 4096
//#define M_PI 3.1415926535897932384626433832795  //!< Pi.
#define RAD2DEG             180.0/M_PI  //!< Convert radians to degrees.
#define RAD2DEG             180.0/M_PI //!< Convert radians to degrees.
typedef unsigned char                 UCHAR;
/******************INS  Model output***************************************/
 struct struct_INS1
 {
 	double dHeading;
 	double dPitch;
 	double dRoll;

 	double dAccx;
 	double dAccy;
 	double dAccz;

 	double dLat;
 	double dLng;
 	double dAltitude;

 	double dState;
 	char GPSState;

 	UCHAR				ucStarNum;
 	UCHAR				ucFONSState;

 	UCHAR				ucStaThreshold;		//ECUÍšÐÅ×ŽÌ¬ãÐÖµ
 	UCHAR				ucStaTimes;			//ECUÍšÐÅ×ŽÌ¬ŒÆÊý
 	UCHAR				ucStaOK;			//ECUÍšÐÅ×ŽÌ¬

 	double				dTimeStamp;			//IMUÊ±ŒäŽÁ

 	double				m_AHRSHeading;			//IMU·¢ÀŽµÄHeading
 	double				dIPitch;			//IMU·¢ÀŽµÄPitch
 	double				dIRoll;				//IMU·¢ÀŽµÄRoll


 	double				dGyroX;				//IMU·¢ÀŽµÄÈÆXÏòAngular rate
 	double				dGyroY;				//IMU·¢ÀŽµÄÈÆYÏòAngular rate
 	double				dGyroZ;				//IMU·¢ÀŽµÄÈÆZÏòAngular rate


 	double				dVdd;				//IMU·¢ÀŽµÄµçÑ¹
 	double				dTemp;				//IMU·¢ÀŽµÄÆœŸùÎÂ¶È



 	UCHAR				ucOStaThreshold;		//IMUÍšÐÅ×ŽÌ¬ãÐÖµ
 	UCHAR				ucOStaTimes;			//IMUÍšÐÅ×ŽÌ¬ŒÆÊý
 	UCHAR				ucOStaOK;			//IMUÍšÐÅ×ŽÌ¬


 	//*****ÐÂŒÓ  fons ASCII ÂëÐ­Òé²¿·Ö
 	double              dRunTime;           //FONS ÉÏµçÔËÐÐÊ±Œä
 	double              dUTC;                //FONS utcÊÀœçÐ­µ÷Ê±Œä
 	double              dEastV;             // FONS  ¶«ÏòËÙ¶È
 	double              dNorthV;            // FONS  ±±ÏòËÙ¶È
 	double              dAltitudeV;         // ÌìÏòËÙ¶È


 };


class CAnalysisINS_OxT
{
   public:
    struct_INS1  INSData_struct;
    CAnalysisINS_OxT(); 
    ~CAnalysisINS_OxT(); 
    bool Init(int port);
    void Update();
    void ParseImu(char* ch,int len);


  private:
   // void OctansData(UCHAR * OctansData,int n);
    void IMU_DataProcFromOxTS(unsigned char* data,int count);
	unsigned int cast_2_byte_to_uint32(unsigned char*  b);
	unsigned short cast_2_byte_to_ushort(unsigned char*  b);
	int cast_3_byte_to_int32(unsigned char*  b);
	double  cast_8_byte_to_double(unsigned char* b);
	float cast_4_byte_to_float(unsigned char *b);

    sockaddr_in myaddr; /* our address */
    sockaddr_in remaddr; /* remote address */
    socklen_t addrlen; /* length of addresses */

    int recvlen; /* # bytes received */
    int fd; /* our socket */
    unsigned char buf[BUFSIZE]; /* receive buffer */

//    std::fstream data_backup;
};
