#include "AnalysisGPS_IMU.h"

CAnalysisGPS_IMU::CAnalysisGPS_IMU()
{
  Data_struct.GPFPDflag=false;
  Data_struct.GTIMUflag=false;
}

CAnalysisGPS_IMU::~CAnalysisGPS_IMU()
{

}

bool CAnalysisGPS_IMU::UDPInit(int port)
{
    addrlen = sizeof(myaddr); /* length of addresses */
    /* create a UDP socket */
    if((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
	perror("cannot create socket\n");
	return false;
    }
    /* bind the socket to any valid IP address and a specific port */
    memset((char*)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr =htonl(INADDR_ANY);
    myaddr.sin_port = htons(port);
    if(bind(fd, (sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
	perror("bind failed GPS");
	return false;
    }
   return true;

}

bool CAnalysisGPS_IMU::SerialInit(const std::__cxx11::string& comport, int baudrate, int timeout_ms)
{
  try {
        ser.setPort(comport);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e) {
        perror("opening comport error ! \n");
        return false;
    }
    return true;
}

void CAnalysisGPS_IMU::stringtochar(std::string str,unsigned char result[4096]){
   int i;
//    unsigned char result[4096];
   for( int i=0;i<str.length();i++)
      result[i] = str[i];
   result[i] = '\0';
//    return result;
}


void CAnalysisGPS_IMU::Update()
{
  if (ser.isOpen() && ser.waitReadable()) 
  {
    ser.waitByteTimes(18);
    std::string buf_string ;
    recvlen = ser.readline(buf_string);
    stringtochar(buf_string,buf);
  }
//recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
    UpdateChar(recvlen,buf);
    
}
    
void CAnalysisGPS_IMU::UpdateChar(int recvlen, unsigned char* buf)
{
    if(recvlen >9)
    {
				{

						static char static_cGPSHeadFlag			 = 0;						//接收到头标志
						static char static_cGPSTailFlag			 = 0;						//接收到尾标志
						static char static_cGPSOverFlag			 = 0;						//接收完成标志
						static char static_cGPSRXDataNum		 = 0;						//接收数据计数器
						static char static_cGPSRXAfterStarDataNum = 0;						//接收数据计数器
						static char static_cGPSRXDataChecksum	 = 0;						//数据校验
						static char static_cGPSRXDataCsc[5];								//接收到的校验位
						char		cTempGPSRXDataCsc1			 = 0;						//数据校验高4位
						char		cTempGPSRXDataCsc2			 = 0;						//数据校验低4位
						char		cTemp						 = 0;

						int bufflenth;
						int readlenth;

	for(int i = 0; i<recvlen; i++)
	{

		cTemp = buf[i];
		if('$' == cTemp)
		{
			static_cGPSHeadFlag = 1;
			static_cGPSTailFlag = 0;
			static_cGPSOverFlag = 0;
			static_cGPSRXDataChecksum = 0;
			static_cGPSRXDataNum = 0;
			static_cGPSRXAfterStarDataNum = 0;
			memset(m_cGPSDataFromReceiver,0,100);
			m_cGPSDataFromReceiver[static_cGPSRXDataNum++] = cTemp;

		}
		else
		{
			if(1 == static_cGPSHeadFlag)
			{
				m_cGPSDataFromReceiver[static_cGPSRXDataNum++] = cTemp;
				if('f' == cTemp)
				{
					static_cGPSTailFlag = 1;
				}
				if(1 == static_cGPSTailFlag)
				{
					static_cGPSRXDataCsc[static_cGPSRXAfterStarDataNum++] = cTemp;

					if(3 == static_cGPSRXAfterStarDataNum)
					{
						//分解计算校验值的高低4位
						cTempGPSRXDataCsc1 = ((static_cGPSRXDataChecksum >> 4) & 0x0F);
						cTempGPSRXDataCsc2 = static_cGPSRXDataChecksum & 0x0F;

						if(cTempGPSRXDataCsc1 < 10)
							cTempGPSRXDataCsc1  += '0';
						else cTempGPSRXDataCsc1 += 'A' - 10;

						if(cTempGPSRXDataCsc2 < 10)
							cTempGPSRXDataCsc2  += '0';
						else cTempGPSRXDataCsc2 += 'A' - 10;

						//校验正确
						//if((static_cGPSRXDataCsc[1] == cTempGPSRXDataCsc1)&&
						//	(static_cGPSRXDataCsc[2] == cTempGPSRXDataCsc2))
						{
							char GPSHead[10];

							memset(GPSHead,0,10);
							memcpy(GPSHead,m_cGPSDataFromReceiver,6);
							if(0 == strcmp(GPSHead,"$GPFPD")||0 == strcmp(GPSHead,"$GNFPD"))
							{
								DataGPFPD(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
								Data_struct.GPFPDflag=true;
								std::cout<<"get_gpfpd";
							}
							else if(0 == strcmp(GPSHead,"$GTIMU"))
							{
								DataGTIMU(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
								Data_struct.GTIMUflag=true;
								std::cout<<"get_gtimu";
							}
							else if(0 == strcmp(GPSHead,"$GPGGA")||0 == strcmp(GPSHead,"$GNGGA"))
							{
								DataGPGGA(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
// 								Data_struct.GGAflag=true;
							}
							else if(0 == strcmp(GPSHead,"$GPRMC")||0 == strcmp(GPSHead,"$GNRMC"))
							{
								DataGPRMC(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
							}
							else if(0 == strcmp(GPSHead,"$GPHPD")||0 == strcmp(GPSHead,"$GNHPD"))
							{
								DataGPHPD(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
							}
							else
							{
								static_cGPSOverFlag = 1;
							}
						}
					}
				}
				else
				{
					static_cGPSRXDataChecksum ^= cTemp;
				}


				if(static_cGPSRXDataNum >= 100)
				{
					static_cGPSOverFlag = 1;
				}
			}
		}

		if(1 == static_cGPSOverFlag)
		{
			static_cGPSHeadFlag			 = 0;
			static_cGPSOverFlag			 = 0;
			static_cGPSTailFlag			 = 0;
			static_cGPSRXDataChecksum	 = 0;
			static_cGPSRXDataNum		 = 0;
			static_cGPSRXAfterStarDataNum = 0;
			memset(m_cGPSDataFromReceiver,0,100);
		}
	}
	}
    }
}

void CAnalysisGPS_IMU::DataGPFPD(char Data[],int n)
{

	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	UINT  unTemp = 0;
	int iTemp=0;
//	CString strTemp;

	int i = 0;
	int j = 0;
	int GPSCommaNumber = 0;
	char GPSDataTemp[15];

	memset(GPSDataTemp,0,15);

	while(i++ <= n)
	{
		//找到逗号
		if(Data[i] == ',')		
		{
			j = 0;
			GPSCommaNumber++;
			switch(GPSCommaNumber)
			{
			case 1: // $GPFPD
				memset(GPSDataTemp,0,15);
				break;
			case 2: // GPSWEEK
				iTemp = atoi(GPSDataTemp);
				Data_struct.GPSWeek=iTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 3: // GPSTIME
				dTemp=atof(GPSDataTemp);
				Data_struct.GPSTime=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 4: // heading
				dTemp=atof(GPSDataTemp);
				Data_struct.dheading=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 5: // pitch
				dTemp=atof(GPSDataTemp);
				Data_struct.dpitch=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 6: // roll
				dTemp=atof(GPSDataTemp);
				Data_struct.droll=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 7: // latitude
				dTemp=atof(GPSDataTemp);
				Data_struct.dLatitude=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 8: // longitude
				dTemp=atof(GPSDataTemp);
				Data_struct.dLongitude=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 9: // altitude
				dTemp = atof(GPSDataTemp);
				Data_struct.daltitude=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 10: // VE
				dTemp=atof(GPSDataTemp);
				Data_struct.dVelE=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 11: // VN
				dTemp=atof(GPSDataTemp);
				Data_struct.dVelN=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 12: //VSKY
				dTemp = atof(GPSDataTemp);
				Data_struct.dVelSky=dTemp;
				memset(GPSDataTemp,0,15);
				break;

			default: 
				memset(GPSDataTemp,0,15);
				break;
			}
		}
// 		else if(Data[i] == '*')
// 		{
// 		  j = 0;
// 		  GPSCommaNumber++;
// 		  switch(GPSCommaNumber)
// 		  {
// 		  case 15: // $GPGGA
// 			  dTemp = atof(GPSDataTemp);
// 			  memset(GPSDataTemp,0,15);
// 			  break;
// 		  default: 
//                             memset(GPSDataTemp,0,15);
//                             break;
// 		  }
// 		}
		else
		{
			if(GPSCommaNumber <=15 )
				GPSDataTemp[j++] = Data[i];
		}
	}
}

void CAnalysisGPS_IMU::DataGTIMU(char Data[], int n)
{
  double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	UINT  unTemp = 0;
	long  lTemp = 0;
//	CString strTemp;

	int i = 0;
	int j = 0;
	int GPSCommaNumber = 0;
	char GPSDataTemp[15];

	memset(GPSDataTemp,0,15);

	while(i++ <= n)
	{
		//找到逗号
		if(Data[i] == ',')		
		{
			j = 0;
			GPSCommaNumber++;
			switch(GPSCommaNumber)
			{
			case 1: // $GTIMU
				memset(GPSDataTemp,0,15);
				break;
			case 2: // GPSWEEK
				lTemp = atof(GPSDataTemp);
				memset(GPSDataTemp,0,15);
				break;
			case 3: // GPSTIME
				dTemp=atof(GPSDataTemp);
				memset(GPSDataTemp,0,15);
				break;
			case 4: // GyroX
				dTemp=atof(GPSDataTemp);
				Data_struct.droX=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 5: // GyroY
				dTemp=atof(GPSDataTemp);
				Data_struct.droY=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 6: // GyroZ
				dTemp=atof(GPSDataTemp);
				Data_struct.droZ=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 7: // AccX
				dTemp=atof(GPSDataTemp);
				Data_struct.AccX=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 8: // AccY
				dTemp=atof(GPSDataTemp);
				Data_struct.AccY=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 9: // AccZ
				dTemp = atof(GPSDataTemp);
				Data_struct.AccZ=dTemp;
				memset(GPSDataTemp,0,15);
				break;

			default: 
				break;
			}
		}
		else
		{
			if(GPSCommaNumber <=15 )
				GPSDataTemp[j++] = Data[i];
		}
	}
}

void CAnalysisGPS_IMU::DataGPGGA(char Data[],int n)
{

	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	UINT  unTemp = 0;
	long  lTemp = 0;
//	CString strTemp;

	int i = 0;
	int j = 0;
	int GPSCommaNumber = 0;
	char GPSDataTemp[15];

	memset(GPSDataTemp,0,15);

	while(i++ <= n)
	{

		if(Data[i] == ',')
		{
			j = 0;
			GPSCommaNumber++;
			switch(GPSCommaNumber)
			{
			case 1: // $GPGGA
				memset(GPSDataTemp,0,15);
				break;
			case 9: // HDOP
				dTemp = atof(GPSDataTemp);
				Data_struct.HDOP=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			default: 
				memset(GPSDataTemp,0,15);
				break;
			}
		}
		else
		{
			if(GPSCommaNumber <=15 )
				GPSDataTemp[j++] = Data[i];
		}
	}
}

void CAnalysisGPS_IMU::DataGPHPD(char Data[], int n)
{
	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	int  unTemp = 0;
	long  lTemp = 0;

	int i = 0;
	int j = 0;
	int GPSCommaNumber = 0;
	char GPSDataTemp[15];

	memset(GPSDataTemp,0,15);

	while(i++ <= n)
	{

		if(Data[i] == ',')		//找到逗号
		{
			j = 0;
			GPSCommaNumber++;
			switch(GPSCommaNumber)
			{
			case 1:	// $GPHPD
				memset(GPSDataTemp,0,15);
				break;

			default: break;
			}
		}
		else
		{
			if(GPSCommaNumber <=7 )
				GPSDataTemp[j++] = Data[i];
		}
	}

}

void CAnalysisGPS_IMU::DataGPRMC(char Data[], int n)
{

	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	int  unTemp = 0;
	long  lTemp = 0;
	int  iTemp = 0;
	int i = 0;
	int j = 0;
	int GPSCommaNumber = 0;
	char GPSDataTemp[15];

	memset(GPSDataTemp,0,15);

	while(i++ <= n)
	{

		if(Data[i] == ',')
		{
			j = 0;
			GPSCommaNumber++;
			switch(GPSCommaNumber)
			{
			case 1: // GPRMC
                               memset(GPSDataTemp,0,15);
			      break;
			case 2: // UTC time hhmmss.sss
			    dTemp = atof(GPSDataTemp);
			    Data_struct.UTCtime=dTemp;
			    iTemp = (int)(dTemp * 1000);
			    Data_struct.hour = iTemp / (int)1e7;
			    iTemp = iTemp % (int)1e7;
			    Data_struct.min = iTemp / (int)1e5;
			    iTemp = iTemp % (int)1e5;
			    Data_struct.second = (double)iTemp / 1000;
			    //std::cout<<"GPSData_struct.second  "<< GPSDataTemp<<std::endl;
			    //strTemp.Format(_T("%d:%d:%d"),(lTemp/10000),(lTemp%10000/100),(lTemp%100));
			    //m_ctrlEditUTC.SetWindowTextW(strTemp);
			    memset(GPSDataTemp,0,15);
			    break;
			case 10: // UTC time ddmmyy
			    iTemp = atoi(GPSDataTemp);
			    Data_struct.UTCdate=iTemp;
			    Data_struct.day = iTemp / 10000;
			    iTemp = iTemp % 10000;
			    Data_struct.month = iTemp / 100;
			    iTemp = iTemp % 100;
			    Data_struct.year = iTemp + 2000;
			    memset(GPSDataTemp,0,15);
			    break;

                    default:
                        memset(GPSDataTemp,0,15);
                        break;
			}
		}
		else
		{
			if(GPSCommaNumber <=7 )
				GPSDataTemp[j++] = Data[i];
		}
	}
}

