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

bool CAnalysisGPS_IMU::check(char buf[])
{
  char temp[200];
  memcpy(temp,buf,200);
  std::string str;
  str=buf;
  if((str.find('*')==std::string::npos)&&(str.find('*') + 3>str.length())) return false;
  int i = 0;
    int result = temp[1];
    for(i = 2; temp[i] != '*'; i++){
        result ^= temp[i];
    }
    
    std::stringstream ss;
    ss<< std::hex << result; // int decimal_value
    std::string res ( ss.str() );
    char cal_check[5];
    strcpy(cal_check,res.c_str());

    for(int j=0;j<res.size();j++){
      if (cal_check[j] >= 'a'&&cal_check[j] <= 'z'){
	      cal_check[j] = cal_check[j] - 32;
	  }
    }//计算得出的校验结果
    char real_check[5];
    memcpy(real_check,buf+i+1,2);
    real_check[2]='\0';//报文中给的校验结果
    
    int check_result=memcmp(cal_check,real_check,2);
    if (check_result==0){
      return true;
    }else{
      return false;
    }
}


void CAnalysisGPS_IMU::Update()
{
  std::string buf_string;
  char buf[4096];
  if (ser.isOpen() && ser.waitReadable()) 
  {
//     ser.waitByteTimes(18);
    
    recvlen = ser.readline(buf_string);
//     stringtochar(buf_string,buf);
    strcpy(buf,buf_string.c_str());
    
  }
  
  bool check_bit=check(buf);
  
//recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
//     UpdateChar(recvlen,buf);
// //     std::cout<<"start update"<<std::endl;
// }
//     
// void CAnalysisGPS_IMU::UpdateChar(int recvlen, unsigned char* buf) {
 if(check_bit){
      char GPSHead[10];

	memset(GPSHead,0,10);
// 	for(int i=0;i<6;i++){
// 	  GPSHead[i]=buf[i];
// 	}
	memcpy(GPSHead,buf,6);
// 	ROS_INFO_STREAM("Read: "<<buf<<"  head:"<<GPSHead<<" length: "<<recvlen);
	if(0 == strcmp(GPSHead,"$GPFPD")||0 == strcmp(GPSHead,"$GNFPD"))
	{
		DataGPFPD(buf,recvlen-1);
		Data_struct.GPFPDflag=true;
// 		std::cout<<"get_gpfpd";
	}
	else if(0 == strcmp(GPSHead,"$GTIMU"))
	{
		DataGTIMU(buf,recvlen-1);
		Data_struct.GTIMUflag=true;
// 		std::cout<<"get_gtimu";
	}
// 	else if(0 == strcmp(GPSHead,"$GPGGA")||0 == strcmp(GPSHead,"$GNGGA"))
// 	{
// 		DataGPGGA(buf,recvlen-1);
// // 		Data_struct.GGAflag=true;
// 	}
// 	else if(0 == strcmp(GPSHead,"$GPRMC")||0 == strcmp(GPSHead,"$GNRMC"))
// 	{
// 		DataGPRMC(buf,recvlen-1);
// 	}
// 	else if(0 == strcmp(GPSHead,"$GPHPD")||0 == strcmp(GPSHead,"$GNHPD"))
// 	{
// 		DataGPHPD(buf,recvlen-1);
// 	}
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
        char tempstatus[5];
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
// 		else if(Data[i] == '*')//status
// 		{
// 		  j = 0;
// 		  GPSCommaNumber++;
// 		  memcpy(Data_struct.status,GPSDataTemp,2*sizeof(char));
// 		  Data_struct.status[2]='\0';
// 		  Data_struct.statusstr=GPSDataTemp;
// 		}
		else
		{
			if(GPSCommaNumber <=17 )
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

