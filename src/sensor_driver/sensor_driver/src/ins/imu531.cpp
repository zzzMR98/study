#include "imu531.h"
#include <math.h>
#include <stdlib.h>


Imu_531::Imu_531()
{
	fdSerial = 0;
    sum=0;
    check_sum=0;
    memset(Fhead,0,sizeof(char)*3); //What is this?
    FheadFound = false;
    FDataPacketCorr = false;
    FbyteCount = 0;
    Fheading_angle = 0; //What about 拼接变量?，表示有多少个180/(2^15-1)单位
    //Fheading;
    //gps_temp;
    heading_angle = 0;  //转换成角度
    //Rx_FOSNData[46];
    data_backup.open("/home/wby/imu_datasave/data_imu",std::fstream::out |std::fstream::app);
}


Imu_531::~Imu_531()
{
    data_backup.close();
    ser.close();
    close(fd);

}


bool  Imu_531::ImuInit(int port)
{
    close(fd);
    addrlen = sizeof(remaddr); /* length of addresses */
    /* create a UDP socket */
    if((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
	perror("cannot create socket\n");
	return false;
    }

    //int enable = 1;
    //if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    //perror("setsockopt(SO_REUSEADDR) failed");

    /* bind the socket to any valid IP address and a specific port */
    memset((char*)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr =htonl(INADDR_ANY);
    myaddr.sin_port = htons(port);

    if(bind(fd, (sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
	perror("bind failed");
	return false;
    }
    return true;
}

bool Imu_531::SerialInit(const std::string& comport, int baudrate, int timeout_ms, int toUDPPort)
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

//    close(fd);
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(fd < 0)   //创建UDP套接字
    {
        perror("cannot create socket\n");
        return false;
    }

    /* bind the socket to any valid IP address and a specific port */
    memset((char*)&toaddr, 0, sizeof(toaddr));
    toaddr.sin_family = AF_INET;
    toaddr.sin_addr.s_addr =inet_addr("127.0.0.1");
    toaddr.sin_port = htons(toUDPPort);

    return true;
}

void Imu_531::UpdateBySerial()
{
    if (ser.waitReadable()) {
        ser.waitByteTimes(62);
        recvlen = ser.read(buf, 62);

    if(recvlen > 0)
    {
	for(int i = 0; i < recvlen; ++i)
	{
	    ParseImu(buf[i]);
	    //ParseImu_ASCII(buf[i]);
	}

	sendto(fd, buf, recvlen, 0, (struct sockaddr *)&toaddr, sizeof(toaddr));
//	  printf("\n");
	/*Position_Trans_From_ECEF_To_UTM((double)m_sFONSData.dFOSNLat,
		(double)m_sFONSData.dFOSNLng,0,0,
		&m_sFONSData.east,&m_sFONSData.north);*/
    }
  }
}
void Imu_531::Update()
{

    //printf("waiting on port %d\n", PORT);
    recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
  //  printf("received %d bytes\n", recvlen);
    if(recvlen > 0)
    {
	for(int i = 0; i < recvlen; ++i)
	{
	    ParseImu(buf[i]);
	  //  printf("%c",buf[i]);
	    //ParseImu_ASCII(buf[i]);
	}
//	  printf("\n");
	/*Position_Trans_From_ECEF_To_UTM((double)m_sFONSData.dFOSNLat,
		(double)m_sFONSData.dFOSNLng,0,0,
		&m_sFONSData.east,&m_sFONSData.north);*/
    }
}

void Imu_531::ParseImu(char* buf,int len)
{

    if(len > 0)
    {
		for(int i = 0; i < len; ++i)
		{
			ParseImu(buf[i]);
		}
    }
}

void Imu_531::GetImuData()
{
   // data=m_sFONSData;
}
void Imu_531::ParseImu_ASCII(char ch)
{

    static char static_cFONSHeadFlag			 = 0;						//接收到头标志

    static char static_cFONSTailFlag			 = 0;						//接收到尾标志

    static char static_cFONSOverFlag			 = 0;						//接收完成标志

    //static char static_cFONSToFONSFlag        = 0;                       //转发给FONS的gps标志位

    static int static_cFONSRXDataNum		 = 0;						//接收数据计数器

    static char static_cFONSRXAfterStarDataNum = 0;						//接收数据计数器

    static char static_cFONSRXDataChecksum	 = 0;						//数据校验

    static char static_cFONSRXDataCsc[3];								//接收到的校验位

    char		cTempFONSRXDataCsc1			 = 0;						//数据校验高4位

    char		cTempFONSRXDataCsc2			 = 0;						//数据校验低4位

    char		cTemp						 = 0;



    cTemp = ch;

    if('$' == cTemp)

    {

	static_cFONSHeadFlag = 1;

	static_cFONSTailFlag = 0;

	static_cFONSOverFlag = 0;

	static_cFONSRXDataChecksum = cTemp;

	static_cFONSRXDataNum = 0;

	memset(m_cFONSData,0,150);

	memset(m_cFONSTemp,0,150);



    }

    if (1 == static_cFONSHeadFlag)

    {

	if('*' == cTemp)

	{

	    static_cFONSTailFlag = 1;

	}



	if ( 1 == static_cFONSTailFlag)

	{

	    static_cFONSRXDataCsc[static_cFONSRXAfterStarDataNum++] = cTemp;

	    if( 3 == static_cFONSRXAfterStarDataNum)

	    {

		//分解计算校验值的高低4位

		cTempFONSRXDataCsc1 = ((static_cFONSRXDataChecksum>>4) & 0x0F);

		cTempFONSRXDataCsc2 = static_cFONSRXDataChecksum & 0x0F;



		if(cTempFONSRXDataCsc1 <10 )

		    cTempFONSRXDataCsc1  += '0';

		else cTempFONSRXDataCsc1 += 'A' - 10;



		if(cTempFONSRXDataCsc2 <10 )

		    cTempFONSRXDataCsc2  += '0';

		else cTempFONSRXDataCsc2 += 'A' - 10;



		//如果校验正确

		if((static_cFONSRXDataCsc[1] == cTempFONSRXDataCsc1)&&

			(static_cFONSRXDataCsc[2] == cTempFONSRXDataCsc2))

		{

		    char FONSHead[10];

		    char *pStr = NULL;

		    char *next_token = NULL;

		    char seps[] = ",\t\n";

		    double dTemp = 0.0;

		    double dTemp1 = 0.0, dTemp2 = 0.0;

		    unsigned int unTemp = 0;

		    //CString strTemp;



		    int i = 0;

		    int j = 0;

		    int FONSCommaNumber = 0;

		    char FONSDataTemp[15];



		    memset(FONSHead,0,10);

		    memset(FONSDataTemp,0,15);

		    memcpy(m_cFONSTemp, m_cFONSData,150);

		    //pStr = strtok_s(m_cFONSData, seps, &next_token);
		    pStr = strtok_r(m_cFONSData, seps, &next_token);



		    if(pStr)

		    {

			//strcpy_s(FONSHead, 10, pStr);
			strcpy(FONSHead, pStr);

			if( 0 == strcmp(FONSHead,"$STD1AUXA"))

			{





			    while( i++ <= static_cFONSRXDataNum)

			    {

				if( i >= 150)

				    i = 0;

				if(',' == m_cFONSTemp[i])

				{

				    j = 0;

				    FONSCommaNumber++;



				    switch(FONSCommaNumber)

				    {

					case 1:

					    // 包头  STD1AUXA

					    dTemp = atof(FONSDataTemp);



					    memset(FONSDataTemp,0, 15);

					    break;

					case 2:

					    //运行时间

					    dTemp = atof(FONSDataTemp);

					    //m_sFONSData.dRunTime = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 3:

					    //UTC时间

					    dTemp = atof(FONSDataTemp);

					    //m_sFONSData.dUTC = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 4:

					    //航向角

					    dTemp = atof(FONSDataTemp);

					    m_sFONSData.dHeading = dTemp;







					    //											if(m_sFONSData.dHeading<=0)
					    //
					    //												m_sFONSData.dHeading = -m_sFONSData.dHeading;
					    //
					    //											else
					    //
					    //												m_sFONSData.dHeading = 360 -m_sFONSData.dHeading;
					    //
					    //											if( m_sFONSData.dHeading == 360.0)
					    //
					    //												m_sFONSData.dHeading = 0;









					    memset(FONSDataTemp, 0, 15);

					    break;

					case 5:

					    //俯仰角

					    dTemp = atof(FONSDataTemp);

					    m_sFONSData.dPitch = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 6:

					    //横滚角

					    dTemp = atof(FONSDataTemp);

					    m_sFONSData.dRoll = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 7:

					    //纬度

					    dTemp = atof(FONSDataTemp);

					    m_sFONSData.dFOSNLat = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 8:

					    //经度

					    dTemp = atof(FONSDataTemp);

					    m_sFONSData.dFOSNLng = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 9:

					    //高度，如果GPS无效，高度保持不变

					    dTemp = atof(FONSDataTemp);

					    m_sFONSData.dFOSNAltitude = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 10:

					    //东向速度

					    dTemp = atof(FONSDataTemp);

					    m_sFONSData.dVelE = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 11:

					    // 北向速度

					    dTemp = atof(FONSDataTemp);

					    m_sFONSData.dVelN = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 12:

					    // 天向速度

					    dTemp = atof(FONSDataTemp);

					    m_sFONSData.dVelSky = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 13:

					    // 系统温度

					    dTemp = atof(FONSDataTemp);

					    //m_sFONSData.dTemp = dTemp;



					    memset(FONSDataTemp, 0, 15);

					    break;

					case 14:

					    // 可用星数

					    dTemp = atoi(FONSDataTemp);

					    //m_sFONSData.ucStarNum = (unsigned char)dTemp;

					    memset(FONSDataTemp, 0, 15);

					    break;

					case 15:

					    // 流程控制字：

					    dTemp = atoi(FONSDataTemp);

					    memset(FONSDataTemp, 0, 15);







					    break;

					case 16:

					    //GPS 有效性  1s 内出现0x55 表示有效， 其他无效

					    dTemp = atof(FONSDataTemp);

					    //m_sFONSData.ucFONSState = (unsigned char)dTemp;

					    memset(FONSDataTemp,0, 15);





					    break;



					default: break;



				    }

				}

				else

				{

				    char s_temp;

				    char *t;

				    s_temp = m_cFONSTemp[i];

				    t = &s_temp;

				    FONSDataTemp[j++] = m_cFONSTemp[i];



				}

			    }



			    {

				//dTemp = atof(FONSDataTemp);

				//								if(!strncmp(FONSDataTemp , "0x55",4))
				//
				//									m_sFONSData.ucFONSState = 1;
				//
				//								else
				//
				//									m_sFONSData.ucFONSState = 0;
				//
				//
				//
				//								memset(FONSDataTemp,0, 15);





				/*m_sIMUData*/m_sFONSData.ucStaTimes=0;

				//if( m_pNaviLocaWnd->m_bStartNavigation )

				//{

				//	g_CriticalSection.Lock();

				//	g_ucDataSource = 2;

				//	g_CriticalSection.Unlock();



				//	SendMessage(m_pNaviLocaWnd->m_hWnd, WM_SENSORDATA_UPDATED,1,0);

				//}

			    }

			}

			else

			{

			    static_cFONSOverFlag = 1;

			}//$STD1AUXA

		    }

		    else

		    {

			static_cFONSOverFlag =1;



		    }// pstr

		}

		else

		{

		    static_cFONSOverFlag =1;

		}//csc



	    }



	    else if(3 < static_cFONSRXAfterStarDataNum)

	    {

		static_cFONSOverFlag = 1;

	    }



	}

	else

	    // 往m_cFONSData里填一帧数据

	{

	    static_cFONSRXDataChecksum ^= cTemp;

	    m_cFONSData[static_cFONSRXDataNum++] = cTemp;

	    static_cFONSRXAfterStarDataNum = 0;

	}

	//  超过数据长度  要重新填写

	if(static_cFONSRXDataNum >=150)

	{

	    static_cFONSOverFlag = 1;

	}





    }

    // 帧尾  标志

    if(1 == static_cFONSOverFlag)

    {

	static_cFONSHeadFlag   = 1;

	static_cFONSOverFlag   = 0;

	static_cFONSTailFlag   = 0;

	static_cFONSRXDataChecksum = 0;

	static_cFONSRXDataNum = 0;

	static_cFONSRXAfterStarDataNum = 0;

	memset(m_cFONSData, 0, 150);

    }



}

void Imu_531::ParseImu(char ch)	//ch是当前处理的那个字节的内容
{
    if(!FheadFound)	//还没有找到包头
    {
	Fhead[0]=Fhead[1];
	Fhead[1]=Fhead[2];
	Fhead[2] = ch;
	//查找包头
	if(Fhead[0]==0x99 && Fhead[1]==0x66 && Fhead[2]==0x3A)
	{
	    FheadFound = true;	//初始被赋值为0
	    sum++;
	  //  printf("12233");
	}

    }
    else	//找到了包头
    {
	FbyteCount++;	//这个变量是为了：在找到包头后，记录当前录入到包头后第几个字节了。

	//填包头
	Rx_FOSNData[0] = 0x99;
	Rx_FOSNData[1] = 0x66;


	Rx_FOSNData[2] = 0x3A;  //填数据长度，定值Ox2A，后改为3A，3A就是58，这58中不包含包头的最后一位？？？？？？？？？？？？？？？？？？？？？？？？？？？？（1）
	//接着取数据
	if(FbyteCount<59)
	{
	    Rx_FOSNData[FbyteCount+2] = ch; //数据位，FbyteCount从1递增到58。

	}
	else	//此时这一帧数据录入完毕
	{
	    Rx_FOSNData[FbyteCount+2] = ch;  //校验位，第46位，后改为第62位
	    FbyteCount = 0;	//清零了，准备下次用了。
	    FheadFound = false;
	    UucCharOrShortInt check;	//这个变量就是为了和校验用的
	    check.shintData = 0;	//初始赋值为0

	    //*****************和校验开始****************//
	    for(int i=0;i<59; i++)
	    {
		check.shintData +=Rx_FOSNData[2+i] ; //和校验,校验3-45位，后改为校验3-61位
	    }

	    if(check.ucData[0] == Rx_FOSNData[61]) //check.ucData[0]为低位，check.ucData[1]为高位
	    {
		FDataPacketCorr = true;//校验正确
		check_sum++;	//这个变量是为了记录收到的数据正确的帧数
	    }
	    else//校验失败
	    {
		//重新开始找头
		FDataPacketCorr = false;
		Fhead[0] = Rx_FOSNData[60];
		Fhead[1] = Rx_FOSNData[61];
		return;	//return到哪了？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？（2）
	    }
	    //*****************和校验结束****************//

	    //*****************解析数据开始****************//
	    if(FDataPacketCorr)//接收到的数据包, 解包
	    {

		FDataPacketCorr = false;

		/****************** UTC-TIME*******************/
		gps_temp._char[0] = Rx_FOSNData[3];
		gps_temp._char[1] = Rx_FOSNData[4];
		gps_temp._char[2] = Rx_FOSNData[5];
		gps_temp._char[3] = Rx_FOSNData[6];

//		printf("%x", gps_temp._long);
		m_sFONSData.utc_time = gps_temp._long;//02451359

		/*********** yaw ***************///heading
		//取 28位29位，对应航向角，单位 180/(2^15-1)度
		Fheading.ucData[0] = Rx_FOSNData[27]; //低位，实际是第28个字节
		Fheading.ucData[1] = Rx_FOSNData[28]; //高位，实际是第29个字节
		Fheading_angle = Fheading.shintData;  //取出来计算用
		heading_angle = Fheading_angle*180/(pow(2.0,15)-1);//转换成角度
		//			if(heading_angle<=0)	//这是什么情况才会出现？？？？？？？？？？？？？？？？？？？？？？？？？？？（3）
		//			  heading_angle = -heading_angle;
		//			else	//这是什么情况才会出现？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？（4）
		//			  heading_angle = 360 -heading_angle;
		//			if( heading_angle == 360.0)
		//			  heading_angle = 0;
		m_sFONSData.dHeading= heading_angle;


		/************ roll ****************/
		Fheading.ucData[0]= Rx_FOSNData[25]; //low
		Fheading.ucData[1] = Rx_FOSNData[26]; //high
		m_sFONSData.dRoll = Fheading.shintData;
		m_sFONSData.dRoll = m_sFONSData.dRoll*180/(pow(2.0,15)-1);//转换成角度

		/************ pitch ***************/
		Fheading.ucData[0]= Rx_FOSNData[29]; //low
		Fheading.ucData[1] = Rx_FOSNData[30]; //high
		m_sFONSData.dPitch = Fheading.shintData;
		m_sFONSData.dPitch  = m_sFONSData.dPitch*180/(pow(2.0,15)-1);//转换成角度

		/************ X ACC ***************/
		Fheading.ucData[0]= Rx_FOSNData[37]; //low
		Fheading.ucData[1] = Rx_FOSNData[38]; //high
		m_sFONSData.dAccx = Fheading.shintData*100/(pow(2.0,15)-1);	//为什么是100？？？？？？？？？？？？？？？？？？？？？？？（5）

		/************ Y ACC ***************/
		Fheading.ucData[0]= Rx_FOSNData[39]; //low
		Fheading.ucData[1] = Rx_FOSNData[40]; //high
		m_sFONSData.dAccy= Fheading.shintData*100/(pow(2.0,15)-1);

		/************ Z ACC ***************/
		Fheading.ucData[0]= Rx_FOSNData[41]; //low
		Fheading.ucData[1] = Rx_FOSNData[42]; //high
		m_sFONSData.dAccz = Fheading.shintData*100/(pow(2.0,15)-1);

		//Pear
		/************ X AR ***************///X轴角速率（机体轴，前）
		Fheading.ucData[0]= Rx_FOSNData[31]; //low
		Fheading.ucData[1] = Rx_FOSNData[32]; //high
		m_sFONSData.dArx = Fheading.shintData*300/(pow(2.0,15)-1);

		/************ Y AR ***************///Y轴角速率（机体轴，上）
		Fheading.ucData[0]= Rx_FOSNData[33]; //low
		Fheading.ucData[1] = Rx_FOSNData[34]; //high
		m_sFONSData.dAry = Fheading.shintData*300/(pow(2.0,15)-1);

		/************ Z AR ***************///Z轴角速率（机体轴，右）
		Fheading.ucData[0]= Rx_FOSNData[35]; //low
		Fheading.ucData[1] = Rx_FOSNData[36]; //high
		m_sFONSData.dArz = Fheading.shintData*300/(pow(2.0,15)-1);

		/************ NORTH Velocity ***************/
		Fheading.ucData[0]= Rx_FOSNData[19]; //low
		Fheading.ucData[1] = Rx_FOSNData[20]; //high
		m_sFONSData.dVelN = Fheading.shintData*256/(pow(2.0,15)-1);

		/************ SKY Velocity ***************/
		Fheading.ucData[0]= Rx_FOSNData[21]; //low
		Fheading.ucData[1] = Rx_FOSNData[22]; //high
		m_sFONSData.dVelSky = Fheading.shintData*256/(pow(2.0,15)-1);

		/************ EAST Velocity ***************/
		Fheading.ucData[0]= Rx_FOSNData[23]; //low
		Fheading.ucData[1] = Rx_FOSNData[24]; //high
		m_sFONSData.dVelE = Fheading.shintData*256/( pow(2.0,15)-1);

		/************ GPS *****************/
		/****************** GPS-LATITUDE *******************/
		gps_temp._char[0] = Rx_FOSNData[45];
		gps_temp._char[1] = Rx_FOSNData[46];
		gps_temp._char[2] = Rx_FOSNData[47];
		gps_temp._char[3] = Rx_FOSNData[48];

		m_sFONSData.dGPSLat = (double)gps_temp._long*180/(pow(2.0,31)-1);

		/****************** GPS-DOP *******************/
		gps_temp._char[0] = Rx_FOSNData[49];
		gps_temp._char[1] = Rx_FOSNData[50];
		gps_temp._char[2] = Rx_FOSNData[51];
		gps_temp._char[3] = Rx_FOSNData[52];

		m_sFONSData.dGPSDop = (double)gps_temp._long*(pow(2.0,14)-1)/(pow(2.0,31)-1);

		/****************** GPS-LONGITUDE *******************/
		gps_temp._char[0] = Rx_FOSNData[53];
		gps_temp._char[1] = Rx_FOSNData[54];
		gps_temp._char[2] = Rx_FOSNData[55];
		gps_temp._char[3] = Rx_FOSNData[56];

		m_sFONSData.dGPSLng = (double)gps_temp._long*180/(pow(2.0,31)-1);

		/****************** GPS-COUNTER *******************/
		gps_temp._char[0] = Rx_FOSNData[57];
		gps_temp._char[1] = Rx_FOSNData[58];
		gps_temp._char[2] = Rx_FOSNData[59];
		gps_temp._char[3] = Rx_FOSNData[60];

		m_sFONSData.dGPSCounter = (double)gps_temp._long;

		//Pear

		/************ FOSN *****************/
		//latitude
		gps_temp._char[0] = Rx_FOSNData[7];
		gps_temp._char[1] = Rx_FOSNData[8];
		gps_temp._char[2] = Rx_FOSNData[9];
		gps_temp._char[3] = Rx_FOSNData[10];

		m_sFONSData.dFOSNLat = (double)gps_temp._long*180/(pow(2.0,31)-1);	//加这个（double）是啥情况？？？？？？？？？？？？？？？？？？？？？？？（6）

		//altitude
		gps_temp._char[0] = Rx_FOSNData[11];
		gps_temp._char[1] = Rx_FOSNData[12];
		gps_temp._char[2] = Rx_FOSNData[13];
		gps_temp._char[3] = Rx_FOSNData[14];

		m_sFONSData.dFOSNAltitude = (double)gps_temp._long*(pow(2.0,14)-1)/(pow(2.0,31)-1);

		//longitude
		gps_temp._char[0] = Rx_FOSNData[15];
		gps_temp._char[1] = Rx_FOSNData[16];
		gps_temp._char[2] = Rx_FOSNData[17];
		gps_temp._char[3] = Rx_FOSNData[18];

		m_sFONSData.dFOSNLng = (double)gps_temp._long*180/(pow(2.0,31)-1);

		//integrated state
		Fheading.ucData[0]= Rx_FOSNData[43] & 0x07; //low
		Fheading.ucData[1] = Rx_FOSNData[44] & 0x00; //high
		m_sFONSData.dFOSNState = Fheading.shintData;

		m_sFONSData.ucStaTimes=0;	//这里确实不懂了？？？？？？？？？ECU通信状态是什么？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？（7）
		//if( m_pNaviLocaWnd->m_bStartNavigation )
		//{
		//g_CriticalSection.Lock();
		//g_ucDataSource = 2;
		//g_CriticalSection.Unlock();

		//SendMessage(m_pNaviLocaWnd->m_hWnd, WM_SENSORDATA_UPDATED,1,0);
		//}

	    }															//解包结束

	    //？？？以下没看懂，坐标转换的工作细节？坐标转换后干了什么？这个要自己好好学一下！
	    //Position_Trans_From_ECEF_To_UTM((double)m_sFONSData.dFOSNLat,(double)m_sFONSData.dFOSNLng,0,0,&m_sFONSData.east,&m_sFONSData.north);
	    //	    data_backup<<"dHeading:"<<m_sFONSData.dHeading<<"\t"
	    //	    		   <<"dPitch: "<<m_sFONSData.dPitch <<"\t"
	    //
	    //				   <<"dAccx: "<<m_sFONSData.dAccx <<"\t"
	    //				   <<"dAccy: "<<m_sFONSData.dAccy <<"\t"
	    //				   <<"dAccz: "<<m_sFONSData.dAccz <<"\t"
	    //
	    //				   <<"dLat:"<<m_sFONSData.dFOSNLat<<"\t"
	    //				   <<"dLng"<<m_sFONSData.dFOSNLng<<"\t"
	    //
	    //	               <<"ucStaTimes"<<m_sFONSData.ucStaTimes<<"\t"
	    //				   <<"east"<<m_sFONSData.east<<"\t"
	    //				   <<"north"<<m_sFONSData.north<<"\t"<<"\n";

	}
    }
}
void Position_Trans_From_ECEF_To_UTM(double latitude,double longitude,double e0, double n0, double *e, double *n)
{
    double WGS84_ECCENTRICITY = (double)0.0818192;						  // e=0.0818192
    double WGS84_EQUATORIAL_RADIUS = (double)6378.137;					  // a=6378.137
    double k0 = (double)0.9996;

    int Zone = (int)(longitude/6) + 1;									  // ŒÆËãËùÔÚÇøÓò
    int lonBase = Zone*6 - 3;											// ŒÆËãËùÔÚÇøÓòÖÐÑë×ÓÎçÏß

    //ÒÔÏÂ°ŽÕÕ¹«ÊœŒÆËã
    double   vPhi = (double)(1 / sqrt(1-pow(WGS84_ECCENTRICITY * sin(latitude*CV_PI_1/180.0),2)));
    double	A	 = (double)(( longitude - lonBase )*CV_PI_1/180.0 * cos(latitude*CV_PI_1/180.0));		// °ŽÕÕ¶«°ëÇòÀŽËã
    double	sPhi = (double)((1 - pow(WGS84_ECCENTRICITY,2)/4.0 - 3*pow(WGS84_ECCENTRICITY,4)/64.0 - 5*pow(WGS84_ECCENTRICITY,6)/256.0) * latitude*CV_PI_1/180.0
	    - (3*pow(WGS84_ECCENTRICITY,2)/8.0 + 3*pow(WGS84_ECCENTRICITY,4)/32.0 + 45*pow(WGS84_ECCENTRICITY,6)/1024.0) * sin(2*latitude*CV_PI_1/180.0)
	    + (15*pow(WGS84_ECCENTRICITY,4)/256.0 + 45*pow(WGS84_ECCENTRICITY,6)/256.0)* sin(4*latitude*CV_PI_1/180.0)
	    - (35*pow(WGS84_ECCENTRICITY,6)/3072.0) * sin(6*latitude*CV_PI_1/180.0));
    double	T	= (double)(pow(tan(latitude*CV_PI_1/180.0),2));
    double	C	= (double)((pow(WGS84_ECCENTRICITY,2)/(1 - pow(WGS84_ECCENTRICITY,2))) * pow(cos(latitude*CV_PI_1/180.0),2));

    *e = (double)((k0*WGS84_EQUATORIAL_RADIUS*vPhi*(A + (1 - T + C)*pow(A,3)/6.0
		    + (5 - 18*T + pow(T,2))*pow(A,5)/120.0))*1000 - e0);
    *n = (double)((k0*WGS84_EQUATORIAL_RADIUS*(sPhi + vPhi * tan(latitude*CV_PI_1/180.0)*(pow(A,2)/2
			+ (5 - T + 9*C + 4*C*C)*pow(A,4)/24.0 + (61 - 58*T + T*T)*pow(A,6)/720.0)))*1000 - n0);

}
