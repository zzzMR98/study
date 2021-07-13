#include "AnalysisINS_OxT.h"
#include <stdlib.h>

CAnalysisINS_OxT::CAnalysisINS_OxT()
{

}

CAnalysisINS_OxT::~CAnalysisINS_OxT()
{
//	data_backup.close();
}

bool CAnalysisINS_OxT::Init(int port)
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
	perror("bind failed INs");
	return false;
    }
   return true;
  }




void CAnalysisINS_OxT::Update()
{
	   recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&myaddr, &addrlen);
	//	if ( (buf[recvlen-2]=='$')&&(buf[recvlen-1]=='H')&&(buf[2-2]=='E')&&(buf[3-2]=='H')&&(buf[4-2]=='D')&&(buf[5-2]=='T'))
	    {
		   IMU_DataProcFromOxTS(buf,recvlen);
	    }
}

void CAnalysisINS_OxT::ParseImu(char* ch,int len)
{
	IMU_DataProcFromOxTS((unsigned char*)ch,len);
}

void CAnalysisINS_OxT::IMU_DataProcFromOxTS(unsigned char* data,int count)
{
	int bufflenth;
	int readlenth;
	unsigned char cTemp= 0;
	unsigned char checksum1=0;
	unsigned char checksum2=0;
	unsigned char ucTailNo=0;
	unsigned char ucHeadNo=0;
	unsigned char ucTailFlag = 0;
	unsigned char ucHeadFlag = 0;
	unsigned char* ReadBuff;

	ReadBuff=data;
	readlenth = count;
		
	if (readlenth>=72)
	{

		for(int i = 0; i<readlenth; i++) //网口直接输出 E7为包头
		{
			if(0xE7==ReadBuff[i])
			{
				ucHeadNo=i;
				ucHeadFlag=1;
			}
			if (ucHeadFlag==1)
			{
				ucHeadFlag=0;
				for (int j =ucHeadNo+1; j < (ucHeadNo + 22); j++)
					checksum1 += ReadBuff[j];
				for (int k =ucHeadNo+1; k < (ucHeadNo + 71); k++)
					checksum2 += ReadBuff[k];
				if ((checksum1==ReadBuff[ucHeadNo + 22])&(checksum2==ReadBuff[ucHeadNo + 71]))
				{
					int temp_acc_x = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 3]);
					int temp_acc_y = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 6]);
					int temp_acc_z = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 9]);
					int temp_angrate_x = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 12]);
					int temp_angrate_y = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 15]);
					int temp_angrate_z = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 18]);	
					int temp_v_n = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 43]);
					int temp_v_e = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 46]);
					int temp_v_d = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 49]);
					int temp_orien_h=cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 52]);
					int temp_orien_p=cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 55]);
					int temp_orien_r=cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 58]);

					//加速度和角速度
					if (temp_acc_x!=-8388608)   {INSData_struct.dAccx=temp_acc_x*0.0001;}
					if (temp_acc_y!=-8388608)   {INSData_struct.dAccy=temp_acc_y*0.0001;}
					if (temp_acc_z!=-8388608)   {INSData_struct.dAccz=temp_acc_z*0.0001;}
/*					if (temp_angrate_x!=-8388608)   {m_VehicleState.angrate_x=temp_angrate_x*0.00001*RAD2DEG;}
					if (temp_angrate_y!=-8388608)   {m_VehicleState.angrate_y=temp_angrate_y*0.00001*RAD2DEG;}
					if (temp_angrate_z!=-8388608)   {m_VehicleState.angrate_z=temp_angrate_z*0.00001*RAD2DEG;}*/	

					if (temp_angrate_x!=-8388608)   {INSData_struct.dGyroX=temp_angrate_x*0.00001*RAD2DEG;}
					if (temp_angrate_y!=-8388608)   {INSData_struct.dGyroY=temp_angrate_y*0.00001*RAD2DEG;}
					if (temp_angrate_z!=-8388608)   {INSData_struct.dGyroZ=temp_angrate_z*0.00001*RAD2DEG;}

					//经纬度和海拔
					INSData_struct.dLat=cast_8_byte_to_double(&ReadBuff[ucHeadNo + 23]) * RAD2DEG;
					INSData_struct.dLng =cast_8_byte_to_double(&ReadBuff[ucHeadNo + 31]) * RAD2DEG;
					INSData_struct.dAltitude=cast_4_byte_to_float(&ReadBuff[ucHeadNo+39]);
					//m_VehicleState.pos_alt=cast_4_byte_to_float(&ReadBuff[ucTailNo-33]) * RAD2DEG;

					//速度
					if (temp_v_n != -8388608) INSData_struct.dNorthV=temp_v_n *0.0001;
					if (temp_v_e != -8388608) INSData_struct.dEastV=temp_v_e *0.0001;
					if (temp_v_d != -8388608) INSData_struct.dAltitudeV=temp_v_d *0.0001;
					//m_VehicleState.speed=sqrt(pow(m_VehicleState.v_n,2)+pow(m_VehicleState.v_e,2)+pow(m_VehicleState.v_d,2));

					//航向角，俯仰角，侧倾角
					if (temp_orien_h != -8388608) INSData_struct.dHeading=temp_orien_h *0.000001* RAD2DEG;
					if (INSData_struct.dHeading<0)
					{
						INSData_struct.dHeading=INSData_struct.dHeading+360;
					}
					if (INSData_struct.dHeading>=360)
					{
						INSData_struct.dHeading=INSData_struct.dHeading-360;
					}
					if (temp_orien_p != -8388608) INSData_struct.dPitch=temp_orien_p *0.000001* RAD2DEG;
					if (temp_orien_r != -8388608) INSData_struct.dRoll=temp_orien_r*0.000001* RAD2DEG;

					INSData_struct.ucStaTimes=0;

				}
			}

		}


//		for(int i = 0; i<readlenth; i++)   //串口输出，E7为包尾
//		{
//			if(0xE7==ReadBuff[i])
//			{
//				ucTailNo=i;
//				ucTailFlag=1;
//			}
//			if (ucTailFlag==1)
//			{
//				ucTailFlag=0;
//				for (int j =(ucTailNo-71); j < (ucTailNo - 50); j++)
//					checksum1 += ReadBuff[j];
//				for (int k =(ucTailNo-71); k < (ucTailNo - 1); k++)
//					checksum2 += ReadBuff[k];
//				if ((checksum1==ReadBuff[ucTailNo-50])&(checksum2==ReadBuff[ucTailNo-1]))
//				{
//					int temp_acc_x = cast_3_byte_to_int32(&ReadBuff[ucTailNo-69]);
//					int temp_acc_y = cast_3_byte_to_int32(&ReadBuff[ucTailNo-66]);
//					int temp_acc_z = cast_3_byte_to_int32(&ReadBuff[ucTailNo-63]);
//					int temp_angrate_x = cast_3_byte_to_int32(&ReadBuff[ucTailNo-60]);
//					int temp_angrate_y = cast_3_byte_to_int32(&ReadBuff[ucTailNo-57]);
//					int temp_angrate_z = cast_3_byte_to_int32(&ReadBuff[ucTailNo-54]);	
//					int temp_v_n = cast_3_byte_to_int32(&ReadBuff[ucTailNo-29]);
//					int temp_v_e = cast_3_byte_to_int32(&ReadBuff[ucTailNo-26]);
//					int temp_v_d = cast_3_byte_to_int32(&ReadBuff[ucTailNo-23]);
//					int temp_orien_h=cast_3_byte_to_int32(&ReadBuff[ucTailNo-20]);
//					int temp_orien_p=cast_3_byte_to_int32(&ReadBuff[ucTailNo-17]);
//					int temp_orien_r=cast_3_byte_to_int32(&ReadBuff[ucTailNo-14]);
//
//					//加速度和角速度
//					if (temp_acc_x!=-8388608)   {INSData_struct.dAccx=temp_acc_x*0.0001;}
//					if (temp_acc_y!=-8388608)   {INSData_struct.dAccy=temp_acc_y*0.0001;}
//					if (temp_acc_z!=-8388608)   {INSData_struct.dAccz=temp_acc_z*0.0001;}
///*					if (temp_angrate_x!=-8388608)   {m_VehicleState.angrate_x=temp_angrate_x*0.00001*RAD2DEG;}
//					if (temp_angrate_y!=-8388608)   {m_VehicleState.angrate_y=temp_angrate_y*0.00001*RAD2DEG;}
//					if (temp_angrate_z!=-8388608)   {m_VehicleState.angrate_z=temp_angrate_z*0.00001*RAD2DEG;}*/
//
//					//经纬度和海拔
//					INSData_struct.dLat=cast_8_byte_to_double(&ReadBuff[ucTailNo-49]) * RAD2DEG;
//					INSData_struct.dLng =cast_8_byte_to_double(&ReadBuff[ucTailNo-41]) * RAD2DEG;
//					//m_VehicleState.pos_alt=cast_4_byte_to_float(&ReadBuff[ucTailNo-33]) * RAD2DEG;
//
//					////速度
//					//if (temp_v_n != -8388608) m_VehicleState.v_n=temp_v_n *0.0001;
//					//if (temp_v_e != -8388608) m_VehicleState.v_e=temp_v_e *0.0001;
//					//if (temp_v_d != -8388608) m_VehicleState.v_d=temp_v_d *0.0001;
//					//m_VehicleState.speed=sqrt(pow(m_VehicleState.v_n,2)+pow(m_VehicleState.v_e,2)+pow(m_VehicleState.v_d,2));
//
//					//航向角，俯仰角，侧倾角
//					if (temp_orien_h != -8388608) INSData_struct.dHeading=temp_orien_h *0.000001* RAD2DEG;
//
//					//INSData_struct.dHeading=INSData_struct.dHeading-180;  //惯导反着安装，所以要-180
//					if (INSData_struct.dHeading<0)
//					{
//						INSData_struct.dHeading=INSData_struct.dHeading+360;
//					}
//					if (INSData_struct.dHeading>360)
//					{
//						INSData_struct.dHeading=INSData_struct.dHeading-360;
//					}
//					if (temp_orien_p != -8388608) INSData_struct.dPitch=temp_orien_p *0.000001* RAD2DEG;
//					if (temp_orien_r != -8388608) INSData_struct.dRoll=temp_orien_r*0.000001* RAD2DEG;
//
//					INSData_struct.ucStaTimes=0;
//					if( m_pNaviLocaWnd->m_bStartNavigation )
//					{
//						g_CriticalSection.Lock();
//						g_ucDataSource = 2;
//						g_CriticalSection.Unlock();
//
//						SendMessage(m_pNaviLocaWnd->m_hWnd, WM_SENSORDATA_UPDATED,1,0);
//					}
//				}
//			}
//
//		}

//		for(int i = 0; i<readlenth; i++)
//		{
//			if(0xE7==ReadBuff[i])
//			{
//				ucHeadNo=i;
//				ucHeadFlag=1;
//			}
//			if (ucHeadFlag==1)
//			{
//				ucHeadFlag=0;
//				for (int j =ucHeadNo; j < (ucHeadNo + 22); j++)
//					checksum1 += ReadBuff[j];
//				for (int k =ucHeadNo; k < (ucHeadNo + 71); k++)
//					checksum2 += ReadBuff[k];
//				if ((checksum1==ReadBuff[ucHeadNo + 22])&(checksum2==ReadBuff[ucHeadNo + 71]))
//				{
//					int temp_acc_x = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 3]);
//					int temp_acc_y = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 6]);
//					int temp_acc_z = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 9]);
//					int temp_angrate_x = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 12]);
//					int temp_angrate_y = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 15]);
//					int temp_angrate_z = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 18]);	
//					int temp_v_n = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 43]);
//					int temp_v_e = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 46]);
//					int temp_v_d = cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 49]);
//					int temp_orien_h=cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 52]);
//					int temp_orien_p=cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 55]);
//					int temp_orien_r=cast_3_byte_to_int32(&ReadBuff[ucHeadNo + 58]);
//
//					//加速度和角速度
//					if (temp_acc_x!=-8388608)   {INSData_struct.dAccx=temp_acc_x*0.0001;}
//					if (temp_acc_y!=-8388608)   {INSData_struct.dAccy=temp_acc_y*0.0001;}
//					if (temp_acc_z!=-8388608)   {INSData_struct.dAccz=temp_acc_z*0.0001;}
///*					if (temp_angrate_x!=-8388608)   {m_VehicleState.angrate_x=temp_angrate_x*0.00001*RAD2DEG;}
//					if (temp_angrate_y!=-8388608)   {m_VehicleState.angrate_y=temp_angrate_y*0.00001*RAD2DEG;}
//					if (temp_angrate_z!=-8388608)   {m_VehicleState.angrate_z=temp_angrate_z*0.00001*RAD2DEG;}*/	
//
//					//经纬度和海拔
//					INSData_struct.dLat=cast_8_byte_to_double(&ReadBuff[ucHeadNo + 23]) * RAD2DEG;
//					INSData_struct.dLng =cast_8_byte_to_double(&ReadBuff[ucHeadNo + 31]) * RAD2DEG;
//					//m_VehicleState.pos_alt=cast_4_byte_to_float(&ReadBuff[ucTailNo-33]) * RAD2DEG;
//
//					////速度
//					//if (temp_v_n != -8388608) m_VehicleState.v_n=temp_v_n *0.0001;
//					//if (temp_v_e != -8388608) m_VehicleState.v_e=temp_v_e *0.0001;
//					//if (temp_v_d != -8388608) m_VehicleState.v_d=temp_v_d *0.0001;
//					//m_VehicleState.speed=sqrt(pow(m_VehicleState.v_n,2)+pow(m_VehicleState.v_e,2)+pow(m_VehicleState.v_d,2));
//
//					//航向角，俯仰角，侧倾角
//					if (temp_orien_h != -8388608) INSData_struct.dHeading=temp_orien_h *0.000001* RAD2DEG;
//					if (INSData_struct.dHeading<0)
//					{
//						INSData_struct.dHeading=INSData_struct.dHeading+360;
//					}
//					if (INSData_struct.dHeading>=360)
//					{
//						INSData_struct.dHeading=INSData_struct.dHeading-360;
//					}
//					if (temp_orien_p != -8388608) INSData_struct.dPitch=temp_orien_p *0.000001* RAD2DEG;
//					if (temp_orien_r != -8388608) INSData_struct.dRoll=temp_orien_r*0.000001* RAD2DEG;
//
//					INSData_struct.ucStaTimes=0;
//					if( m_pNaviLocaWnd->m_bStartNavigation )
//					{
//						g_CriticalSection.Lock();
//						g_ucDataSource = 2;
//						g_CriticalSection.Unlock();
//
//						SendMessage(m_pNaviLocaWnd->m_hWnd, WM_SENSORDATA_UPDATED,1,0);
//					}
//				}
//			}
//
//		}
	}

}

//数据类型转换
unsigned int CAnalysisINS_OxT:: cast_2_byte_to_uint32(unsigned char* b)
{
	union { unsigned int x; unsigned char c[4]; } u;
	u.c[1] = b[0];
	u.c[0] = b[1];
	u.c[2] = 0;
	u.c[3] = 0;
	return u.x;
}
unsigned short CAnalysisINS_OxT:: cast_2_byte_to_ushort(unsigned char* b)
{
	union { unsigned short x; unsigned char c[2]; } u;
	u.c[1] = b[0];
	u.c[0] = b[1];
	return u.x;
}
int CAnalysisINS_OxT::cast_3_byte_to_int32(unsigned char*  b)
{
	union { int x; unsigned char c[4]; } u;
	u.c[1] = b[0];
	u.c[2] = b[1];
	u.c[3] = b[2];
	return u.x >> 8;
}
double CAnalysisINS_OxT:: cast_8_byte_to_double(unsigned char* b)
{
	union { double x; unsigned char c[8]; } u;
	u.c[0] = b[0];
	u.c[1] = b[1];
	u.c[2] = b[2];
	u.c[3] = b[3];
	u.c[4] = b[4];
	u.c[5] = b[5];
	u.c[6] = b[6];
	u.c[7] = b[7];
	return u.x;
}
float CAnalysisINS_OxT::cast_4_byte_to_float(unsigned char *b)
{
	union { float x;unsigned char c[4]; } u;
	u.c[0] = b[0];
	u.c[1] = b[1];
	u.c[2] = b[2];
	u.c[3] = b[3];
	return u.x;
}
