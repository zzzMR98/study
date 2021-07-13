#include <dlfcn.h>
#include "AnalysisECU.h"

CAnalysisECU::CAnalysisECU()
{
    //霍钊添加
    CANFound = -1;
}

CAnalysisECU::~CAnalysisECU()
{
//	data_backup.close();
}

bool  CAnalysisECU::Init(int port)
{
    addrlen = sizeof(myaddr); /* length of addresses */
    /* create a UDP socket */
    if((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
	perror("cannot create ecu socket\n");
	return false;
    }

    //int enable = 1;
    //if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    //perror("setsockopt(SO_REUSEADDR) failed");

    /* bind the socket to any valid IP address and a specific port */
    memset((char*)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
   myaddr.sin_addr.s_addr =htonl(INADDR_ANY);
	//myaddr.sin_addr.s_addr =htons("192.168.0.254");
    myaddr.sin_port = htons(port);

    if(bind(fd, (sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
		perror("ECU input  bind failed");
		 return false;
    }

   return true;
}

void CAnalysisECU::ParseEcu(char* buf,int len)
{
	ECU_DataProcFromVehicle((unsigned char*)(buf));
}

void CAnalysisECU::Update()
{
	//    printf("waiting on port %d\n", ECU_IN_PORT);
	    recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
#ifdef TOYOTAPLATFORM
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
	recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
	recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
	recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
	recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
	recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);

#endif

#ifdef CATERPILLARCAR6//6吨
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
#endif



	  //  printf("received navigation ECU %d bytes\n", recvlen);
#ifdef TANGPLATFORM_76GF6

		if((recvlen == 50))
		{
			ECU_DataProcFromVehicle(buf);
		}
#endif


#ifdef TOYOTAPLATFORM

	//	if((recvlen == 50))
		{
			ECU_DataProcFromVehicle(buf);
//            FILE *fp11=fopen("/home/stefan/IV_RCS/state.txt","a");
//            fprintf(fp11,"%x	%x	%x	%x\n",
//                    buf[0],
//                    buf[1],
//                    buf[2],
//                    buf[3]);
//            fclose(fp11);
		}
#endif


}

double CAnalysisECU::convert_ctrlvalue2steeringangle(int ctrlvalue, bool direction, double steeringratio_l_1, double steeringratio_r_1)
{
	double steeringangle=0.0;
#ifdef TOYOTAPLATFORM
	//double TOYOTA_Steering_ratio_L = 2000 / 16.04 ;
	//double TOYOTA_Steering_ratio_R = 2000 / 16.22 ;

	if(!direction)//��ת
		steeringangle = (double) ctrlvalue / steeringratio_l_1;
	else//��ת
		steeringangle = (double) - ctrlvalue / steeringratio_r_1;

#endif


#ifdef BYDRAY
	//double BYD_Steering_ratio_L = 2698 / 13.96 ;
	//double BYD_Steering_ratio_R = 2545 / 13.25 ;

	steeringangle = (double)(ctrlvalue - 7900) ;
	if(steeringangle > 0 )//��ת
		steeringangle = steeringangle / steeringratio_l_1;
	else
		steeringangle = steeringangle / steeringratio_r_1;
#endif

#ifdef TANGPLATFORM_76GF6 //2016-08-29 ltf
	//double BYD_Steering_ratio_L = 2698 / 13.96 ;
	//double BYD_Steering_ratio_R = 2545 / 13.25 ;

	steeringangle = (double)(ctrlvalue - 7800) ;
	if(steeringangle > 0 )//��ת
		steeringangle = steeringangle / steeringratio_l_1;
	else
		steeringangle = steeringangle / steeringratio_r_1;
#endif
#ifdef FOTONBUS//福田客车
	steeringangle = (double)(ctrlvalue - 7800) ;
	if(steeringangle > 0)
		steeringangle = steeringangle / steeringratio_l_1;
	else
		steeringangle = steeringangle / steeringratio_r_1;
#endif

	return steeringangle;
}

void CAnalysisECU::ECU_DataProcFromVehicle(unsigned char* data)
{
#ifdef TANGPLATFORM_76GF6//如果是比亚迪唐平台，需要修改，关海杰。



	unsigned char static_ucECURXDataChecksum = 0;
	unsigned char datacopy[50];
	memcpy(datacopy, data, 50);

	unsigned char tmp1,tmp2;
	data[0]=datacopy[48];
	data[1]=datacopy[49];

	for(int i=0; i < 48; i++)
	{
		data[2+i]=datacopy[i];

	}
	//校验
	if( (0xAA == data[0] )&&( 0x55 == data[1]))
	{

		//方向盘角度
		int tmp=((data[3]&0xFC)/4+data[4]*64+(data[5]&0x03)*4096);

		ECUData_struct.fFLRWheelAverAngle = -convert_ctrlvalue2steeringangle( tmp , 0 , steeringratio_l, steeringratio_r);
		tmp1=data[13];//车速
		tmp2=data[14]&0x0F;
		double PH=0.06875*(tmp2*256+tmp1);
		ECUData_struct.fForwardVel = (double)PH / 3.6;

		//if((data[2]&0xC0)==0x40)//若EPS反馈处于"正常驾驶模式",则状态切换到"人工驾驶模式",qjy,20160813
		//	m_sendcmdsocket.RunningStatus=3;


		////左轮速
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+16, 4);
		//tmp = atoi(cDataTemp);

		////右轮速
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+21, 4);
		//tmp = atoi(cDataTemp);

		//档位
		ECUData_struct.f_shift=  (data[2]&0x3C)/4;//档位
	//	cout<<"档位 ";
		//cout<<int(VehicleStateFromECU.f_shift);
	//	VehicleStateFromECU.f_shift= 5;//具体档位暂无。

		//m_sVehicleStateToNet.longitutdectrl_enabled = true;//纵向控制使能
		//m_sVehicleStateToNet.lateralctrl_enabled =true;//横向控制使能


		//		//左灯
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+32, 1);
		//m_sECUData.ucLeftLamp = atoi(cDataTemp);

		////右灯
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+34, 1);
		//m_sECUData.ucRightLamp = atoi(cDataTemp);

		////纵向控制模式
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+36, 1);
		//m_sECUData.longitutdectrl_enabled = atoi(cDataTemp);
		//
		////横向控制模式
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+38, 1);
		//m_sECUData.lateralctrl_enabled = atoi(cDataTemp);

		//油压1

		ECUData_struct.pressure_back = 0.1*data[20];//left 制动压力

		ECUData_struct.petral_pressure = 0.1*data[21];//Right

		////制动踏板信号
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+40, 1);
		//tmp = atoi(cDataTemp);
		//m_sVehicleStateToNet.brake_enabled = (tmp ==0)? false : true;
		if((ECUData_struct.pressure_back>0.01)||(ECUData_struct.petral_pressure>0.01))
			ECUData_struct.brake_enabled=1;
		else
			ECUData_struct.brake_enabled=0;

		////具体档位
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+28, 1);
		//tmp = atoi(cDataTemp);
		//m_sVehicleStateToNet.f_shift1 = tmp;


		////紧急制动
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+30, 1);
		//m_sVehicleStateToNet.f_estop = atoi(cDataTemp);

		////左灯
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+32, 1);
		//m_sVehicleStateToNet.f_leftlamp = atoi(cDataTemp);

		////右灯
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+34, 1);
		//m_sVehicleStateToNet.f_rightlamp = atoi(cDataTemp);

		////纵向控制模式
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+36, 1);
		//tmp = atoi(cDataTemp);
		//m_sVehicleStateToNet.longitutdectrl_enabled = (tmp ==0)? false : true;
		//
		////横向控制模式
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+38, 1);
		//tmp = atoi(cDataTemp);
		//m_sVehicleStateToNet.lateralctrl_enabled = (tmp ==0)? false : true;

		////制动踏板信号
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+40, 1);
		//tmp = atoi(cDataTemp);
		//m_sVehicleStateToNet.brake_enabled = (tmp ==0)? false : true;

		////GPS方向
		//memset(cDataTemp, 0, 30);
		//memcpy(cDataTemp, data+42, 3);
		//m_sVehicleStateToNet.fHeading = (double)atoi(cDataTemp) * 3.1415926535897932384626433832795/180;
		//m_sVehicleStateToNet.fTheta=pi/2-m_sVehicleStateToNet.fHeading;



		//*************************GHJ20171012增加车辆底层反馈信息*************************//
		ECUData_struct.FrontLeftWheelSpeedStatus=data[28]&0x01;  //0有效 1无效

		ECUData_struct.FrontLeftWheelSpeed=(((unsigned int)data[29]&0x1F)*128+((data[28]>>1)&0x7F))*0.06875;

		ECUData_struct.FrontRightWheelSpeedStatus=(data[29]>>5)&0x01;  //0有效 1无效

		ECUData_struct.FrontRightWheelSpeed=(((unsigned int)data[31]&0x03)*1024+data[30]*4+((data[29]>>6)&0x03))*0.06875;

		ECUData_struct.BackLeftWheelSpeedStatus=(data[31]>>2)&0x01;  //0有效 1无效

		ECUData_struct.BackLeftWheelSpeed=(((unsigned int)data[32]&0x7F)*32+((data[31]>>3)&0x1F))*0.06875;

		ECUData_struct.BackRightWheelSpeedStatus=(data[32]>>7)&0x01;  //0有效 1无效

		ECUData_struct.BackRightWheelSpeed=(((unsigned int)data[34]&0x0F)*256+data[33])*0.06875;

		ECUData_struct.Yaw_Rate=(((unsigned int)data[36]&0x0F)*256+data[35])*0.002132603-2.0943;

		ECUData_struct.Yaw_Rate_Offset=((unsigned int)data[37]*16+(data[36]>>4)&0x0F)*0.002132603-0.13;

		ECUData_struct.Yaw_Rate_Status=(data[38]>>7)&0x01;  //0无效 1有效

		ECUData_struct.AX=(((unsigned int)data[40]&0x0F)*256+data[39])*0.027126736-21.593;

		ECUData_struct.AX_Offset=((unsigned int)data[41]*16+((data[40]>>4)&0x0F))*0.027126736-21.593;

		ECUData_struct.AX_Status=(data[12]>>1)&0x01;

		ECUData_struct.AY=(((unsigned int)data[10]&0x0F)*256+data[9])*0.027126736-21.593;

		ECUData_struct.AY_Offset=((unsigned int)data[11]*16+((data[10]>>4)&0x0F))*0.027126736-21.593;

		ECUData_struct.AY_Status=(data[12]>>1)&0x01;
		//GHJ20171013测试使用，测试后删除
//		FILE *fp = fopen("../IV_RCS/src/GetECUData/ECU-Data.txt", "a");
//		if(fileflag==0)
//		{
//			fprintf(fp,"%s\t%s\t%s\t%s\t%s\t%s\t%s\t\n",FrontLeftWheelSpeed,FrontRightWheelSpeed,BackLeftWheelSpeed,BackRightWheelSpeed,Yaw_Rate,AX,AY);
//			fileflag++;
//		}
//		fprintf(fp, "%.5f\t%.5f\t%.5f\t%.5f\t\n%.5f\t%.5f\t%.5f\t", FrontLeftWheelSpeed,FrontRightWheelSpeed,BackLeftWheelSpeed,BackRightWheelSpeed,Yaw_Rate,AX,AY);
//		fclose(fp);
	}
#endif

#ifdef BYDRAY

	unsigned char static_ucECURXDataChecksum = 0;
	for(int i=0; i < 47 + 18; i++)
	{
		static_ucECURXDataChecksum ^= data[i];
	}

	unsigned char cTemp1 = ((static_ucECURXDataChecksum >> 4) & 0x0F);
	unsigned char cTemp2 = (static_ucECURXDataChecksum & 0x0F);

	if(cTemp1 < 10)
		cTemp1 += '0';
	else cTemp1 += 'A' - 10;

	if(cTemp2 < 10)
		cTemp2 += '0';
	else cTemp2 += 'A' - 10;

	//校验
	if( cTemp1 == data[47+18] && cTemp2 == data[48+18])
	{
		//转向电机位置
		char cDataTemp[30];
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+5, 5);
		int tmp = atoi(cDataTemp);
		ECUData_struct.fFLRWheelAverAngle = convert_ctrlvalue2steeringangle( tmp , 0 , steeringratio_l, steeringratio_r);

		//车速
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+11, 4);
		tmp = atoi(cDataTemp);
		ECUData_struct.fForwardVel = (double)tmp / 100;

		//左后轮速
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+16, 4);
		tmp = atoi(cDataTemp);
		ECUData_struct.BackLeftWheelSpeed=tmp;

		//右后轮速
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+21, 4);
		tmp = atoi(cDataTemp);
		ECUData_struct.BackRightWheelSpeed=tmp;

		//档位
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+26, 1);
		tmp = atoi(cDataTemp);
		ECUData_struct.f_shift = tmp;

		//具体档位
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+28, 1);
		tmp = atoi(cDataTemp);
		ECUData_struct.f_shift1 = tmp;


		//紧急制动
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+30, 1);
		ECUData_struct.f_estop = atoi(cDataTemp);

		//左灯
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+32, 1);
		ECUData_struct.f_leftlamp = atoi(cDataTemp);

		//右灯
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+34, 1);
		ECUData_struct.f_rightlamp = atoi(cDataTemp);

		//纵向控制模式
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+36, 1);
		tmp = atoi(cDataTemp);
		ECUData_struct.longitutdectrl_enabled = (tmp ==0)? false : true;

		//横向控制模式
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+38, 1);
		tmp = atoi(cDataTemp);
		ECUData_struct.lateralctrl_enabled = (tmp ==0)? false : true;

		//制动踏板信号
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+40, 1);
		tmp = atoi(cDataTemp);
		ECUData_struct.brake_enabled = (tmp ==0)? false : true;

		//GPS方向
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+42, 3);
		double heading = (double)atoi(cDataTemp) * 3.1415926535897932384626433832795/180;
	//	ECUData_struct.Odometer_theta = pi/2 - heading;

		//发动机转速
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+46, 4);
		ECUData_struct.EnginRate = atoi(cDataTemp);

		//油压1
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+51, 3);
		ECUData_struct.pressure_back = atoi(cDataTemp);

		//油压2
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+55, 3);
		ECUData_struct.petral_pressure = atoi(cDataTemp);

		//制动故障码
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+59, 2);

		//过热保护
		memset(cDataTemp, 0, 30);
		memcpy(cDataTemp, data+62, 1);
	}
#endif
#ifdef TOYOTAPLATFORM //丰田平台修改，霍钊
//    ofstream ofile;
//    ofile.open("/home/stefan/IV_RCS/data.txt");
//    ofile<<data[0]<<'\t'<<data[1]<<'\t'<<data[2]<<'\t'<<data[3]<<'\t'<<data[4]<<'\n'<<endl;
//    ofile.close();
//    FILE *fp11=fopen("/home/stefan/IV_RCS/state.txt","a");
//    fprintf(fp11,"%x	%x	%x	%x\n",
//            data[0],
//            data[1],
//            data[2],
//            data[3]);
//    fclose(fp11);

    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x1a) && (data[4]==0x01))
    {
        CANFound = 1;
    }
    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x1a) && (data[4]==0x02))
    {
        CANFound = 2;
    }
    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x1a) && (data[4]==0x03))
    {
        CANFound = 3;
    }
    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x1a) && (data[4]==0x04))
    {
        CANFound = 4;
    }
    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x2a) && (data[4]==0x01))
    {
        CANFound = 5;
    }
    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x2a) && (data[4]==0x02))
    {
        CANFound = 6;
    }
    if((data[0]==0x88)&& (data[1]==0x00) && (data[2]==0x00) && (data[3]==0x3a) && (data[4]==0x01))
    {
        CANFound = 7;
    }

    unsigned char tmp;
    unsigned char tmp1;
    unsigned char tmp2;
    unsigned char tmp3;
    unsigned char tmp4;

    switch(CANFound)
    {
        case 1:
            //制动压力
            tmp = data[5];
            ECUData_struct.pressure_back = ((double)tmp)/10.0;   //左侧制动压力
            tmp = data[6];
            ECUData_struct.petral_pressure = ((double)tmp)/10.0; //右侧制动压力

            //制动使能反馈
//            if((ECUData_struct.pressure_back>0.01)||(ECUData_struct.petral_pressure>0.01))
//                ECUData_struct.brake_enabled=1;
//            else
//                ECUData_struct.brake_enabled=0;

            //人工驾驶油门反馈
            tmp1 = data[9];
            tmp2 = data[10];
            ECUData_struct.T_throttlePedalPosition1 = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0-0.94)/2.0;
            if (ECUData_struct.T_throttlePedalPosition1 < 0)
                ECUData_struct.T_throttlePedalPosition1 = 0;
            if (ECUData_struct.T_throttlePedalPosition1 > 1)
                ECUData_struct.T_throttlePedalPosition1 = 1;

            tmp1 = data[11];
            tmp2 = data[12];
            ECUData_struct.T_throttlePedalPosition2 = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0-1.84)/2.0;
            if (ECUData_struct.T_throttlePedalPosition2 < 0)
                ECUData_struct.T_throttlePedalPosition2 = 0;
            if (ECUData_struct.T_throttlePedalPosition2 > 1)
                ECUData_struct.T_throttlePedalPosition2 = 1;

            CANFound = -1;

            break;

        case 2:
            //人工驾驶制动压力
            tmp1 = data[5];
            tmp2 = data[6];
            ECUData_struct.T_lFBrakePressure = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0)*5.0-0.68;
            if (ECUData_struct.T_lFBrakePressure < 0)
                ECUData_struct.T_lFBrakePressure = 0;

            tmp1 = data[7];
            tmp2 = data[8];
            ECUData_struct.T_rFBrakePressure = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0)*5.0-0.68;
            if (ECUData_struct.T_rFBrakePressure < 0)
                ECUData_struct.T_rFBrakePressure = 0;

            tmp1 = data[9];
            tmp2 = data[10];
            ECUData_struct.T_lRBrakePressure = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0)*5.0-0.68;
            if (ECUData_struct.T_lRBrakePressure < 0)
                ECUData_struct.T_lRBrakePressure = 0;

            tmp1 = data[11];
            tmp2 = data[12];
            ECUData_struct.T_rRBrakePressure = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0)*5.0-0.68;
            if (ECUData_struct.T_rRBrakePressure < 0)
                ECUData_struct.T_rRBrakePressure = 0;

            CANFound = -1;

            break;

        case 3:
            //四轮轮速
            tmp1 = data[5];
            tmp2 = data[6];
            ECUData_struct.FrontLeftWheelSpeed = ((((unsigned short)(tmp1))<<8)+tmp2)/24.0*M_PI;

            tmp1 = data[7];
            tmp2 = data[8];
            ECUData_struct.FrontRightWheelSpeed = ((((unsigned short)(tmp1))<<8)+tmp2)/24.0*M_PI;

            tmp1 = data[9];
            tmp2 = data[10];
            ECUData_struct.BackLeftWheelSpeed = ((((unsigned short)(tmp1))<<8)+tmp2)/24.0*M_PI;

            tmp1 = data[11];
            tmp2 = data[12];
            ECUData_struct.BackRightWheelSpeed = ((((unsigned short)(tmp1))<<8)+tmp2)/24.0*M_PI;

            CANFound = -1;

            break;

        case 4:
            //故障码
            ECUData_struct.T_escerrorCode = data[5];
            ECUData_struct.T_bottomerrorCode = data[6];

            CANFound = -1;

            break;

        case 5:
            //转向电机
            tmp1 = data[5];
            tmp2 = data[6];
            tmp3 = data[7];
            tmp4 = data[8];
            ECUData_struct.fFLRWheelAverAngle = -(((int(tmp1))<<24)+((int(tmp2))<<16)+((int(tmp3))<<8)+tmp4)/13330.0;
//cout<<"fFLRWheelAverAngle="<<ECUData_struct.fFLRWheelAverAngle<<endl;

            CANFound = -1;

            break;

        case 6:
            //档位
            //ECUData_struct.f_shift = data[5];

            //车速
            tmp1 = data[6];
            tmp2 = data[7];

            ECUData_struct.fForwardVel = (double)((((unsigned short)(tmp1))<<8)+tmp2)/100;


            //制动踏板位置
            tmp1 = data[8];
            tmp2 = data[9];

            petral_pressure = ((((unsigned short)(tmp1))<<8)+tmp2)/1024.0;
            if(petral_pressure>0.5)
                ECUData_struct.brake_enabled=true;
            else
                ECUData_struct.brake_enabled=false;
            CANFound = -1;

            break;
        case 7:
            //OBD发动机转速
            tmp1 = data[5];
            tmp2 = data[6];
            ECUData_struct.EnginRate = (((unsigned short)(tmp1))<<8)+tmp2;
            //cout<<'ECUData_struct.EnginRate '<<ECUData_struct.EnginRate<<endl;

            //OBD档位
            ECUData_struct.f_shift = data[7];
            ECUData_struct.f_shift1 = data[8];
            //发动机负荷率
            tmp1 = data[9];
            tmp2 = data[10];
            ECUData_struct.Engine_load = (double)((((unsigned short)(tmp1))<<8)+tmp2)/100;

            CANFound = -1;

            break;
        default:
            break;
    }
#endif
#ifdef CATERPILLARCAR6 //6t


    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x1a) && (data[4]==0x01))
    {
        CANFound = 1;
    }
    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x1a) && (data[4]==0x02))
    {
        CANFound = 2;
    }
    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x1a) && (data[4]==0x03))
    {
        CANFound = 3;
    }
    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x1a) && (data[4]==0x04))
    {
        CANFound = 4;
    }
    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x2a) && (data[4]==0x01))
    {
        CANFound = 5;
    }
    if((data[0]==0x88)&& (data[1]==0x01) && (data[2]==0x00) && (data[3]==0x2a) && (data[4]==0x02))
    {
        CANFound = 6;
    }
    if((data[0]==0x88)&& (data[1]==0x00) && (data[2]==0x00) && (data[3]==0x3a) && (data[4]==0x01))
    {
        CANFound = 7;
    }

    unsigned char tmp;
    unsigned char tmp1;
    unsigned char tmp2;
    unsigned char tmp3;
    unsigned char tmp4;

    switch(CANFound)
    {
        case 1:
            //制动压力
            tmp = data[5];
            ECUData_struct.pressure_back = ((double)tmp)/10.0;   //左侧制动压力
            tmp = data[6];
            ECUData_struct.petral_pressure = ((double)tmp)/10.0; //右侧制动压力

            //制动使能反馈
//            if((ECUData_struct.pressure_back>0.01)||(ECUData_struct.petral_pressure>0.01))
//                ECUData_struct.brake_enabled=1;
//            else
//                ECUData_struct.brake_enabled=0;

            //人工驾驶油门反馈
            tmp1 = data[9];
            tmp2 = data[10];
            ECUData_struct.T_throttlePedalPosition1 = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0-0.94)/2.0;
            if (ECUData_struct.T_throttlePedalPosition1 < 0)
                ECUData_struct.T_throttlePedalPosition1 = 0;
            if (ECUData_struct.T_throttlePedalPosition1 > 1)
                ECUData_struct.T_throttlePedalPosition1 = 1;

            tmp1 = data[11];
            tmp2 = data[12];
            ECUData_struct.T_throttlePedalPosition2 = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0-1.84)/2.0;
            if (ECUData_struct.T_throttlePedalPosition2 < 0)
                ECUData_struct.T_throttlePedalPosition2 = 0;
            if (ECUData_struct.T_throttlePedalPosition2 > 1)
                ECUData_struct.T_throttlePedalPosition2 = 1;

            CANFound = -1;

            break;

        case 2:
            //人工驾驶制动压力
            tmp1 = data[5];
            tmp2 = data[6];
            ECUData_struct.T_lFBrakePressure = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0)*5.0-0.68;
            if (ECUData_struct.T_lFBrakePressure < 0)
                ECUData_struct.T_lFBrakePressure = 0;

            tmp1 = data[7];
            tmp2 = data[8];
            ECUData_struct.T_rFBrakePressure = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0)*5.0-0.68;
            if (ECUData_struct.T_rFBrakePressure < 0)
                ECUData_struct.T_rFBrakePressure = 0;

            tmp1 = data[9];
            tmp2 = data[10];
            ECUData_struct.T_lRBrakePressure = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0)*5.0-0.68;
            if (ECUData_struct.T_lRBrakePressure < 0)
                ECUData_struct.T_lRBrakePressure = 0;

            tmp1 = data[11];
            tmp2 = data[12];
            ECUData_struct.T_lRBrakePressure = (((((unsigned short)(tmp1))<<8)+tmp2)/1024.0)*5.0-0.68;
            if (ECUData_struct.T_rRBrakePressure < 0)
                ECUData_struct.T_rRBrakePressure = 0;

            CANFound = -1;

            break;

        case 3:
            //四轮轮速
            tmp1 = data[5];
            tmp2 = data[6];
            ECUData_struct.FrontLeftWheelSpeed = ((((unsigned short)(tmp1))<<8)+tmp2)/24.0*PI;

            tmp1 = data[7];
            tmp2 = data[8];
            ECUData_struct.FrontRightWheelSpeed = ((((unsigned short)(tmp1))<<8)+tmp2)/24.0*PI;

            tmp1 = data[9];
            tmp2 = data[10];
            ECUData_struct.BackLeftWheelSpeed = ((((unsigned short)(tmp1))<<8)+tmp2)/24.0*PI;

            tmp1 = data[11];
            tmp2 = data[12];
            ECUData_struct.BackRightWheelSpeed = ((((unsigned short)(tmp1))<<8)+tmp2)/24.0*PI;

            CANFound = -1;

            break;

        case 4:
            //故障码
            ECUData_struct.T_escerrorCode = data[5];
            ECUData_struct.T_bottomerrorCode = data[6];

            CANFound = -1;

            break;

        case 5:
            //转向电机
            tmp1 = data[5];
            tmp2 = data[6];
            tmp3 = data[7];
            tmp4 = data[8];
            ECUData_struct.fFLRWheelAverAngle = -(((int(tmp1))<<24)+((int(tmp2))<<16)+((int(tmp3))<<8)+tmp4)/13330.0;
//cout<<"fFLRWheelAverAngle="<<ECUData_struct.fFLRWheelAverAngle<<endl;

            CANFound = -1;

            break;

        case 6:
            //档位
            //ECUData_struct.f_shift = data[5];

            //车速
            tmp1 = data[6];
            tmp2 = data[7];

            ECUData_struct.fForwardVel = (double)((((unsigned short)(tmp1))<<8)+tmp2)/100;


            //制动踏板位置
            tmp1 = data[8];
            tmp2 = data[9];



            CANFound = -1;

            break;
        case 7:
            //OBD发动机转速
            tmp1 = data[5];
            tmp2 = data[6];
            ECUData_struct.EnginRate = (((unsigned short)(tmp1))<<8)+tmp2;
            //cout<<'ECUData_struct.EnginRate '<<ECUData_struct.EnginRate<<endl;

            //OBD档位
            ECUData_struct.f_shift = data[7];
            ECUData_struct.f_shift1 = data[8];

            CANFound = -1;

            break;
        default:
            break;
    }
#endif
}


