#include "AnalysisGPS.h"

#define PPI 3.14159265358979
#define global_Re 6378245
//#define AnalysisGPS_e (1/298.257)
#define global_wie 7.292e-5

using namespace Eigen;

//for COdometry
COdometry::~COdometry()
{

}

void COdometry::filter_init(double latitude,double longitude,double altitude,double yaw,double pitch,double roll)
{
	m_latitude=latitude;
	m_longitude=longitude;
	m_altitude=altitude;
	m_yaw=yaw;
	m_pitch=pitch;
	m_roll=roll;
	flag_filter_init=true;
}

///currently not used
void COdometry::predictDR(double v_RL, double v_RR, double v_FL, double v_FR, double phi, double timestamp)//四轮轮速估计
{
	double delta_L = atan(tan(phi)*L_ / (L_ - m_bri*tan(phi)));
	double delta_R = atan(tan(phi)*L_ / (L_ + m_bri*tan(phi)));
	Matrix<double, 5, 1> z;
	Matrix<double,5,2> H;
	H << 0, 1, 1, 1, 1,
			1, -m_bri, m_bri, -m_bri, m_bri;
	z << tan(phi)*v_RR / L_, v_RL, v_RR, v_FL*cos(delta_L), v_FR*cos(delta_R);
	Matrix2d PHI = MatrixXd::Identity(2,2);
	Vector2d x_pr = PHI*x_4ws_;
	Matrix<double, 5, 1> z_pr;
	z_pr << x_4ws_(1), x_4ws_(0) - 1.2*x_4ws_(1), x_4ws_(0) + 1.2*x_4ws_(1), x_4ws_(0) - 1.2*x_4ws_(1), x_4ws_(0) + 1.2*x_4ws_(1);
	Matrix<double, 5, 1> delta_z = z - z_pr;

	P_4ws = PHI*P_4ws*PHI.transpose() + Q_4ws;
	MatrixXd K = P_4ws*H.transpose()*(H*P_4ws*H.transpose()+R_4ws).inverse();
	P_4ws = (MatrixXd::Identity(2, 2) - K*H)*P_4ws;
	x_4ws_ = x_pr + K*delta_z;
	//omega值太小，容易被V影响不准
}


/**
 * integrated EKF algorithm. we'd better seprate the system pridict and measurement update part for code reuse.
 * @param x
 * @param x_pr
 * @param delta_z
 * @param P
 * @param Q
 * @param R
 * @param PHI
 * @param H
 * @param lambu
 */
void COdometry::EKF(Ref<MatrixXd>  x, Ref<MatrixXd>  x_pr,Ref<MatrixXd>   delta_z, Ref<MatrixXd>   P,Ref<MatrixXd>   Q, Ref<MatrixXd>   R,
		Ref<MatrixXd>   PHI, Ref<MatrixXd>  H, double lambu)
{
	P= lambu*(PHI*P*PHI.transpose() + Q);
	MatrixXd K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
	P= P- K*H*P;
	x = x_pr + K*delta_z;

}

/// more information about potter of square root algorithm can be found on 卡尔曼滤波与组合导航原理
void COdometry::potter_time_update(Ref<MatrixXd>  deltaP, Ref<MatrixXd>  deltaQ, Ref<MatrixXd>   PHI,  Ref<MatrixXd>   tao,double num)
{
	deltaP=num*deltaP; ///< when num > 1, it is used for adaptive filter, which has been proven unstable. we current make num==1.
	int row_top=deltaP.rows();
	MatrixXd bottomMatrix=(tao*deltaQ).transpose();
	int row_bottom=bottomMatrix.rows();
	MatrixXd A;
	A.resize(row_top+row_bottom,row_top);//Matrix给定大小的唯一用法
	A.topRows(row_top)=deltaP.transpose()*PHI.transpose();
	A.bottomRows(row_bottom)=bottomMatrix;
	//时间更新
	HouseholderQR<MatrixXd> qr(A);
	MatrixXd tempM=qr.matrixQR().triangularView<Upper>();
	deltaP=tempM.topRows(row_top);
    //std::cout<<tempM<<std::endl;
    MatrixXd tempM2=deltaP.transpose();
    deltaP=tempM2;//delat_k/k-1
//    std::cout<<"shijiangengxin wancheng,deltaP is : "<<m_LdeltaP<<std::endl;
}

void COdometry::potter_measure_update(Ref<MatrixXd>  deltaP, Ref<MatrixXd>  R, Ref<MatrixXd> H, Ref<MatrixXd>   delta_z,  Ref<MatrixXd>   x)
{
    //量测更新
    MatrixXd a,K;
    double b,gama;
    int m=delta_z.rows();
    for(int j=0;j<m;j++)
    {

    	a=(H.row(j)*deltaP).transpose();
    	MatrixXd tempM3=a.transpose()*a;
    	b=pow(tempM3(0,0)+R(j,j),-1);
    	gama=pow(1+sqrt(b*R(j,j)),-1);
    	K=b*deltaP*a;
    	x=x+K*(delta_z(j,0));
    	deltaP=deltaP-gama*K*a.transpose();

    }
//	std::cout<<"liangcegengxin wancheng,deltaP is : "<<m_LdeltaP<<std::endl;

}

/// combine adaptive algorithm to potter algorithm.
void COdometry::LASRKF_measureupdate(Ref<MatrixXd>  deltaP, Ref<MatrixXd>  deltaR, Ref<MatrixXd> H, Ref<MatrixXd>   delta_z,Ref<MatrixXd>   delta_z_forR,  Ref<MatrixXd>   x ,double num)
{
	//cout<<"measure update start"<<endl;
	LLT<MatrixXd> llt((1-num)*deltaR*deltaR.transpose()+num*delta_z_forR*delta_z_forR.transpose());//-d*H*deltaP*deltaP.transpose()*H.transpose()
	//MatrixXd temp_vec=H*deltaP;
	//llt.rankUpdate(temp_vec.col(0),-d);
	deltaR=llt.matrixL();
	MatrixXd R=deltaR*deltaR.transpose();
    //量测更新
    MatrixXd a,K;
    double b,gama;
    int m=delta_z.rows();
    for(int j=0;j<m;j++)
    {

    	a=(H.row(j)*deltaP).transpose();
    	MatrixXd tempM3=a.transpose()*a;
    	b=pow(tempM3(0,0)+R(j,j),-1);
    	gama=pow(1+sqrt(b*R(j,j)),-1);
    	K=b*deltaP*a;
    	x=x+K*(delta_z(j,0));
    	deltaP=deltaP-gama*K*a.transpose();

    }
	//cout<<"delta R is:*********"<<m_LdeltaR_GPS<<endl;
}


void COdometry::LidarInit(double IMUla,double IMUlo,double altitude,double IMUroll,double IMUpitch, double IMUyaw, double timestamp)
{
    lat_rad = IMUla;
    lon_rad = IMUlo;
    m_altitude=altitude;
    m_roll=IMUroll;
    m_pitch=IMUpitch;
    m_yaw=IMUyaw;

    m_Lflag=true; // filter init flag
    m_Lx=MatrixXd::Zero(4,1);
    cout<<"Lidar init finish,the latitude is : "<<m_UTMx<<", the longitude is : "<<m_UTMy<<endl;
    //m_UTMzone=static_cast<int>((IMUlo*180/3.14159265 + 180.0) / 6) + 1;
    geographic_to_grid(a, e2, IMUla, IMUlo, &zone, &hemi, &m_UTMy, &m_UTMx);
    Lidarinit_timestamp = timestamp; //used for extrapolator

}

void COdometry::LidarInit_withoutAngle(double IMUla,double IMUlo,double altitude, double timestamp)
{
    m_altitude=altitude;
    lat_rad = IMUla;
    lon_rad = IMUlo;
    m_Lflag=true; // filter init flag
    m_Lx=MatrixXd::Zero(4,1);
    //m_UTMzone=static_cast<int>((IMUlo*180/3.14159265 + 180.0) / 6) + 1;
    geographic_to_grid(a, e2, IMUla, IMUlo, &zone, &hemi, &m_UTMy, &m_UTMx);
    Lidarinit_timestamp = timestamp; //used for extrapolator

}

//for Lidar&GPS
void COdometry::Lidar_time_update(double dt,double LidarSe,double LidarSn,double LidarSu,Quaterniond q_delta,bool timeUpdate_flag, double timestamp)//激光里程计&GPS误差估计,用IMU经纬度初始化
{
    extrapolator_.reset_oldest_time(timestamp);

    // time alignment using IMU velocity.
    // if lidar states init (change) between time interval of system time update, the measurement of lidar odometry
    // shall minus the distance between last time update and nearest Lidar init.
    if(timeupdate_timestamp != 0 && timeupdate_timestamp < Lidarinit_timestamp && Lidarinit_timestamp - timeupdate_timestamp < 0.1)
	{
        LOG(WARNING) << "Lidar_time_update time extrapolator " << -timeupdate_timestamp + Lidarinit_timestamp;
		auto pose_diff = extrapolator_.ExtrapolatePoseAckermann(timeupdate_timestamp, Lidarinit_timestamp);
		pose_diff = ::ivcommon::transform::Rigid3d::Rotation(ivcommon::transform::RollPitchYaw(m_roll, m_pitch, m_yaw)) * pose_diff;
		LidarSe -= pose_diff.translation()(0);
		LidarSn -= pose_diff.translation()(1);
		LidarSu -= pose_diff.translation()(2);
	}
	timeupdate_timestamp = timestamp;

    Matrix<double,3,3> R_n2b=euler2R(m_roll,m_pitch,m_yaw);
    R_n2b=q_delta*R_n2b;//里程计解算出姿态角
    Matrix3d R_b2n=R_n2b.transpose();
    Matrix<double,4,4> LF=MatrixXd::Zero(4,4);//状态转移阵
    LF.block<2,2>(0,2)=R_b2n.block<2,2>(0,0);//delta_S_Lidar
    LF.block<2,2>(2,2)=-1./1000*MatrixXd::Identity(2,2);//delta_vb

    //系统噪声
    MatrixXd LG=MatrixXd::Zero(4,2);
    LG.block<2,2>(2,0)=MatrixXd::Identity(2,2);
    //离散化
    MatrixXd Ltao=(MatrixXd::Identity(4,4)+dt*LF/2)*LG*dt;//噪声驱动阵
    Matrix<double,4,4> LPHI=LF*dt+MatrixXd::Identity(4,4);
    //一步预测

    // we only predict when GPS is valid so that GPS can fix the predict error.
    if(timeUpdate_flag)
    {
     m_Lx=LPHI*m_Lx;
     potter_time_update(m_LdeltaP,m_LdeltaQ,LPHI,Ltao,1);
    }


    m_UTMx+=LidarSe;
    m_UTMy+=LidarSn;
    m_altitude+=LidarSu;

    m_UTMx -= m_Lx(0);
    m_UTMy -= m_Lx(1);
    m_Lx(0) = 0;
    m_Lx(1) = 0;
    timeupdate_timestamp = timestamp;

    grid_to_geographic(a,e2,zone,hemi, m_UTMy,  m_UTMx,
           &lat_rad,&lon_rad);//zzh

    Vector3d V_n;
    V_n<<LidarSe/dt,LidarSn/dt,LidarSu/dt;
    m_V_b=R_n2b*V_n;

    Vector3d euler_now=R2euler(R_n2b);
    m_roll=euler_now(0);
    m_pitch=euler_now(1);
    m_yaw=euler_now(2);
    //尽量不要再写量测，保证激光里程计解算独立
	 /*
	 Matrix<double,3,1> LidarS;
	 LidarS<<LidarSx,
                        LidarSy,
                        LidarSz;
     MatrixXd S_n=R_b2n*LidarS;//dong bei tian
	 if(abs(S_n(0,0))>5)
		 S_n(0,0)=0;
	 m_UTMx+=S_n(0,0);
	 if(abs(S_n(1,0))>5)
		 S_n(1,0)=0;
	 m_UTMy+=S_n(1,0);
	 if(abs(S_n(2,0))>5)
		 S_n(2,0)=0;
	 m_altitude+=S_n(2,0);


	 //之前的角增量算法
	 //Matrix<double,3,3> YAW,ROLL,PITCH;
	 //YAW<<cos(Lidaryaw), sin(Lidaryaw), 0,
     //               -sin(Lidaryaw), cos(Lidaryaw),0,
     //               0,0,1;
     //PITCH<<1,0,0,
     //                   0,cos(Lidarpitch),sin(Lidarpitch),
     //                  0,-sin(Lidarpitch),cos(Lidarpitch);
     //ROLL<<cos(Lidarroll), 0,-sin(Lidarroll),
     //               0,1,0,
     //               sin(Lidarroll),0, cos(Lidarroll);
	 //R_n2b=ROLL*PITCH*YAW*R_n2b;
	 //R_b2n=R_n2b.transpose();

	 //m_Lpitch=asin(R_n2b(1,2));
	 //m_Lroll=atan(-R_n2b(0,2)/R_n2b(2,2));
	 //double yaw_zhengxian=-R_n2b(1,0)/cos(m_Lpitch);
	 //if(yaw_zhengxian>=0)
     //       m_Lyaw=acos(R_n2b(1,1)/cos(m_Lpitch));
     //else
     //   m_Lyaw=2*PPI-acos(R_n2b(1,1)/cos(m_Lpitch));
	 //yaw取值0-2pi

	 m_Lyaw=Lidaryaw;
	 m_Lpitch=Lidarpitch;
	 m_Lroll=Lidarroll;
	 double dtemp=2*PPI-m_Lyaw-GPSheading;
	 while(dtemp>PPI)
		 dtemp-=2*PPI;
	 while(dtemp<-PPI)
		 dtemp+=2*PPI;

	 std::cout<<m_Lx(8)<<std::endl;
	 //判断GPSheading是否有效，若无效，只用里程计信息
		if(GPSheading>0.0001&&abs(dtemp)<30*PPI/180)
		 {

	 z<<m_Lx0-m_GPSxy.x,m_Ly0-m_GPSxy.y,dtemp;//不能跨UTMzone
	 Matrix<double, 3,1> delta_z=z-z_pr;


	 //static double lambuda=1.0;
	 //Matrix<double, 3,3> C_cali=(lambuda/(1+lambuda))*delta_z*delta_z.transpose();
	 //Matrix3d M_cali=LH*PHI*m_LP*PHI.transpose()*LH.transpose();
	 //Matrix3d N_cali=C_cali-LH*m_LQ*LH.transpose()-m_LR;
	 //double trace_NdM=(N_cali(0,0)+N_cali(1,1)+N_cali(2,2))/
	 //		 	 	 	 	 	 	 	 	 	(M_cali(0,0)+M_cali(1,1)+M_cali(2,2));//trace/trace
	 //lambuda=max(1.0,trace_NdM);

	 //std::cout<<"lambuda finish"<<std::endl;
	 //EKF(m_Lx,Lx_pr, delta_z, m_LP,m_LQ,m_LR,PHI,LH, 1);//lambuda:自适应滤波的渐消因子
	 potter(m_Lx,Lx_pr,delta_z, m_LR,PHI,LH);//平方根滤波
    //std::cout<<"the x error is "<<m_Lx(0)<<std::endl;
   // std::cout<<"the y error is "<<m_Lx(1)<<std::endl;
    //std::cout<<"the angle error is "<<m_Lx(4)<<std::endl;
	 m_Lx0-=m_Lx(0);
	 m_Ly0-=m_Lx(1);
	 m_Lyaw+=m_Lx(8);
	 std::cout<<"东向误差"<<m_Lx(0)<<std::endl;
	 std::cout<<"北向误差"<<m_Lx(1)<<std::endl;
	 std::cout<<"航向误差"<<m_Lx(8)<<std::endl;
    }
	 while(m_Lyaw>=2*PPI)
		 m_Lyaw-=2*PPI;
	 while(m_Lyaw<0)
		 m_Lyaw+=2*PPI;


	 m_UTM.UTMXYToLatLon(m_Lx0, m_Ly0, Lidarzone, false, m_LLaLo);
	 Matrix3d temp_R=euler2R(m_Lyaw,m_Lpitch,m_Lroll);
	 m_Lpose=R2quat(temp_R);
	 //转回经纬度
	 */
}

void COdometry::Lidar_measure_update_GPS(double latitude,double longitude, double timestamp)
{
	double last_timestamp = std::max(timeupdate_timestamp, Lidarinit_timestamp);
    //LOG(WARNING) << "Lidar_measure_update_GPS time extrapolator " << timestamp - last_timestamp;
	auto pose_diff = extrapolator_.ExtrapolatePoseAckermann(last_timestamp, timestamp);
	pose_diff = ::ivcommon::transform::Rigid3d::Rotation(ivcommon::transform::RollPitchYaw(m_roll, m_pitch, m_yaw))*pose_diff;


	Matrix<double, 2,4> LH=MatrixXd::Zero(2,4);
	LH.block<2,2>(0,0)=MatrixXd::Identity(2,2);
	Matrix<double,2,1> Lz;
	double GPSE;
	double GPSN;
	geographic_to_grid(a, e2, latitude, longitude, &zone, &hemi, &GPSN, &GPSE);
	Lz<<m_UTMx + pose_diff.translation()(0) - GPSE, m_UTMy + pose_diff.translation()(1) - GPSN;
	MatrixXd L_deltaz=Lz-LH*m_Lx;
	cout<<"GPS L_deltaz is :"<<L_deltaz<<endl;
	cout<<"GPS Lz is :"<<Lz<<endl;
//	MatrixXd Merror=L_deltaz;
//	cout<<"Merror is :"<<Merror<<endl;
//	Matrix<double, 2,1> L_deltaz_forR=L_deltaz;
//	if(fabs(L_deltaz_forR(0,0))<20)
//		L_deltaz_forR(0,0)=50;
//	else if(fabs(L_deltaz_forR(0,0))>20)
//		L_deltaz_forR(0,0)=100;
//	if(fabs(L_deltaz_forR(1,0))<20)
//		L_deltaz_forR(1,0)=50;
//	else if(fabs(L_deltaz_forR(1,0))>20)
//		L_deltaz_forR(1,0)=100;

	LASRKF_measureupdate(m_LdeltaP, m_LdeltaR_GPS, LH, L_deltaz, L_deltaz, m_Lx,0.5);
//	potter_measure_update(m_LdeltaP, m_LR_GPS, LH, L_deltaz, m_Lx);
}

void COdometry::Lidar_measure_update_ECU(double v)
{
	Matrix<double ,2,4> LH=MatrixXd::Zero(2,4);
	Vector3d V_b;
	V_b<<0,v,0;
	cout<<"ECU velocity is"<<v<<endl;
	cout<<"ODO velocity is"<<m_V_b(1)<<endl;
	LH.block<2,2>(0,2)=MatrixXd::Identity(2,2);
	Matrix<double, 2,1> Lz=(m_V_b-V_b).topRows(2);
	MatrixXd L_deltaz=Lz-LH*m_Lx;
	cout<<"ECU L_deltaz is :"<<L_deltaz<<endl;
	cout<<"ECU Lz is :"<<Lz<<endl;
	//MatrixXd Merror=L_deltaz*L_deltaz.transpose();
	//if(abs(Merror(0,0))<5&&abs(Merror(1,1))<5)
	LASRKF_measureupdate(m_LdeltaP, m_LdeltaR_ECU, LH, L_deltaz, L_deltaz, m_Lx,0);
}

void COdometry::Lidar_measure_update_resetangle(double roll,double pitch,double yaw)
{
	m_roll=roll;
	m_pitch=pitch;
	m_yaw=yaw;
}

//集中式卡尔曼滤波器 begin. it is about to make a INS system instead of use it.
void COdometry::IMU_time_update(double dt, double latitude, double longitude, double height, double vbx, double vby, double vbz,
		double ax, double ay, double az,
		double roll, double pitch, double yaw, double omega_x, double omega_y, double omega_z)
{
	Matrix<double,15,15> IF=MatrixXd::Zero(15,15);//状态转移阵
	double RplusH=global_Re+height;
	Matrix<double,3,3> Rn2b=euler2R(roll,pitch,yaw);//有问题
	Matrix3d Rb2n=Rn2b.transpose();
	Vector3d a_B;
	a_B<<ax,ay,az;
	Vector3d a_N=Rb2n*a_B;
	Vector3d v_B;
	v_B<<vbx,vby,vbz;
	Vector3d v_N=Rb2n*v_B;
	double ve=v_N(0);
	double vn=v_N(1);
	double vu=v_N(2);

	IF(0,2)=-vn/pow((RplusH),2);
	IF(0,4)=1/RplusH;
	IF(1,0)=ve/cos(latitude)*tan(latitude)/RplusH;
	IF(1,2)=-ve/cos(latitude)/pow(RplusH,2);
	IF(1,3)=1/cos(latitude)/RplusH;
	IF(2,5)=1;
	//~ve_dot
	IF(3,7)=-a_N(2);
	IF(3,8)=a_N(1);
	//IF(3,18)=Rb2n(0,0)*ax;
	//IF(3,19)=Rb2n(0,1)*ay;
	//IF(3,20)=Rb2n(0,2)*az;
	IF(3,3)=(vn*tan(latitude)-vu)/RplusH;
	IF(3,4)=2*global_wie*sin(latitude)+ve*tan(latitude)/RplusH;
	IF(3,5)=-(2*global_wie*cos(latitude)+ve/RplusH);
	IF(3,0)=2*global_wie*(vu*sin(latitude)+vn*cos(latitude))+ve*vn/cos(latitude)/cos(latitude)/RplusH;
	IF(3,2)=(ve*vu-ve*vn*tan(latitude))/pow(RplusH,2);
	IF(3,12)=Rb2n(0,0);
	IF(3,13)=Rb2n(0,1);
	IF(3,14)=Rb2n(0,2);
	//~vn_dot
	IF(4,8)=-a_N(0);
	IF(4,6)=a_N(2);
	//IF(4,18)=Rb2n(1,0)*ax;
	//IF(4,19)=Rb2n(1,1)*ay;
	//IF(4,20)=Rb2n(1,2)*az;
	IF(4,3)=-2*(global_wie*sin(latitude)+ve*tan(latitude)/RplusH);
	IF(4,4)=-vu/RplusH;
	IF(4,5)=-vn/RplusH;
	IF(4,0)=-(2*ve*global_wie*cos(latitude)+pow(ve/cos(latitude),2)/RplusH);
	IF(4,2)=vn*vu/pow(RplusH,2)+ve*ve*tan(latitude)/pow(RplusH,2);
	IF(4,12)=Rb2n(1,0);
	IF(4,13)=Rb2n(1,1);
	IF(4,14)=Rb2n(1,2);
	//~vu_dot
	IF(5,7)=a_N(0);
	IF(5,6)=-a_N(1);
	//IF(5,18)=Rb2n(2,0)*ax;
	//IF(5,19)=Rb2n(2,1)*ay;
	//IF(5,20)=Rb2n(2,2)*az;
	IF(5,3)=2*(global_wie*cos(latitude)+ve/RplusH);
	IF(5,4)=2*vn/RplusH;
	IF(5,0)=-2*ve*sin(latitude);
	IF(5,2)=-(vn*vn+ve*ve)/pow(RplusH,2);
	IF(5,12)=Rb2n(2,0);
	IF(5,13)=Rb2n(2,1);
	IF(5,14)=Rb2n(2,2);
	//phi_e_dot
	IF(6,7)=global_wie*sin(latitude)+ve*tan(latitude)/RplusH;
	IF(6,8)=-(global_wie*cos(latitude)+ve/RplusH);
	IF(6,4)=-1/RplusH;
	IF(6,2)=vn/pow(RplusH,2);
	//IF(6,15)=-Rb2n(0,0)*omega_x;
	//IF(6,16)=-Rb2n(0,1)*omega_y;
	//IF(6,17)=-Rb2n(0,2)*omega_z;
	IF(6,9)=-Rb2n(0,0);
	IF(6,10)=-Rb2n(0,1);
	IF(6,11)=-Rb2n(0,2);
	//phi_n_dot
	IF(7,6)=-(global_wie*sin(latitude)+ve*tan(latitude)/RplusH);
	IF(7,8)=-vn/RplusH;
	IF(7,0)=-global_wie*sin(latitude);
	IF(7,3)=1/RplusH;
	IF(7,2)=-ve/pow(RplusH,2);
	//IF(7,15)=-Rb2n(1,0)*omega_x;
	//IF(7,16)=-Rb2n(1,1)*omega_y;
	//IF(7,17)=-Rb2n(1,2)*omega_z;
	IF(7,9)=-Rb2n(1,0);
	IF(7,10)=-Rb2n(1,1);
	IF(7,11)=-Rb2n(1,2);
	//phi_u_dot
	IF(8,6)=global_wie*cos(latitude)+ve/RplusH;
	IF(8,7)=vn/RplusH;
	IF(8,0)=global_wie*cos(latitude)+ve/cos(latitude)/cos(latitude)/RplusH;
	IF(8,3)=tan(latitude)/RplusH;
	IF(8,2)=-ve*tan(latitude)/RplusH;
	//IF(8,15)=-Rb2n(2,0);
	//IF(8,16)=-Rb2n(2,1);
	//IF(8,17)=-Rb2n(2,2);
	IF(8,9)=-Rb2n(2,0);
	IF(8,10)=-Rb2n(2,1);
	IF(8,11)=-Rb2n(2,2);
	//将陀螺的启动误差和慢变漂移加起来，当做慢变漂移
	IF(9,9)=-1/300;//tao_g=300s
	IF(10,10)=-1/300;
	IF(11,11)=-1/300;

	//系统噪声为w_ins=[wgx wgy wgz wax way waz]T
	Matrix<double, 15,6> IG=MatrixXd::Zero(15,6);
	IG.block<3,3>(6,0)=Rb2n;
	IG.block<3,3>(9,0)=MatrixXd::Identity(3,3);
	IG.block<3,3>(12,3)=MatrixXd::Identity(3,3);

	//离散化
	MatrixXd IPHI=IF*dt+MatrixXd::Identity(15,15);
	cout<<"the DT is : "<<dt<<endl;
	MatrixXd Itao=(MatrixXd::Identity(15,15)+dt*IF/2)*IG*dt;//噪声驱动阵
	//一步预测 均方误差Pk/k-1=IPHI*Pk-1*IPHI.transpose();+Itao*Qk-1*Itao.transpose();
	potter_time_update(m_IdeltaP, m_IdeltaQ, IPHI, Itao,1);

	//m_Ix=IPHI*m_Ix;
	m_Ix(2,0)=0;
	//m_Ix(6,0)=0;
	//m_Ix(7,0)=0;
	//m_Ix(8,0)=0;
	//m_Ix(12,0)=0;
	//m_Ix(13,0)=0;
	//m_Ix(14,0)=0;
	m_latitude=latitude;
	m_longitude=longitude;
	m_altitude=height;


	cout<<" error is: "<<m_Ix*180/M_PI<<endl;

	m_ve=ve;
	m_vn=vn;
	m_vu=vu;
	/*
	Vector3d phi_vec;
	phi_vec<<-m_Ix(6,0),-m_Ix(7,0),-m_Ix(8,0);
	Matrix3d Rt2p=MatrixXd::Identity(3,3)+antisymmetric_matrix(phi_vec);
	Rn2b=Rn2b*Rt2p;
	m_pitch=asin(Rn2b(1,2));
	m_roll=atan(-Rn2b(0,2)/Rn2b(2,2));
	double yaw_zhengxian=-Rn2b(1,0)/cos(m_pitch);
	if(yaw_zhengxian>=0)
		m_yaw=acos(Rn2b(1,1)/cos(m_pitch));
	else
	    m_yaw=2*PPI-acos(Rn2b(1,1)/cos(m_pitch));
	//yaw取值0-2pi
	*/
	m_yaw=yaw;
	m_pitch=pitch;
	m_roll=roll;
	m_omegabx=omega_x;
	m_omegaby=omega_y;
	m_omegabz=omega_z;

	flag_filter_init=true;
}


void COdometry::IMU_measure_update_Odometry(double latitude,double longitude, double altitude, Vector3d euler)
{
	Matrix<double,6,15> IH=MatrixXd::Zero(6,15);
	IH.block<3,3>(0,0)=MatrixXd::Identity(3,3);
	IH.block<3,3>(3,6)=MatrixXd::Identity(3,3);

	Matrix<double,6,1> Iz=MatrixXd::Zero(6,1);
	Matrix3d Rn2b=euler2R(euler(0),euler(1),euler(2));
	Matrix3d Rndot2b=euler2R(m_roll,m_pitch,m_yaw);
	Matrix3d Rn2ndot=Rndot2b.transpose()*Rn2b;
	Vector3d phi=R2euler(Rn2ndot);
	Iz<<m_latitude-latitude, m_longitude-longitude, m_altitude-altitude,
			phi(1), phi(0), phi(2);

	MatrixXd I_deltaz=Iz-IH*m_Ix;
	//cout<<"m_omegabz is"<<m_omegabz<<endl;
	//cout<<"Odo_omegabz is:"<<2*omega_q.z()<<endl;
	//cout<<"Odo_Iz is:"<<Iz*180/PPI<<endl;
	//cout<<"I_deltaz is :"<<I_deltaz*180/PPI<<endl;
	//potter_measure_update(m_IdeltaP, m_IR_Odo, IH, I_deltaz, m_Ix);
	cout<<"I_deltaz is :"<<I_deltaz*180/PPI<<endl;
	cout<<"Iz is :"<<Iz*180/PPI<<endl;
	LASRKF_measureupdate(m_IdeltaP, m_IdeltaR_Odo, IH, I_deltaz, I_deltaz, m_Ix,0.5);
}

void COdometry::IMU_measure_update_GPS(double latitude, double longitude, double height, double yaw)
{
	Matrix<double, 4,15> IH=MatrixXd::Zero(4,15);
	IH.block<3,3>(0,0)=MatrixXd::Identity(3,3);
	IH(3,8)=1;
	Matrix<double,4,1> Iz;
	Iz<<m_latitude-latitude, m_longitude-longitude, m_altitude-height, m_yaw-yaw;
	MatrixXd I_deltaz=Iz-IH*m_Ix;
	if(abs(yaw)<0.1/PPI)
		I_deltaz(3,0)=0;
	I_deltaz(2,0)=0;
	cout<<"I_deltaz is :"<<I_deltaz*180/PPI<<endl;
	cout<<"Iz is :"<<Iz*180/PPI<<endl;
	MatrixXd Merror=I_deltaz*I_deltaz.transpose();
	//if(abs(Merror(0,0))<5e-5&&abs(Merror(1,1))<5e-5)
	LASRKF_measureupdate(m_IdeltaP, m_IdeltaR_GPS, IH, I_deltaz, I_deltaz, m_Ix,0.6);
	//potter_measure_update(m_IdeltaP, m_IR_GPS, IH, I_deltaz, m_Ix);
}

void COdometry::IMU_measure_update_ECU(double v,double omega)
{
	Matrix<double, 4,15> IH=MatrixXd::Zero(4,15);
	IH.block<3,3>(0,3)=MatrixXd::Identity(3,3);
	IH(3,11)=1;
	Matrix<double,4,1> Iz;
	Vector3d v_b;
	v_b<<0,v,0;
	Matrix3d Rn2b=euler2R(m_roll,m_pitch,m_yaw);
	Matrix3d Rb2n=Rn2b.transpose();
	Vector3d v_n=Rb2n*v_b;
	Iz<<m_ve-v_n(0), m_vn-v_n(1), m_vu-v_n(2), m_omegabz-omega;
	MatrixXd I_deltaz=Iz-IH*m_Ix;

	I_deltaz(3,0)=0;
	cout<<"I_deltaz is :"<<I_deltaz<<endl;
	cout<<"Iz is :"<<Iz<<endl;
	//potter_measure_update(m_IdeltaP, m_IR_GPS, IH, I_deltaz, m_Ix);
	LASRKF_measureupdate(m_IdeltaP, m_IdeltaR_ECU, IH, I_deltaz,I_deltaz, m_Ix,0.5);
}
//集中式卡尔曼滤波器 end

//联邦卡尔曼滤波器 begin
//联邦卡尔曼滤波器 end

double COdometry::getLidarLa()
{
	return lat_rad;
}

double COdometry::getLidarLo()
{
	return lon_rad;
}

double COdometry::getLidarheading()
{
    double temp=2*PPI-m_yaw;
	return temp;
}

/**
 * this function is made for coor trans, opposite to point trans.
 * if want to get point trans euler_to_R, see ::ivcommon::transform::Rigid3d::fromYPR
 * @param roll
 * @param pitch
 * @param yaw
 * @return
 */
Matrix3d COdometry::euler2R(double roll,double pitch,double yaw)
{
	//euler2R
	Matrix3d R;
	R=AngleAxisd(-roll,Vector3d::UnitY())*
			AngleAxisd(-pitch,Vector3d::UnitX())
			*AngleAxisd(-yaw,Vector3d::UnitZ());

	return R;
}

Quaterniond  COdometry::R2quat(Matrix3d R)
{
	Quaterniond temp;
	temp=R;
	return temp;
}

/**
 * this function is made for coor trans, opposite to point trans.
 * if want to get point trans R_to_euler, see ::ivcommon::transform::Rigid3::toYPR
 * @param R_n2b
 * @return
 */
Vector3d COdometry::R2euler(Ref<MatrixXd>  R_n2b)
{
	Vector3d euler;//roll pitch yaw
	euler(1)=asin(R_n2b(1,2));
	euler(0)=atan(-R_n2b(0,2)/R_n2b(2,2));
	double yaw_zhengxian=-R_n2b(1,0)/cos(euler(1));
	if(yaw_zhengxian>=0)
		euler(2)=acos(R_n2b(1,1)/cos(euler(1)));
    else
    	euler(2)=-acos(R_n2b(1,1)/cos(euler(1)));
	 //yaw取值-pi~pi
	return euler;
}

// we will use it when making quaternion derivation.
Matrix3d COdometry::antisymmetric_matrix(Vector3d vec)
{
	Matrix3d temp;
	temp<<0, -vec(2), vec(1),
		  vec(2), 0, -vec(0),
		  -vec(1), vec(0), 0;
	return temp;
}

double COdometry::getla()
{
	return m_latitude-m_Ix(0);
}
double COdometry::getlo()
{
	return m_longitude-m_Ix(1);
}
double COdometry::getaltitude()
{
	return m_altitude-m_Ix(2);
}
double COdometry::getqx()
{
	return m_pose.x();
}
double COdometry::getqy()
{
	return m_pose.y();
}
double COdometry::getqz()
{
	return m_pose.z();
}
double COdometry::getqw()
{
	return m_pose.w();
}
double COdometry::getroll()
{
	return m_roll;
}
double COdometry::getpitch()
{
	return m_pitch;
}
double COdometry::getyaw()
{
	return m_yaw;
}
