#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <math.h>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <algorithm>
//#include "CoorConv.h"//for coord trans
#include "util/utm/utm.h"
#include "util/utm/datum.h"
#include <eigen3/Eigen/Dense>
#include "pose_extrapolator.h"

using namespace std;
using namespace Eigen;

/**
 * \brief
 * a Karman based filter. the input is data from GPS INS Lidarodometry (ECU not recommond at present), and the output is fused Latitude Longitute Height, euler angle.
 * the whole process can be divided into 3 step: init error Matrix(extremely important), time update and measure update.
 */
class COdometry
{
	public:

    /**
     * init all the cov Matrix including state ,system error and multiple measurement error. improper values may lead system crash.
     */
	COdometry(double time) : extrapolator_(time)
	{
		//四轮轮速估计 not used currently
		L_=4.66;//z轴距，海梁中巴车qjy 20171020
		m_bri = 1.114;
		P_4ws = MatrixXd::Identity(2, 2);
		Q_4ws = MatrixXd::Identity(2, 2);
		R_4ws = MatrixXd::Identity(5, 5);

        initParam_LG();
        initParam_LI();

		a=6378137;
		e2= 0.0818192*0.0818192;//e的平方
		zone =UTM_ZONE_AUTO;
		hemi = HEMI_NORTH;

	}

	~COdometry();

	void initParam_LG()
    {
        //里程计和GPS组合导航
        /// init timestamp for extrapolator
        Lidarinit_timestamp = 0;
        timeupdate_timestamp = 0;
        m_Lflag=false;
        m_LP = MatrixXd::Zero(4, 4); ///< system states covariance
        m_LP.block<2,2>(0,0)=1*MatrixXd::Identity(2,2); ///< position init covariance
        m_LP.block<2,2>(2,2)=0.1*MatrixXd::Identity(2,2); ///< velocity init covariance

        m_LQ = 5e-3*MatrixXd::Identity(2, 2); ///< system predict noise covariance

        m_LR_GPS=10*MatrixXd::Identity(2, 2); ///< measurement noise covariance of GPS
        m_LR_ECU=1e-2*MatrixXd::Identity(2,2);///< measurement noise covariance of ECU

        LLT<MatrixXd> lltofP(m_LP); ///< square root of system states covariance for square root Kalman Algorithm
        m_LdeltaP=lltofP.matrixL();

        LLT<MatrixXd> lltofQ(m_LQ);
        m_LdeltaQ=lltofQ.matrixL();

        LLT<MatrixXd> lltofR_GPS(m_LR_GPS);
        m_LdeltaR_GPS=lltofR_GPS.matrixL();

        LLT<MatrixXd> lltofR_ECU(m_LR_ECU);
        m_LdeltaR_ECU=lltofR_ECU.matrixL();
        //std::cout<<"initialize,deltaP is : "<<m_LdeltaP<<std::endl;
        //std::cout<<"initialize,deltaQ is : "<<m_LdeltaR_GPS<<std::endl;
    }

    void initParam_LI()
    {
        //里程计和惯导组合导航

        m_DGPSstate='1';
        flag_filter_init=false;
        m_Ix=MatrixXd::Zero(15,1);

        m_IP=MatrixXd::Zero(15,15);
        m_IP.block<2,2>(0,0)=5e-8*MatrixXd::Identity(2,2);
        m_IP(2,2)=0.01;
        m_IP.block<3,3>(3,3)=MatrixXd::Zero(3,3);
        m_IP.block<3,3>(6,6)=1e-5*MatrixXd::Identity(3,3);
        m_IP.block<6,6>(9,9)=MatrixXd::Zero(6,6);

        m_IQ=MatrixXd::Zero(6,6);
        m_IQ(0,0)=pow(0.01/3600,2);
        m_IQ(1,1)=m_IQ(0,0);
        m_IQ(2,2)=m_IQ(0,0);
        m_IQ(3,3)=1e-8;
        m_IQ(4,4)=1e-8;
        m_IQ(5,5)=1e-8;

        m_IR_GPS=MatrixXd::Zero(4,4);
        //m_IR_GPS(0,0)=pow(0.02/60,2);
        m_IR_GPS(0,0)=2e-3;
        m_IR_GPS(1,1)=m_IR_GPS(0,0);
        m_IR_GPS(2,2)=100;
        m_IR_GPS(3,3)=1;

        m_IR_ECU=MatrixXd::Zero(4,4);
        //m_IR_GPS(0,0)=pow(0.02/60,2);
        m_IR_ECU(0,0)=1;
        m_IR_ECU(1,1)=m_IR_ECU(0,0);
        m_IR_ECU(2,2)=m_IR_ECU(0,0);
        m_IR_ECU(3,3)=1;

        m_IR_Odo=MatrixXd::Zero(6,6);
        m_IR_Odo.block<2,2>(0,0)=1e-5*MatrixXd::Identity(2,2);
        m_IR_Odo(2,2)=0.1;
        m_IR_Odo.block<3,3>(3,3)=1e-2*MatrixXd::Identity(3,3);


        LLT<MatrixXd> lltofIP(m_IP);
        m_IdeltaP=lltofIP.matrixL();

        LLT<MatrixXd> lltofIQ(m_IQ);
        m_IdeltaQ=lltofIQ.matrixL();

        LLT<MatrixXd> lltofIR_GPS(m_IR_GPS);
        m_IdeltaR_GPS=lltofIR_GPS.matrixL();

        LLT<MatrixXd> lltofIR_Odo(m_IR_Odo);
        m_IdeltaR_Odo=lltofIR_Odo.matrixL();

        LLT<MatrixXd> lltofIR_ECU(m_IR_ECU);
        m_IdeltaR_ECU=lltofIR_ECU.matrixL();
    }

	/**
	 * inti filter param. usually from INS
	 * @param latitude
	 * @param longitude
	 * @param altitude
	 * @param yaw
	 * @param pitch
	 * @param roll
	 */
	void filter_init(double latitude,double longitude,double altitude,double yaw,double pitch,double roll);

	void AddInsVelocityData(const Time_Align::InsVelocityData& insVelocityData)
    {
	    extrapolator_.AddInsVelocityData(insVelocityData);
    }
	/**
	 * time update using potter suqare root algorithm.
	 * @param deltaP lower triangular decomposition of state error covMatrix;
	 * @param deltaQ lower triangular decomposition of system error covMatrix
	 * @param PHI    system state one step transfer Matrix
	 * @param tao    system noise input Matrix
	 * @param num    temperorily unused
	 */
	void potter_time_update(Ref<MatrixXd>  deltaP, Ref<MatrixXd>  deltaQ, Ref<MatrixXd>   PHI,  Ref<MatrixXd>   tao,double num);

	/**
	 * measure update using potter square root algorithm
	 * @param deltaP lower triangular decomposition of state error covMatrix;
	 * @param R      measure error covMatrix, vary with different measurements;
	 * @param H      measure error model
	 * @param delta_z residuals
	 * @param x     system state
	 */
	void potter_measure_update(Ref<MatrixXd>  deltaP, Ref<MatrixXd>  R, Ref<MatrixXd> H, Ref<MatrixXd>   delta_z,  Ref<MatrixXd>   x);
	void EKF(Ref<MatrixXd>  x, Ref<MatrixXd>  x_pr,Ref<MatrixXd>   delta_z, Ref<MatrixXd>   P,Ref<MatrixXd>   Q, Ref<MatrixXd>   R,
			Ref<MatrixXd>   PHI, Ref<MatrixXd>  H, double lambu);///< 渐消卡尔曼

	/**
	 * measure update using adaptive square root algorithm
	 * @param deltaP         lower triangular decomposition of state error covMatrix;
	 * @param deltaR         lower triangular decomposition of measure error covMatrix, vary with different measurements;
	 * @param H              measure error model
	 * @param delta_z        residuals
	 * @param delta_z_forR   usualy the same to residuals
	 * @param x              system state
	 * @param num            proportion of the new calculated measure error
	 */
	void LASRKF_measureupdate(Ref<MatrixXd>  deltaP, Ref<MatrixXd>  deltaR, Ref<MatrixXd> H, Ref<MatrixXd>   delta_z, Ref<MatrixXd>   delta_z_forR,  Ref<MatrixXd>   x, double num);
	/**
	 * inti filter param. usually from INS
	 * @param IMUla
	 * @param IMUlo
	 * @param altitude
	 * @param IMUroll
	 * @param IMUpitch
	 * @param IMUyaw
	 */
    void LidarInit(double IMUla,double IMUlo,double altitude,double IMUroll,double IMUpitch, double IMUyaw, double timestamp);

    void LidarInit_withoutAngle(double IMUla,double IMUlo,double altitude, double timestamp);

	void predictDR(double v_RL, double v_RR,double v_FL,double v_FR,double phi,double timestamp);///< 四轮轮速估计, currently unused

	/**
	 * Lidarodometry and GPS fuse system    time update
	 * @param dt          steplength
	 * @param LidarSx     stepping LidarOdometry mileage increment in East
	 * @param LidarSy     stepping LidarOdometry mileage increment in North
	 * @param LidarSz     stepping LidarOdometry mileage increment in Height
	 * @param q_delta     stepping LidarOdometry quaternion increment
	 * @param DGPSstate   flag to indicate whether GPS is valid right now
	 */
    void Lidar_time_update(double dt,double LidarSe,double LidarSn,double LidarSu,Quaterniond q_delta,bool Lidar_time_update, double timestamp);//激光里程计&GPS误差估计

    /**
     * Lidarodometry and GPS fuse system    measure update for GPS
     * @param latitude   GPS latitude
     * @param longitude  GPS longitude
     */
    void Lidar_measure_update_GPS(double latitude,double longitude, double timestamp);
    void Lidar_measure_update_ECU(double v);

    /**
     * reset the euler angle of filter
     * @param roll
     * @param pitch
     * @param yaw
     */
    void Lidar_measure_update_resetangle(double roll,double pitch,double yaw);
    //集中卡尔曼 begin

    /**
     * the function with prefix "IMU" is designed for IMU-core-state filter, which has been proven to be improper
     */
	void IMU_time_update(double dt, double latitude, double longitude, double height, double vbx, double vby, double vbz,
    					double ax, double ay, double az,
    					double roll, double pitch, double yaw, double omega_x, double omega_y, double omega_z);
    void IMU_measure_update_Odometry(double latitude,double longitude, double altitude, Vector3d euler);
    void IMU_measure_update_GPS(double latitude, double longitude, double height, double yaw);
    void IMU_measure_update_ECU(double v,double omega);
    //集中卡尔曼 end

    Matrix3d euler2R(double roll,double pitch,double yaw);
    Quaterniond R2quat(Matrix3d R);
    Vector3d R2euler(Ref<MatrixXd>  R);
    Matrix3d antisymmetric_matrix(Vector3d vec);
    double getLidarLa();
    double getLidarLo();
    double getLidarheading();
    double getla();
    double getlo();
    double getaltitude();
    double getqx();
    double getqy();
    double getqz();
    double getqw();
    double getroll();
    double getpitch();
    double getyaw();

    bool flag_filter_init;//for IMU filter
    bool m_Lflag;//对准完成标志位，1完成,for Lidar filter

private:
    //common use
    double m_latitude;//dong
    double m_longitude;//bei
    double m_altitude;
    double m_yaw;
    double m_pitch;
    double m_roll;
    double m_ve;//东北天系下速度
    double m_vn;
    double m_vu;
    double m_omegabx;
    double m_omegaby;
    double m_omegabz;
    Quaterniond m_pose;
    char m_DGPSstate;

    //UTMCoor m_GPSxy;//用于UTM转换
    //WGS84Corr m_LLaLo;//当前时刻的经纬度
    //UTMconv m_UTM;
    //int m_UTMzone;//UTM used
    double m_UTMx;//激光里程计解算xy
    double m_UTMy;
    double a;
    double e2;//e的平方
    GridZone zone;
    Hemisphere hemi;
    double lat_rad;
    double lon_rad;

    /// 四轮估计
    Vector2d x_4ws_; //v,omega,omega temperorily unused
    Matrix2d P_4ws;
    Matrix2d Q_4ws;
    Matrix<double, 5, 5> R_4ws;
    double L_;//轴距
    double m_bri;//半桥长度1114

    /// for GPS&Lidar&DR(&IMU) zzh0515
    Vector3d m_V_b;//b系车速
    Matrix<double, 4,4> m_LP;
    Matrix<double, 2,2> m_LQ;
    Matrix<double, 2,2> m_LR_GPS;
    Matrix<double, 2,2> m_LR_ECU;
    Matrix<double, 4,4> m_LdeltaP;
    Matrix<double, 2,2> m_LdeltaQ;
    Matrix<double, 2,2> m_LdeltaR_GPS;
    Matrix<double, 2,2> m_LdeltaR_ECU;
    Matrix<double,4,1> m_Lx;//delta_rx_Lidar,delta_ry_Lidar,delta_ve,delta_vn,delta_vu,delta_rx_GPS,delta_ry_GPS;
    double Lidarinit_timestamp;
    double timeupdate_timestamp;
    Time_Align::PoseExtrapolator extrapolator_;

    /// IMU&Odo zzh0330
    Matrix<double,15,15> m_IP;
    Matrix<double,6,6> m_IQ;//wgxyz,waxyz
    Matrix<double,15,15> m_IdeltaP;
    Matrix<double,6,6> m_IdeltaQ;//wgxyz,waxyz
    Matrix<double,6,6> m_IR_Odo;//
    Matrix<double,4,4> m_IR_GPS;
    Matrix<double,4,4> m_IR_ECU;
    Matrix<double,4,4> m_IdeltaR_GPS;
    Matrix<double,6,6> m_IdeltaR_Odo;
    Matrix<double,4,4> m_IdeltaR_ECU;

    Matrix<double,15,1> m_Ix;//~L,~lambuda,~h,~ve,~vn,~vu,phi_e,phi_n,phi_u,e_bx,e_by,e_bz,delta_x,delta_y,delta_z

};

