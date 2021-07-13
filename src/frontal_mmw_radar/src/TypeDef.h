#pragma once

//每一个毫米波雷达目标的数据结构体
typedef struct _moving_object_millimeter{
	unsigned short target_ID; //目标ID号，从1-64
	double range;			//目标距离
	double v;				//目标速度
	double angle;			//目标方位角
	double x;
	double y;
	//mdj_begin
	double Relative_xv;
	double Relative_yv;
	double Relative_acc_x;
	double Relative_acc_y;
	unsigned short Obiect_class;
	double Object_Length;
	double Object_Width;
	unsigned short index_deviation_x;
	unsigned short index_deviation_y;
	unsigned short index_deviation_xv;
	unsigned short index_deviation_yv;
	//mdj_end
	bool valid;
	//zhanghm:20170911 new add
	unsigned short status;//目标状态,CAN_TX_TRACK_STATUS
	unsigned short moving; //运动状态
	bool moving_fast;
	bool moving_slow;
}moving_object_millimeter;

typedef struct _delphi_radar_target{
	moving_object_millimeter delphi_detection_array[64]; //64个目标数据
	unsigned short ACC_Target_ID;//ACC目标的ID号,0-64,0意味着没有ACC目标
	float ESR_vehicle_speed;//从毫米波雷达获取的车速  m/s
	float ESR_yaw_rate; //从毫米波雷达获取的横摆角速度 degree/s
	float vehicle_speed_origin;//原始车辆速度
	float yaw_rate_origin;//原始车辆横摆角速度
}delphi_radar_target;

typedef struct _continental_radar_target{
	moving_object_millimeter continental_detection_array[64];
	unsigned short MaxDistanceCfg;
	unsigned short SensorID;
	unsigned short OutputTypeCfg;
	unsigned short SendQualityCfg;
	unsigned short SendExtInfoCfg;
	unsigned short NofObjectFilterCfg;
	unsigned short Max_NofObj;
	float Min_Distance;
	float Max_Distance;
	float Min_Azimuth;
	float Max_Azimuth;
	float Min_VrelOncome;
	float Max_VrelOncome;
	float Min_VrelDepart;
	unsigned short Min_ProbExists;
	unsigned short Max_ProbExists;
	float Min_Y;
	float Max_Y;
	float Min_X;
	float Max_X;
	float Min_VYRightLeft;
	float Max_VYRightLeft;
	float Min_VXOncome;
	float Max_VXOncome;
	float Min_VYLeftRight;
	float Max_VYLeftRight;
	float Min_VXDepart;
	float Max_VXDepart;
	unsigned short NofObjects;
}continental_radar_target;

//Vehicle information used to send to MMW Radar
typedef struct Vehicle_Info_
{
	float yaw_rate; //degree/s
	float vehicle_speed;//Unit: m/s
	float steering_angle;// Unit: degree
}Vehicle_Info;
