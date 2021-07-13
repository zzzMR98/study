#ifndef OBJECT_DETECTION_RADAR_H_
#define OBJECT_DETECTION_RADAR_H_
//C&C++ headers
#include <cstdio>
#include <vector>
#include <algorithm>
//OpenCV
#include <opencv2/opencv.hpp>
//project headers
#include "TypeDef.h"

using std::vector;

#define NUM 64  //毫米波雷达总的检测目标数
#define METER2PIXEL 5 //1米为5个像素点，根据相机标定参数来定,每个像素间隔0.2m，即像素分辨率为20cm。
#define OBJECT_WIDTH 2 //默认实际车宽为2米
#define PLANE_WIDTH 401  //鸟瞰图
#define PLANE_HEIGHT 601

#define SHOW_TARGET_INFO //显示目标属性信息
#define DISTANCE_TO_RADAR //鸟瞰图中距离标尺为到雷达的距离

//可变参数
#define RADAR2CAR 1.5 //毫米波到车后轴距离，米制单位
#define XLim 2.2//感兴趣目标点在毫米波雷达x正方向最大距离限制

#define SAVE_GET_DATA //是否保存获得的待处理的毫米波数据和自身车辆信息


const int persMap_Middle = (PLANE_WIDTH - 1) / 2; //鸟瞰图宽度中点坐标
const int T1 = 15; //历史设的T1，T2都是20
const int T2 = 20;


class ObjectDetection
{
	public:
	//constructors
    ObjectDetection(void);
    ~ObjectDetection(void);
    public:
    //类数据获取

    void set_radar_data(const continental_radar_target& radar);
    void show_result(vector<moving_object_millimeter>& valid_obj);//显示全部有效目标
    void draw_basic_info(); //绘制基本信息

    //zhanghm-20180304-毫米波雷达数据可视化
    void main_function2(vector<moving_object_millimeter> &vecObj_Temp);

    moving_object_millimeter getSendData(); //获得最终需要发送的目标

    /************************/
    /*Visualiz related*/
    void DisplayAll(); //控制最终显示
    void DrawCommonElements();//绘制基本要素，大家共享的信息，如一些标题，文字说明等
    /*************************/
    private:

   //成员变量
    public:
    bool save_flag;
    IplImage* m_MMW_img_bak; //用于存储绘制好基本要素的雷达图
    IplImage* m_MMW_img; //雷达点鸟瞰图

    //从雷达获取的信息
    moving_object_millimeter MMW_detection_array[NUM];//获取的毫米波雷达目标数据

    Vehicle_Info vehicle_info_; //获得的自身车辆信息
    float vehicle_speed_origin_; //原始车速
    moving_object_millimeter final_obj;
    FILE* fpp;
  //实际处理的雷达和车辆信息数据
    FILE* fp_radar_file;
    FILE* fp_vehicle_file;

    char* text;   //用来显示各种文本
    CvFont cf;
    CvFont cf2; //计时用字体

    vector<moving_object_millimeter> vecObj_Moving;

    vector<moving_object_millimeter> vecObj_Interest;
    vector<double> vecObj_Distance;

    int m_frameCount;
    int _count_num;
    int _recvNum; //接收数据次数

    private:
    moving_object_millimeter _empty_obj;
};

#endif /*OBJECT_DETECTION_RADAR_H_*/
