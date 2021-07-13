#include "con_object_detection_radar.h"
#define PI 3.1415926535898
const double toRAD = PI/180.0;

ObjectDetection::ObjectDetection(void)
{
    //图像成员变量指针内存申请
	m_MMW_img_bak = cvCreateImage(cvSize(PLANE_WIDTH, PLANE_HEIGHT), IPL_DEPTH_8U, 3);
	m_MMW_img = cvCreateImage(cvSize(PLANE_WIDTH, PLANE_HEIGHT), IPL_DEPTH_8U, 3);   //雷达点鸟瞰图
    cvZero(m_MMW_img_bak);
    cvZero(m_MMW_img);


    text = new char[260];
    //字体初始化
    cvInitFont(&cf, CV_FONT_HERSHEY_PLAIN, 0.8, 0.8, 0.0, 1);
    cvInitFont(&cf2, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0.0, 1);
    for(int i = 0; i<NUM; i++)
    {
    	MMW_detection_array[i].target_ID = 0;
    	MMW_detection_array[i].x = 0.0f;
    	MMW_detection_array[i].y = 0.0f;
    	MMW_detection_array[i].Relative_xv = 0.0f;
    	MMW_detection_array[i].Relative_yv = 0.0f;
    	MMW_detection_array[i].angle = 0.0f;
    	MMW_detection_array[i].Relative_acc_x = 0.0f;
    	MMW_detection_array[i].Obiect_class = 0;
    	MMW_detection_array[i].Object_Length = 0.0f;
    	MMW_detection_array[i].Object_Width =0.0f;

    }


    _empty_obj.target_ID = 0;
    _empty_obj.x = 0.0f;
    _empty_obj.y = 0.0f;
    _empty_obj.Relative_xv = 35.2778f; //发送默认速度,35.2778m/s = 127km/h
    _empty_obj.Relative_yv = 0.0f;
    _empty_obj.angle = 0.0f;
    _empty_obj.Relative_acc_x = 0.0f;
    _empty_obj.Obiect_class = 0;
    _empty_obj.Object_Length = 0.0f;
    _empty_obj.Object_Width = 0.0f;
     final_obj = _empty_obj;

     m_frameCount = 0;
    _count_num = 0;
    _recvNum = 0;

     fp_radar_file = fopen("1_radar_data_handle.txt","w");
     fp_vehicle_file = fopen("2_vehicle_data_handle.txt","w");
}

ObjectDetection::~ObjectDetection(void)
{

}

void ObjectDetection::draw_basic_info()
{
    //画车,用点代表本车
    cvCircle(m_MMW_img_bak, cvPoint(200, 600), 5, cvScalar(0, 0, 255), 2);

    //画距离图，五个像素=实际距离1米
#ifdef DISTANCE_TO_RADAR   //默认显示距离为到毫米波雷达的距离
    for (int h = 600-RADAR2CAR*METER2PIXEL, t = 0; (h-25) > 0; h -= 25, t += 5)   //画横线
    {
        cvLine(m_MMW_img_bak, cvPoint(0, h), cvPoint(400, h), cvScalar(122, 122, 122));
        sprintf(text, "%d m", t);
        cvPutText(m_MMW_img_bak, text, cvPoint(5, h - 2), &cf, cvScalar(0, 255, 255));
    }
#else
    for (int h = 500, t = 0; h > 0; h -= 25, t += 5)   //画横线
    {
        cvLine(m_Delphi_img_bak, cvPoint(0, h), cvPoint(400, h), cvScalar(122, 122, 122));
        sprintf(text, "%d m", t);
        cvPutText(m_Delphi_img_bak, text, cvPoint(5, h - 2), &cf, cvScalar(0, 255, 255));
    }
#endif
    for (int w = 50; w < 400; w += 50)   //画竖线
    {
        cvLine(m_MMW_img_bak, cvPoint(w, 0), cvPoint(w, 600), cvScalar(122, 122, 122));
    }

    int vertical_distance_line = 1.5;//绘制左右竖线距离线
    int pixel_dis = cvRound(vertical_distance_line*METER2PIXEL);
    cvLine(m_MMW_img_bak,cvPoint(200-pixel_dis,0),cvPoint(200-pixel_dis,600),CV_RGB(0,0,255));   //分辨率为0.2
    cvLine(m_MMW_img_bak,cvPoint(200+pixel_dis,0),cvPoint(200+pixel_dis,600),CV_RGB(0,0,255));


}

void ObjectDetection::set_radar_data(const continental_radar_target& radar)
{
	for (int i = 0 ; i < 64 ; i++)
	{
	  memset(&MMW_detection_array[i], 0, sizeof(moving_object_millimeter));
	}
	for (int i = 0; i < NUM; ++i)
{
    MMW_detection_array[i].target_ID= radar.continental_detection_array[i].target_ID;
    MMW_detection_array[i].Relative_xv = radar.continental_detection_array[i].Relative_xv;
    MMW_detection_array[i].Relative_yv = radar.continental_detection_array[i].Relative_yv;
    MMW_detection_array[i].v = sqrt(pow(radar.continental_detection_array[i].Relative_xv , 2) + pow(radar.continental_detection_array[i].Relative_yv, 2));
    MMW_detection_array[i].x = radar.continental_detection_array[i].x;
    MMW_detection_array[i].y = radar.continental_detection_array[i].y;
    MMW_detection_array[i].range = sqrt(pow(radar.continental_detection_array[i].x,2) + pow(radar.continental_detection_array[i].y,2));
    MMW_detection_array[i].angle = radar.continental_detection_array[i].angle;
	MMW_detection_array[i].Relative_acc_x = radar.continental_detection_array[i].Relative_acc_x;
	MMW_detection_array[i].Obiect_class = radar.continental_detection_array[i].Obiect_class;
	MMW_detection_array[i].Object_Length = radar.continental_detection_array[i].Object_Length;
	MMW_detection_array[i].Object_Width = radar.continental_detection_array[i].Object_Width;

#ifdef SAVE_GET_DATA
	/*---------------毫米波雷达数据保存--------------------*/

	/***********************************************************
		valid x y range  status moving moving_fast moving_slow
	***********************************************************/
	fprintf(fp_radar_file, "%d %.3f %.3f %.3f %.3f %.3f %.3f %d %.3f %.3f",
			radar.continental_detection_array[i].target_ID,
			radar.continental_detection_array[i].Relative_xv,
			radar.continental_detection_array[i].Relative_yv,
			radar.continental_detection_array[i].x,
			radar.continental_detection_array[i].y,
			radar.continental_detection_array[i].angle,
			radar.continental_detection_array[i].Relative_acc_x,
			radar.continental_detection_array[i].Obiect_class,
			radar.continental_detection_array[i].Object_Length,
			radar.continental_detection_array[i].Object_Width);

	/*---------------毫米波雷达数据保存--------------------*/
}
	fprintf(fp_radar_file, "\n");
#endif
}

void ObjectDetection::main_function2(vector<moving_object_millimeter> &vecObj_Temp)
{
    DrawCommonElements();
    //筛选雷达点
    vecObj_Temp.clear();   //清除向量

    for(int i = 0;i < NUM;i++)
    {
    	vecObj_Temp.push_back(MMW_detection_array[i]);

//    	if(MMW_detection_array[i].x > 40 || MMW_detection_array[i].y > 6 || MMW_detection_array[i].y < -6)
//    	{
//    		continue;
//    	}
//
//        double range = sqrt(pow(MMW_detection_array[i].x , 2) + pow(MMW_detection_array[i].y , 2));
//
//        if(range > 1.0)
//        {
//            vecObj_Temp.push_back(MMW_detection_array[i]);
//        }
    }

    if(!vecObj_Temp.empty())   //有毫米波检测目标
    {
        show_result(vecObj_Temp);   //有效目标点全部显示
    }

}

void ObjectDetection::show_result(vector<moving_object_millimeter>& valid_obj)   //显示全部有效目标
{
    CvPoint MMW_pos;
    for(vector<moving_object_millimeter>::iterator it = valid_obj.begin();it!=valid_obj.end();++it)
    {

    	//毫米波检测目标点在鸟瞰图上的坐标
    	MMW_pos.x = persMap_Middle - (*it).y*METER2PIXEL;
        MMW_pos.y = (PLANE_HEIGHT - 1) - ((*it).x + RADAR2CAR)*METER2PIXEL;   //图上的y向的正向是向下

        //鸟瞰图中显示白色点
        cvCircle(m_MMW_img,MMW_pos, 1, cvScalar(255,255, 255), 2);

        //显示目标径向相对速度，

        //附带显示其他属性信息
        //目标序号,v, moving
        sprintf(text,"%d %.3f",(*it).target_ID,(*it).Relative_xv);
        cvPutText(m_MMW_img,text,cvPoint(MMW_pos.x+2,MMW_pos.y),&cf,cvScalar(0,255,255));

    }

}

void ObjectDetection::DrawCommonElements()
{
	cvCopy(m_MMW_img_bak,m_MMW_img);   //每次都要清除上一次绘制的鸟瞰图

}
void ObjectDetection::DisplayAll()
{
	cvNamedWindow("MMW_image",CV_WINDOW_NORMAL);
    cvShowImage("MMW_image", m_MMW_img);
    cvWaitKey(10);
}
