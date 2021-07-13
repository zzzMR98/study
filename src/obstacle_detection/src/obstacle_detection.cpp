/*
 * obstacle_detection.cpp
 *
 *  Created on: 2018年5月4日
 *      Author: zhubc
 */
#include "obstacle_detection.h"

/*
 * \brief 高度差检测
 * \param pointcloud 输入点云
 * \param heightdiffpointcloud 输出点云
 * \param ogm_data 栅格地图
 * \param resolution 栅格分辨率
 * \param heightdiffthreshold 高度差阈值
 * \param countthreshold 栅格中符合特征点的个数要求
 * \param definition 栅格中符合高度差阈值对应的状态
 * 
 * countthreshold 点云数量阈值，最高最低点数量大于阈值时才算障碍
 */

void Obstacle_Detection::heightdiffOgmDetection(Cloud& pointcloud,Cloud& heightdiffpointcloud
		,OGMData<unsigned char>& ogm_data
		,double resolution,double heightdiffthreshold,int countthreshold,int definition)
{
    //这个OGMData是一个类吗 找不到定义 zmr?
	double vehicle_x = ogm_data.vehicle_x;
	double vehicle_y = ogm_data.vehicle_y;
	OGMData<float> minzogm(ogm_data.ogmheight,ogm_data.ogmwidth,resolution);
	static unsigned int* passible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	static unsigned int* nopassible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	memset(passible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size); //内存中的值全部设为0 zmr
	memset(nopassible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
	//网格初始化
	//ogm_data.ogm属性有unknown,rigidnopassable和passable

	//    memset(ogm_data.ogm,0,ogm_data.ogmcell_size);
	for(int i = 0;i<minzogm.ogmcell_size;i++)
	{
		minzogm.ogm[i]=1000; //为什么要设成1000？初始化？ yes zmr
	}


	if(1){
		//近处网格，用大网格分析高度差，然后投影到小网格中去。对于远距离马路沿子有较好结果，但是噪声可能产生干扰
		for (int i = 0; i < pointcloud.points.size(); i++){
			if(pointcloud.points[i].range < 0.5) //？ zmr? 去除距离较近(打在车上)的点
				continue;
			float x = pointcloud.points[i].x,
					y = pointcloud.points[i].y,
					z = pointcloud.points[i].z;
			int intensity  = pointcloud.points[i].intensity;
			float newy = y + vehicle_y;//整体往后移动了ogm_y_offset
			float newx = x + vehicle_x;

			if((newx >=0  && newx <= minzogm.ogmwidth) &&
					(newy >=0 && newy < minzogm.ogmheight) &&
					(z >= - 2 && z <=  Z_MAX)){

				int col = boost::math::round(newx / minzogm.ogmresolution) ;
				int row = boost::math::round(newy / minzogm.ogmresolution) ;

				if((row >=0 && row < minzogm.ogmheight_cell)
						&& (col >=0 && col < minzogm.ogmwidth_cell)){
					int index = row * minzogm.ogmwidth_cell + col;
					if( minzogm.ogm[index] >  z)
						minzogm.ogm[index] = z;   //遍历点云，找到每个栅格z最小值 zmr
				}
			}

		}

		for (int i = 0; i < pointcloud.points.size(); i++){
			if(pointcloud.points[i].range < 0.5)
				continue;
			float x = pointcloud.points[i].x,
					y = pointcloud.points[i].y,
					z = pointcloud.points[i].z;

			float newy = y + vehicle_y;//整体往下移动了ogm_y_offset米
			float newx = x + vehicle_x;

			if((newx >= 0  && newx <= ogm_data.ogmwidth) &&
					(newy >=0 && newy < ogm_data.ogmheight) &&
					(z >= - 1.5 && z <=  Z_MAX)){

				int col = boost::math::round(newx / minzogm.ogmresolution) ;
				int row = boost::math::round(newy / minzogm.ogmresolution) ;

				if((row >=0 && row < minzogm.ogmheight_cell)
						&& (col >=0 && col < minzogm.ogmwidth_cell)){

					int index = row * minzogm.ogmwidth_cell + col;

					if(z -  minzogm.ogm[index] > heightdiffthreshold){

						heightdiffpointcloud.push_back(pointcloud.points[i]); //赋值 zmr
						heightdiffpointcloud.back().passibility = 0; //? zmr? 未知？
						int col1 = boost::math::round(x / ogm_data.ogmresolution) + ( ogm_data.ogmwidth_cell - 1 ) / 2;
						int row1 = boost::math::round(newy / ogm_data.ogmresolution) ; //投影到小网格里 zmr

						int index1 = row1 * ogm_data.ogmwidth_cell + col1;
						//                        ogm_data.ogm[index1] = RIGIDNOPASSABLE;
						nopassible_point_count_ogm_count[index1]++;
					}
					else
					{

						int col1 = boost::math::round(x / ogm_data.ogmresolution) + ( ogm_data.ogmwidth_cell - 1 ) / 2;
						int row1 = boost::math::round(newy / ogm_data.ogmresolution) ;
						int index1 = row1 * ogm_data.ogmwidth_cell + col1;
						passible_point_count_ogm_count[index1]++;
					}
				}
			}
		}
		for(int i = 0; i < ogm_data.ogmcell_size;i++)
		{
			if(nopassible_point_count_ogm_count[i]>countthreshold && passible_point_count_ogm_count[i]>countthreshold) //? zmr?
				ogm_data.ogm[i] = definition;
			else if (nopassible_point_count_ogm_count[i]==0 && passible_point_count_ogm_count[i]>countthreshold) {
				ogm_data.ogm[i] = PASSABLE;
			}
		}
		// for(int i = 0;i < heightdiffpointcloud.points.size(); i++)
		// {
		// 	if(heightdiffpointcloud.points[i].passibility <=0.1)
		// 	{
		// 		float x = heightdiffpointcloud.points[i].x, y = heightdiffpointcloud.points[i].y,z = heightdiffpointcloud.points[i].z;
		// 		float newy = y + vehicle_y;//整体往下移动了ogm_y_offset米
		// 		float newx = x + vehicle_x;
		// 		if((newx >= 0  && newx <= ogm_data.ogmwidth) &&(newy >=0 && newy < ogm_data.ogmheight)
		// 				&&	(z >= - 1.5 && z <=  Z_MAX))
		// 		{
		// 			int col_2 = boost::math::round(x / ogm_data.ogmresolution) + ( ogm_data.ogmwidth_cell - 1 ) / 2;
		// 			int row_2 = boost::math::round(newy / ogm_data.ogmresolution) ;
		// 			int index_2 =  row_2 * ogm_data.ogmwidth_cell + col_2;
		// 			if(ogm_data.ogm[index_2]==definition)
		// 			{
		// 				heightdiffpointcloud.points[i].passibility = 0;
		// 			}
		// 			else
		// 			{
		// 				heightdiffpointcloud.points[i].passibility = 1;
		// 			}

		// 		}
		// 	}
		// }
	}
	ogm_data.updateStamp();
	// delete [] passible_point_count_ogm_count;
	// delete [] nopassible_point_count_ogm_count;
	//  pointcloud = *pointcloud;

}

/*!
 *
 * \brief 将毫米波雷达检测目标转换成栅格地图
 * \param pointcloud 输入毫米波雷达目标
 * \param ogm_data 栅格地图
 *
 */
void Obstacle_Detection::obstacleDetectionForMMWRadar( const Radarpose &pose_xyz,OGMData<unsigned char>& ogm_data )
{
    memset(ogm_data.ogm,0,ogm_data.ogmcell_size);
    for (int i = 0; i < pose_xyz.size(); i++)
    {
        float x = pose_xyz[i].position.x,
        y = pose_xyz[i].position.y,
        z = pose_xyz[i].position.z;
        float newx = x + ogm_data.vehicle_x;
        float newy = y + ogm_data.vehicle_y;	  //栅格地图中的实际y值
        //判断是否在栅格地图内部
        if ((newx >= 0 && newx < ogm_data.ogmwidth)
            && (newy >= 0 && newy < ogm_data.ogmheight))
        {
            int col = boost::math::round(
                    newx / ogm_data.ogmresolution);			  //横向栅格
            int row = boost::math::round(
                    newy / ogm_data.ogmresolution);		  //纵向栅格
            if ((row >= 0 && row < ogm_data.ogmheight_cell)
                && (col >= 0 && col < ogm_data.ogmwidth_cell))
            {
                int index = row * ogm_data.ogmwidth_cell + col;
                //将目标所在的栅格标记为正障碍
                ogm_data.ogm[index] = RIGIDNOPASSABLE;
            }
        }
    }

}

/**
 * @brief 在地面是平面的情况下，将高于lower_height_bound的点都认为是障碍物，同时用hyper_height_bound排除悬空障碍物
 * 
 * @param pointcloud 输入点云
 * @param ogm_data 栅格地图
 * @param resolution 栅格分辨率
 * @param lower_height_bound 高度下限阈值
 * @param hyper_height_bound 高度上限阈值
 * @param heightdiffthreshold 较小的高度差
 * @param definition 对应较小高度差的障碍物类别
 * @param heightdiffhigherthreshold 较大的高度差
 * @param definition2 对应较大高度差的障碍物类别
 */
void Obstacle_Detection::obstacleDetectionForFlatFloor(Cloud& pointcloud,OGMData<unsigned char>& ogm_data,
		double resolution,float lower_height_bound,float hyper_height_bound,double heightdiffthreshold,int definition, 
		double heightdiffhigherthreshold,int definition2)
{

	double vehicle_x = ogm_data.vehicle_x;
	double vehicle_y = ogm_data.vehicle_y;
	OGMData<float> maxzogm(ogm_data.ogmheight,ogm_data.ogmwidth,resolution); 
	OGMData<float> minzogm(ogm_data.ogmheight,ogm_data.ogmwidth,resolution);
	for(int i = 0;i<minzogm.ogmcell_size;i++)
	{
		minzogm.ogm[i]=1000;
	}
	for (int i = 0; i < pointcloud.points.size(); i++){
		if(pointcloud.points[i].range < 0.5)
			continue;
		float x = pointcloud.points[i].x,
				y = pointcloud.points[i].y,
				z = pointcloud.points[i].z;
		if ((x*x + y*y) > 20000) continue;
		// int intensity  = pointcloud.points[i].intensity;
		if(z > lower_height_bound) {
			float newy = y + vehicle_y;//整体往后移动了ogm_y_offset
			float newx = x + vehicle_x;

			int col = boost::math::round(newx / maxzogm.ogmresolution) ;
			int row = boost::math::round(newy / maxzogm.ogmresolution) ;

			if((row >=0 && row < maxzogm.ogmheight_cell)
					&& (col >=0 && col < maxzogm.ogmwidth_cell)){
				int index = row * maxzogm.ogmwidth_cell + col;
				if( maxzogm.ogm[index] < z) maxzogm.ogm[index] = z;
				if (minzogm.ogm[index] > z) minzogm.ogm[index] = z;
			}
		}
	}

	for(int i = 0; i < ogm_data.ogmcell_size;i++)
	{
		if(maxzogm.ogm[i] < hyper_height_bound) {
			if (maxzogm.ogm[i]>heightdiffhigherthreshold)
				ogm_data.ogm[i] = definition2;
			else if(maxzogm.ogm[i]>heightdiffthreshold)
				ogm_data.ogm[i] = definition;
		}
	}
	ogm_data.updateStamp();
}

/*!
 * \brief 道路边沿检测（越野环境下误检太多，没有用到）
 * \param pointcloud 输入点云
 * \param borderpointcloud 输出点云
 * \param ogm_data 栅格地图
 * \param LASER_LAYER 点云线数
 * \param x_offset 雷达在车体坐标系的x
 * \param y_offset 雷达在车体坐标系的y
 */
void Obstacle_Detection::border_detection(const Cloud& pointcloud,Cloud& borderpointcloud,
		OGMData<unsigned char>& ogm_data,int LASER_LAYER,double x_offset,double y_offset)
{

	//路沿检测函数
	//对于点云的初始化，将其全视作可通行区域

	//雷达参数
	int col_count = pointcloud.size() / LASER_LAYER;

	//路沿检测参数
	float delta_angle_thresh = 60;//adjustable param 路沿检测的角度阈值
	int cloud_window_size = 8;//adjustable param 相邻的帧数
	if(1)
	{
		for(int i = 0 ; i < LASER_LAYER ; i++)
		{
			//这里是将每一线的角度转换成弧度，加上beta是对垂直角度偏移量的一个修正，根据雷达安装的情况

			for(int j = 0 ; j < col_count - cloud_window_size; j++)
			{
				//第i线雷达的数据
				int index = j * LASER_LAYER + i;

				if(pointcloud.points[index].range < 1)
					continue;
				if(pointcloud.points[index].x < 2 && pointcloud.points[index].x > -2)
					continue;
				if(pointcloud.points[index].z > 0.1)
					continue;

				//这个方位角是正切，是对于其切线的斜率的求取（好像没有什么用）

				int index1 = (j + cloud_window_size) * LASER_LAYER + i;

				if(pointcloud.points[index1].range < 1)
					continue;
				//第一个点的切线角度expected_tangential_angle
				float slope_angle = atan2(pointcloud.points[index].y-y_offset, pointcloud.points[index].x-x_offset) * 180 / M_PI;
				float expected_tangential_angle = slope_angle - 90; //这是在雷达坐标系下去比较角度 zmr
				//������+-pi
				if(expected_tangential_angle > 180) expected_tangential_angle -= 360;
				else if(expected_tangential_angle <= -180) expected_tangential_angle += 360;
				//第十帧的点与第一帧的点连线的斜率
				float actual_tangential_angle = atan2(pointcloud.points[index1].y - pointcloud.points[index].y,
						pointcloud.points[index1].x - pointcloud.points[index].x) * 180 / M_PI;

				//这里就是用前后大约是10帧的点进行判断，用第一个点的切线与1点与10点之间连线的斜率进行比较，如果差距比较大，则判断是路沿
				//实际上路沿就是这种检测效果，实际点云当检测到路沿的时候就凹陷下去一块，之后就又弯回来，然后就又正常，因此比较这个角度应该可以判断ZhuBC
				float delta_angle = actual_tangential_angle - expected_tangential_angle;
				/*********************************************************20180420slope detection******************/
				double delta_range;
				float index1_dis = (pointcloud.points[index1].y) *  (pointcloud.points[index1].y) + (pointcloud.points[index1].x) * (pointcloud.points[index1].x) + (pointcloud.points[index1].z) * (pointcloud.points[index1].z) ;
				float index_dis = (pointcloud.points[index].y) * (pointcloud.points[index].y) + (pointcloud.points[index].x) * (pointcloud.points[index].x) + (pointcloud.points[index].z) * (pointcloud.points[index].z);
				delta_range = sqrt(fabs((double)index1_dis - (double)index_dis));
				/*********************************************************20180420slope detection******************/
				if(delta_angle > 180) delta_angle -= 360;
				else if(delta_angle < -180) delta_angle += 360;

				if(delta_angle>90)
					delta_angle = 180 - delta_angle;
				//如果大于阈值，则认为是路沿
				if(fabs(delta_angle) > delta_angle_thresh && delta_range > 1.0 && delta_range < 2)
				{
					borderpointcloud.push_back(pointcloud.points[index]);
					borderpointcloud.back().passibility = 0.;
					//					pointcloud.points[index].passibility = 0.0;
				}

			}

		}
	}

	//生成栅格地图并将passibility=0的点放在栅格里面
	if(1)
	{
		memset(ogm_data.ogm,0,ogm_data.ogmcell_size);
		unsigned int* passible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
		unsigned int* nopassible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
		memset(passible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
		memset(nopassible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
		float ogm_y_offset = 20.0f;//对于y的偏移量，车体坐标系转成栅格地图的变化

		for (int i = 0; i < borderpointcloud.points.size(); i++)
		{
			float x = borderpointcloud.points[i].x,
					y = borderpointcloud.points[i].y,
					z = borderpointcloud.points[i].z;

			float newy = y + ogm_y_offset;//栅格地图中的实际y值
			//判断是否在栅格地图内部
			if((x >=-ogm_data.ogmwidth/2 &&x <= ogm_data.ogmwidth/2 ) && (newy >=0 && newy < ogm_data.ogmheight) && (z >= - 1.5 && z <=  Z_MAX))
			{
				int col = boost::math::round(x / ogm_data.ogmresolution) + ( ogm_data.ogmwidth_cell - 1 ) / 2;//横向栅格
				int row = boost::math::round(newy / ogm_data.ogmresolution) ;//纵向栅格

				if((row >=0 && row < ogm_data.ogmheight_cell) && (col >=0 && col < ogm_data.ogmwidth_cell))
				{
					int index = row * ogm_data.ogmwidth_cell + col;
					//如果为之前检测到的路沿，则计数器加1
					if(borderpointcloud.points[i].passibility < 0.5)
						nopassible_point_count_ogm_count[index]++;
					else
						passible_point_count_ogm_count[index]++;
				}
			}
		}
		int nopassible_count_thresh = 5;//同一个栅格内的5个点都是那种状态才满足。
		for(int i = 0 ; i < ogm_data.ogmcell_size ; i++)
		{
			if(nopassible_point_count_ogm_count[i] > nopassible_count_thresh)
			{
				//同一个点连续扫描五次，如果都是障碍物的话，则判断为是不可通行区域ZhuBC
				ogm_data.ogm[i] = NEGATIVENOPASSABLE;
			}
			else
			{
				ogm_data.ogm[i] = PASSABLE;
			}
		}
		for (int i = 0; i < borderpointcloud.points.size(); i++)
		{
			float x = borderpointcloud.points[i].x,
					y = borderpointcloud.points[i].y,
					z = borderpointcloud.points[i].z;

			float newy = y + ogm_y_offset;//栅格地图中的实际y值
			//判断是否在栅格地图内部
			if((x >=-ogm_data.ogmwidth/2 &&x <= ogm_data.ogmwidth/2 ) && (newy >=0 && newy < ogm_data.ogmheight) && (z >= - 1.5 && z <=  Z_MAX))
			{
				int col = boost::math::round(x / ogm_data.ogmresolution) + ( ogm_data.ogmwidth_cell - 1 ) / 2;//横向栅格
				int row = boost::math::round(newy / ogm_data.ogmresolution) ;//纵向栅格

				if((row >=0 && row < ogm_data.ogmheight_cell) && (col >=0 && col < ogm_data.ogmwidth_cell))
				{
					int index = row * ogm_data.ogmwidth_cell + col;
					//如果为之前检测到的路沿，则计数器加1
					if(ogm_data.ogm[index] != RIGIDNOPASSABLE)
						borderpointcloud.points[i].passibility = 1;
				}
			}
		}
		delete [] passible_point_count_ogm_count;
		delete [] nopassible_point_count_ogm_count;
	}

}
/*
 *
 * \brief 将点云转换成栅格地图
 * \param pointcloud 输入点云
 * \param ogm_data 点云对应的栅格地图
 * \param countthreshold 每个栅格中特征点个数（大于此阈值才认为此栅格具有这种属性）
 *
 * countthreshold 点云数量阈值，障碍点数量大于阈值时才算障碍
 */
void Obstacle_Detection::cloud2OGMForRsLidar(const Cloud& pointcloud,OGMData<unsigned char>& ogm_data,int countthreshold)
{
	memset(ogm_data.ogm,0,ogm_data.ogmcell_size);
	unsigned int* passible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	unsigned int* nopassible_point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	unsigned int* nopassible_point_count_negative_ogm_count = new unsigned int[ogm_data.ogmcell_size];
	memset(passible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
	memset(nopassible_point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
	memset(nopassible_point_count_negative_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);
	for (int i = 0; i < pointcloud.size(); i++) {
		float x = pointcloud.points[i].x
				, y =pointcloud.points[i].y
				, z =pointcloud.points[i].z;
		float newx = x + ogm_data.vehicle_x;
		float newy = y + ogm_data.vehicle_y;	  //栅格地图中的实际y值
		//判断是否在栅格地图内部
		if ((newx >= 0
				&& newx < ogm_data.ogmwidth)
				&& (newy >= 0 && newy < ogm_data.ogmheight)
				&& (z >= -1.5 && z <= Z_MAX)) {
			int col = boost::math::round(
					newx / ogm_data.ogmresolution);			  //横向栅格
			int row = boost::math::round(
					newy / ogm_data.ogmresolution);		  //纵向栅格
			if ((row >= 0 && row < ogm_data.ogmheight_cell)
					&& (col >= 0 && col < ogm_data.ogmwidth_cell)) {
				int index = row * ogm_data.ogmwidth_cell + col;
				//如果为之前检测到的路沿，则计数器加1
				if (pointcloud.points[i].passibility < 0.5)
					nopassible_point_count_ogm_count[index]++;
				//負障碍物 
				else if(pointcloud.points[i].passibility > 3.1 && pointcloud.points[i].passibility < 4.1) 
					nopassible_point_count_negative_ogm_count[index]++;
				else
					passible_point_count_ogm_count[index]++;
			}
		}
	}
	for(int i = 0 ; i < ogm_data.ogmcell_size; i++)
	{
		if(nopassible_point_count_ogm_count[i] > countthreshold && nopassible_point_count_negative_ogm_count[i] < countthreshold)
		{
			//同一个点连续扫描五次，如果都是障碍物的话，则判断为是不可通行区域ZhuBC
			ogm_data.ogm[i] = RIGIDNOPASSABLE;
		}
		else if(nopassible_point_count_negative_ogm_count[i] > countthreshold && nopassible_point_count_ogm_count[i] < countthreshold)
		{
			ogm_data.ogm[i] = FIRSTNEGATIVENOPASSABLE;
		}
		else if(nopassible_point_count_negative_ogm_count[i] > countthreshold && nopassible_point_count_ogm_count[i] > countthreshold)
		{
			ogm_data.ogm[i] = RIGIDNOPASSABLE;
		}
		else if(passible_point_count_ogm_count[i] > countthreshold)
			ogm_data.ogm[i] = PASSABLE;
	}
	for (int i = 0; i < pointcloud.size(); i++) {
		float x = pointcloud.points[i].x
				, y =pointcloud.points[i].y
				, z =pointcloud.points[i].z;
		float newx = x + ogm_data.vehicle_x;
		float newy = y + ogm_data.vehicle_y;	  //栅格地图中的实际y值
		//判断是否在栅格地图内部
		if ((newx >= 0
				&& newx < ogm_data.ogmwidth)
				&& (newy >= 0 && newy < ogm_data.ogmheight)
				&& (z >= -1.5 && z <= Z_MAX)) {
			int col = boost::math::round(
					newx / ogm_data.ogmresolution);			  //横向栅格
			int row = boost::math::round(
					newy / ogm_data.ogmresolution);		  //纵向栅格
			if ((row >= 0 && row < ogm_data.ogmheight_cell)
					&& (col >= 0 && col < ogm_data.ogmwidth_cell))
			{
				int index = row * ogm_data.ogmwidth_cell + col;
				//如果为之前检测到的路沿，则计数器加1
				if(ogm_data.ogm[index] ==  FIRSTNEGATIVENOPASSABLE)
				{
					ogm_data.ogm[index] = NEGATIVENOPASSABLE;
					int count = pointcloud.points[i].intensity / ogm_data.ogmresolution;
					//					cout<<"count ==="<<count<<endl;
					//					cout<<"pointcloud.points[i].intensity = "<<pointcloud.points[i].intensity<<endl;
					for(int k = 1; k < count + 1; k ++)
					{
						for(int d = -3; d < 4;d++)
						{
							if(row - k < 0)
								continue;
							int new_count = (row - k) * ogm_data.ogmwidth_cell + col + d;
							//						cout<<"row - k = "<< row - k<<endl;
							//						cout<<"pointcloud.points[i].intensity = "<<pointcloud.points[i].intensity<<endl;
							ogm_data.ogm[new_count] = NEGATIVENOPASSABLE;
						}
					}
				}

			}
		}
	}
	delete [] passible_point_count_ogm_count;
	delete [] nopassible_point_count_ogm_count;
	delete[] nopassible_point_count_negative_ogm_count;
}

/*!
 * \brief 将每个栅格中点的个数大于阈值的栅格做好标记（已知区域）
 * \param pointcloud 输入点云
 * \param ogm_data 点云对应的栅格地图
 * \param countthreshold 栅格中点的个数的阈值
 */
void Obstacle_Detection::purecloud2OGM(const Cloud& pointcloud, OGMData<unsigned char>& ogm_data, int countthreshold)
{

	//	memset(ogm_data.ogm,0,ogm_data.ogmcell_size);
	unsigned int* point_count_ogm_count = new unsigned int[ogm_data.ogmcell_size];

	memset(point_count_ogm_count,0,sizeof(int)*ogm_data.ogmcell_size);

	for (int i = 0; i < pointcloud.size(); i++) {
		float x = pointcloud.points[i].x
				, y =pointcloud.points[i].y
				, z =pointcloud.points[i].z;
		float newx = x + ogm_data.vehicle_x;
		float newy = y + ogm_data.vehicle_y;	  //栅格地图中的实际y值
		//判断是否在栅格地图内部
		if ((newx >= 0
				&& newx < ogm_data.ogmwidth)
				&& (newy >= 0 && newy < ogm_data.ogmheight)
				&& (z >= -1.5 && z <= Z_MAX)) {
			int col = boost::math::round(
					newx / ogm_data.ogmresolution);			  //横向栅格
			int row = boost::math::round(
					newy / ogm_data.ogmresolution);		  //纵向栅格
			if ((row >= 0 && row < ogm_data.ogmheight_cell)
					&& (col >= 0 && col < ogm_data.ogmwidth_cell)) {
				int index = row * ogm_data.ogmwidth_cell + col;
				

				point_count_ogm_count[index]++;

			}
		}
	}
	for(int i = 0 ; i < ogm_data.ogmcell_size; i++)
	{
		if(point_count_ogm_count[i] > countthreshold)
		{
			//同一个点连续扫描五次，如果都是障碍物的话，则判断为是不可通行区域ZhuBC ??zmr?
			ogm_data.ogm[i] = PASSABLE;
		}
	}
	delete [] point_count_ogm_count;
}


/*!
 * \brief 针对于velodyne32线雷达的负障碍检测
 * \param pointcloud 输入点云
 * \param LASER_LAYER 雷达线数
 */
void Obstacle_Detection::negativeDetection32(Cloud& pointcloud, int LASER_LAYER)
{
	///////可以滤出一些距离不太正常的点也可以是相邻两线之间差的比较多的点。1228傍晚记
	//负障碍检测 参数设置
	memset(polaraxismat_ , 0 ,  sizeof(PolarPointDI)*LASER_LAYER*3601);//0506修改,防止越界
	int col_count = pointcloud.size() / LASER_LAYER; //每圈的点数
	float deltaz_thresh = 0.3;//adjustable param //0.3
	float negative_deltaz_thresh=0.3;//adjustable param
	float slope_thresh = tan(25* M_PI / 180);// adjustable param //30
	double Z_Ground = 0.3;//adjustable param
	for(int i = 0;i<pointcloud.size();i++)
	{
		//    	pointcloud.points[i].passibility = 0;
		float azimuth = atan2(pointcloud.points[i].y,pointcloud.points[i].x)*180/M_PI;
		while(azimuth > 360.0) azimuth -= 360;
		while(azimuth <=0) azimuth += 360;
		pointcloud.points[i].azimuth = azimuth;
		int mat_i = azimuth * 10;
		PolarPointDI temppolarpoint;
		temppolarpoint.distance = pointcloud.points[i].range;
		temppolarpoint.index = i;
		polaraxismat_[i%LASER_LAYER][mat_i] = temppolarpoint;
	}
	for(int i = 0 ; i < col_count ; i++)
	{
		for(int j = 0 ; j < LASER_LAYER/**2/3*/ ; j++)
		{
			int ori_index =i * LASER_LAYER + grabber_H_.indexmaptable[j].number;//第i组数据的第j线的编号
			//将一些不必要的点过滤掉
			if(pointcloud.points[ori_index].range < 0.5|| pointcloud.points[ori_index].range>60)
				continue;

			if(pointcloud.points[ori_index].y< -20||pointcloud.points[ori_index].x< -5||pointcloud.points[ori_index].x > 5) //back exist false detection , so need delete it
				continue;

			if(pointcloud.points[ori_index].z< Z_MIN||pointcloud.points[ori_index].z > Z_MAX) //back exist false detection , so need delete it
				continue;
			if(pointcloud.points[ori_index].y < -0.5)
				continue;

			if(pointcloud.points[ori_index].z > Z_Ground)
				continue;
			int col_delta = -5 * pointcloud.points[ori_index].range + 95;

			if(1)
			{
				/******************************************0522被ZhuBC封印 没有用到******************************************************************/
				//            	double tempazimuthj=temppointj.azimuth ;
				//            	int mat_i=tempazimuthj * 10;//这里单位是0.1度，放大了10倍，水平角度的范围是0到3600
				//            	int mat_i_start=mat_i-2;
				//            	if (mat_i_start<0)
				//            		mat_i_start=0;
				//            	int mat_i_stop=mat_i+2;
				//            	if 	(mat_i_stop>3599)
				//            		mat_i_stop=3599;
				//            	int disminindex=-1;
				//            	int disminindex_j=-1;
				//            	int disminmat_i=mat_i_start;
				//            	double dismin = 1000;
				//            	double dismax = 0;
				//            	for(int temp_i = mat_i_start ; temp_i<=mat_i_stop;temp_i++)
				//            	{
				//            		double tempdis= polaraxismat_[index_t][temp_i].distance;//极坐标
				//            		int index_temp = polaraxismat_[index_t][temp_i].index;
				//            		if(tempdis<0.5)
				//            			continue;
				//            		if(pointcloud.points[index_temp].passibility < 0.5)
				//            			continue;
				//            		//取一周中最近的那个点，由于64线雷达有时候不是同一个垂直角的，所以取最近的作为正常的点来用。
				//            		if  (dismin > tempdis)
				//            		{
				//            			dismin = tempdis;
				//            			disminindex=polaraxismat_[index_t][temp_i].index;
				//            			disminmat_i=temp_i;
				//                 	}
				//            	}
				//
				//            	if(disminindex < 0)
				//            		continue;
				//            	ori_index1=disminindex;
				//
				//            	temppointt=pointcloud.points[ori_index1];//这个是正常的那个值。
				/******************************************0522被ZhuBC封印 没有用到******************************************************************/
				if(LASER_LAYER == 32)//32线
				{
					int front_layer;//j+1线
					int back_layer;//j-1线
					if(j ==0)
					{
						back_layer = LASER_LAYER - 1;
						front_layer = j + 1;
					}
					else if(j == LASER_LAYER - 1)
					{
						back_layer = j - 1;
						front_layer = 0;
					}
					else
					{
						back_layer = j - 1;
						front_layer = j + 1;
					}
					int back_point, front_point;//i-40点,i+40点
					if(i < col_delta)
					{
						back_point = i - col_delta + col_count;
						front_point = i + col_delta;
					}
					else if (i > col_count - col_delta - 1)
					{
						back_point = i - col_delta;
						front_point = i + col_delta - col_count;
					}
					else
					{
						back_point = i - col_delta;
						front_point = i + col_delta;
					}
					int ori_index1 =i * LASER_LAYER + grabber_H_.indexmaptable[front_layer].number;//第i组数据的第j+1线的数据
					int ori_index2 = i * LASER_LAYER + grabber_H_.indexmaptable[back_layer].number;//第i组数据的第j-1线的数据
					int ori_index3 = back_point * LASER_LAYER + grabber_H_.indexmaptable[j].number;//第i-40组数据的第j线的数据
					int ori_index4 = front_point * LASER_LAYER + grabber_H_.indexmaptable[j].number;//第i+40组数据的第j线的数据
					if(pointcloud.points[ori_index3].range < 0.5 || pointcloud.points[ori_index4].range < 0.5 || pointcloud.points[ori_index1].range < 0.5 || pointcloud.points[ori_index2].range < 0.5)//0506滤出一些车体范围内的误检点.
						continue;
					//height delta
					double front_delta_z = pointcloud.points[ori_index1].z - pointcloud.points[ori_index].z;//第j+1线和第j线点的z差
					double back_delta_z = pointcloud.points[ori_index2].z - pointcloud.points[ori_index].z;//第j线和第j-1线点的z差
					//delta range
					double front_delta_range = pointcloud.points[ori_index1].range - pointcloud.points[ori_index].range;//第j+1线和第j线点的range差
					double back_delta_range = pointcloud.points[ori_index].range - pointcloud.points[ori_index2].range;//第j线和第j-1线点的range差
					double last_delta_range = fabs(pointcloud.points[ori_index].range - pointcloud.points[ori_index3].range);//第j线i点和i-40的range差
					double next_delta_range = fabs(pointcloud.points[ori_index4].range - pointcloud.points[ori_index].range);//第j线i+40和i点的range差
					int index_farthest = 0;
					int index_nearest = 0;
					int flag = 0;
					if(pointcloud.points[ori_index].range > 13 && pointcloud.points[ori_index].range < 17 )//set parameters according to distance(>10)
					{
						if(front_delta_z > 0.02 && (back_delta_range / front_delta_range) > 1.5)
						{
							if(back_delta_range > 0.7 && back_delta_range < 3 && back_delta_z > 0.1 && back_delta_z < 0.25)
							{
								if((next_delta_range < 1.2 && next_delta_range > 0.3) && ( last_delta_range < 1.2 && last_delta_range > 0.3))
								{
									pointcloud.points[ori_index1].passibility = 4.0;
									pointcloud.points[ori_index].passibility = 4.0;
									pointcloud.points[ori_index2].passibility = 4.0;
								}
								else //if((next_delta_range < 1.2 && next_delta_range > 0.3) || ( last_delta_range < 1.2 && last_delta_range > 0.3))
								{
									double theta1 = fabs((pointcloud.points[ori_index].y - pointcloud.points[ori_index3].y) / (pointcloud.points[ori_index].x - pointcloud.points[ori_index3].x));
									double theta2 = fabs((pointcloud.points[ori_index].y - pointcloud.points[ori_index4].y) / (pointcloud.points[ori_index].x - pointcloud.points[ori_index4].x));
									if(fabs(theta1 - theta2) < 3)
									{
										pointcloud.points[ori_index1].passibility = 4.0;
										pointcloud.points[ori_index].passibility = 4.0;
										pointcloud.points[ori_index2].passibility = 4.0;
									}
								}
							}
						}
					}
					else if(pointcloud.points[ori_index].range <= 13 && pointcloud.points[ori_index].range >= 6)
					{
						if(front_delta_z > 0.02 && front_delta_z < 0.2 && front_delta_range < 0.4 && front_delta_range > 0.01)
						{
							if(back_delta_range > 0.7 && back_delta_range < 2.5 && back_delta_z > 0.1 && back_delta_z < 0.25)
							{
								if((next_delta_range < 1.2 && next_delta_range > 0.4) && (last_delta_range < 1.2 && last_delta_range > 0.4))
								{
									pointcloud.points[ori_index1].passibility = 4.0;
									pointcloud.points[ori_index].passibility = 4.0;
									pointcloud.points[ori_index2].passibility = 4.0;
								}
							}
						}
					}
				}
			}
		}
	}
}

/*!
 * \brief 对负障碍检测得到的栅格地图进行图像膨胀处理
 * \param inputogm 输入栅格
 * \param outputogm 输出栅格
 */
void Obstacle_Detection::ogmDilation(OGMData<unsigned char>& inputogm, OGMData<unsigned char>& outputogm)
{
	int minheight = 500,minwidth = 500,maxheight = 0,maxwidth = 0;
	for(int i = 0; i < inputogm.ogmheight_cell; i ++)
	{
		for(int j = 1; j < inputogm.ogmwidth_cell; j ++)
		{
			if(inputogm.ogm[j + i * inputogm.ogmwidth_cell] == NEGATIVENOPASSABLE)
			{
				if(inputogm.ogm[j - 1 + i * inputogm.ogmwidth_cell] == NEGATIVENOPASSABLE || inputogm.ogm[j + 1 +i * inputogm.ogmwidth_cell] == NEGATIVENOPASSABLE)
				{
					if(i < minheight)
						minheight = i;
					if(j < minwidth)
						minwidth = j - 1;
					if(i > maxheight)
						maxheight = i;
					if(j > maxwidth)
						maxwidth = j + 1;
				}
				else
					inputogm.ogm[j + i * inputogm.ogmwidth_cell] = PASSABLE;
			}
		}
	}
	if(minheight != 500 && minwidth != 500 && maxheight != 0 && maxwidth != 0 && (maxheight - minheight) < 15 && (maxwidth - minwidth) < 25 && minwidth > 1 && maxwidth >2)
	{
		for(int i = minheight; i < maxheight + 1; i ++)
		{
			for(int j = minwidth; j < maxwidth + 1; j ++)
			{
				inputogm.ogm[j + i * inputogm.ogmwidth_cell] = NEGATIVENOPASSABLE;
			}
		}
	}
	outputogm = inputogm;
}

/*!
 * \brief 将栅格地图显示出来
 * \param windowname 窗口名称
 * \param ogmdata 栅格地图
 */
void Obstacle_Detection::showOGM(const char* windowname , const OGMData<unsigned char>& ogmdata)
{
	cvNamedWindow(windowname,0);
	CvMat *slopemat = cvCreateMat(ogmdata.ogmheight_cell,ogmdata.ogmwidth_cell,CV_8UC3);
	cvZero(slopemat);
	int heightnum = ogmdata.ogmheight_cell/(10/ogmdata.ogmresolution);
	int widthnum = ogmdata.ogmwidth_cell/(10/ogmdata.ogmresolution);
	//10m 一条线 zmr
	for(int i = 0; i<heightnum ;i++)
	{
		cvLine(slopemat,cvPoint(0,slopemat->height*i/heightnum),
				cvPoint(slopemat->width-1,slopemat->height*i/heightnum),cvScalar(255,0,0));
	}
	
	for(int i=1;i<widthnum;i++)
	{
		cvLine(slopemat,cvPoint(slopemat->width*i/widthnum,0),
				cvPoint(slopemat->width*i/widthnum,slopemat->height-1),cvScalar(255,0,0));
	}

	float ogmresolution = ogmdata.ogmresolution;

	cvRectangle(slopemat,cvPoint(slopemat->width/2-1/ogmresolution,
			slopemat->height-(20+2)/ogmresolution),
			cvPoint(slopemat->width/2+1/ogmresolution,slopemat->height-(20-2)/ogmresolution),
			cvScalar(0,255,0));
	for(int j=0;j<ogmdata.ogmheight_cell;j++)
	{
		unsigned char* pdata = (unsigned char*)(slopemat->data.ptr + (ogmdata.ogmheight_cell - 1 - j)* slopemat->step);
		for(int i=0 ;i < ogmdata.ogmwidth_cell ; i++)
		{
			unsigned char val = ogmdata.ogm[i + j*ogmdata.ogmwidth_cell];
			if(val > 0)
			{

				unsigned char temp;
				if(val == RIGIDNOPASSABLE)
				{
					temp = 255;
					pdata[3*i] = 0;
					pdata[3*i+1] = temp;
					pdata[3*i+2] = 0;

				}
				else if(val == NEGATIVENOPASSABLE)
				{
					temp = 255;
					pdata[3*i] = 0;
					pdata[3*i+1] = 0;
					pdata[3*i+2] = temp;
				}
				else if(val == PASSABLE)
				{
					temp = 100;
					pdata[3*i] = temp;
					pdata[3*i+1] = temp;
					pdata[3*i+2] = temp;
				}

			}

		}

	}
	cvShowImage(windowname,slopemat);
	cvReleaseMat(&slopemat);
}

/*!
 * \brief 针对于velodyne64线雷达的负障碍检测
 * \param pointcloud 输入点云
 */
void Obstacle_Detection::negativeDetection64(Cloud& pointcloud)
{
	///////可以滤出一些距离不太正常的点也可以是相邻两线之间差的比较多的点。1228傍晚记
	//负障碍检测 参数设置

	int LASER_LAYER = 64;
	memset(polaraxismat_ , 0 ,  sizeof(PolarPointDI)*LASER_LAYER*1801);//0506修改,防止越界
	int col_count = pointcloud.size() / LASER_LAYER;
	float deltaz_thresh = 0.3;//adjustable param //0.3
	float negative_deltaz_thresh=0.3;//adjustable param
	float slope_thresh = tan(25* M_PI / 180);// adjustable param //30
	double Z_Ground = 0.3;//adjustable param
	for(int i = 0;i<pointcloud.size();i++)
	{
		float azimuth = atan2(pointcloud.points[i].y,pointcloud.points[i].x)*180/M_PI;
		while(azimuth > 360.0) azimuth -= 360;
		while(azimuth <=0) azimuth += 360;
		pointcloud.points[i].azimuth = azimuth;
		int mat_i = azimuth * 10;
		PolarPointDI temppolarpoint;
		temppolarpoint.distance = pointcloud.points[i].range;
		temppolarpoint.index = i;
		polaraxismat_[i%LASER_LAYER][mat_i] = temppolarpoint;
	}
	for(int i = 0 ; i < col_count ; i++)
	{
		for(int j = 0 ; j < LASER_LAYER/**2/3*/ ; j++)
		{
			int index_j=grabber_H_.indexmaptable[j].number;//第j线对应的位置
			int ori_index =i * LASER_LAYER + grabber_H_.indexmaptable[j].number;//第i组数据的第j线的编号
			pcl::PointXYZI temppointj=pointcloud.points[ori_index];//将该点提取出来

			if(j > 1 && j < LASER_LAYER - 2)
			{
				int index_front = grabber_H_.indexmaptable[j + 1].number;
				int index_j = grabber_H_.indexmaptable[j].number;
				int index_back = grabber_H_.indexmaptable[j - 1].number;
				int ori_index[4];
				memset(ori_index,0,sizeof(int) * 4);
				pcl::PointXYZI point_front_2,point_front, point_now, point_back;
				double tempazimuthj=temppointj.azimuth ;
				int mat_i=tempazimuthj * 10;//这里单位是0.1度，放大了10倍，水平角度的范围是0到1800
				int mat_i_start=mat_i-1;
				if (mat_i_start<0)
					mat_i_start=0;
				int mat_i_stop=mat_i+1;
				if (mat_i_stop>3599)
					mat_i_stop=3599;
				int disminindex_j=-1;
				int count = 0;
				for(int temp_index = j - 1; temp_index < j + 3;temp_index++)
				{
					double dismin = 1000;
					int disminindex=-1;
					int disminmat_i=mat_i_start;
					for(int temp_i = mat_i_start ; temp_i <= mat_i_stop;temp_i++)
					{
						int index = grabber_H_.indexmaptable[temp_index].number;
						double tempdis= polaraxismat_[index][temp_i].distance;//极坐标
						int index_temp = polaraxismat_[index][temp_i].index;
						if(tempdis<0.5)
							continue;
						if(pointcloud.points[index_temp].passibility < 0.5)
							continue;
						//取一周中最近的那个点，由于64线雷达有时候不是同一个垂直角的，所以取最近的作为正常的点来用。
						if  (dismin > tempdis)
						{
							dismin = tempdis;
							disminindex=polaraxismat_[index][temp_i].index;
							disminmat_i=temp_i;
						}
						if(disminindex < 0)
							continue;
					}
					ori_index[count] = disminindex;
					count = count + 1;
				}
				if(ori_index[0] < 0 || ori_index[1] < 0 || ori_index[2] < 0 || ori_index[3] < 0)
					continue;
				point_front_2=pointcloud.points[ori_index[3]];//这个是正常的那个值。
				point_front=pointcloud.points[ori_index[2]];//这个是正常的那个值。
				point_now=pointcloud.points[ori_index[1]];//这个是正常的那个值。
				point_back=pointcloud.points[ori_index[0]];//这个是正常的那个值。
				if(point_now.range < 0.5|| point_now.range>60)
					continue;

				if(point_now.y< -20||point_now.x< -30||point_now.x > 30) //back exist false detection , so need delete it
					continue;

				if(point_now.z< Z_MIN||point_now.z > Z_MAX) //back exist false detection , so need delete it
					continue;

				if(point_now.y < -1)
					continue;

				if(point_now.z > Z_Ground)
					continue;
				if(point_now.x > 4 || point_now.x < -4) //zmr?
					continue;
				if(point_now.y < 4) //zmr?
					continue;
				int col_delta = 40;//-5 * pointcloud.points[ori_index[1]].range + 95;
				if(point_front.range < 0.5 || point_now.range < 0.5 || point_back.range < 0.5)
					continue;
				int back_point, front_point;//i-40点,i+40点
				if(i < col_delta)
				{
					back_point = i - col_delta + col_count;
					front_point = i + col_delta;
				}
				else if (i > col_count - col_delta - 1)
				{
					back_point = i - col_delta;
					front_point = i + col_delta - col_count;
				}
				else
				{
					back_point = i - col_delta;
					front_point = i + col_delta;
				}
				int ori_index3 = back_point * LASER_LAYER + grabber_H_.indexmaptable[j].number;//第i-40组数据的第j线的数据
				int ori_index4 = front_point * LASER_LAYER + grabber_H_.indexmaptable[j].number;//第i+40组数据的第j线的数据
				int ori_index5 = (back_point + col_delta/2) * LASER_LAYER + grabber_H_.indexmaptable[j].number;//第i-40组数据的第j线的数据  第i-20组数据的第j线的数据 zmr?
				int ori_index6 = (front_point - col_delta/2) * LASER_LAYER + grabber_H_.indexmaptable[j].number;//第i+40组数据的第j线的数据 第i+20组数据的第j线的数据
				//set parameters by distance>10
				if(point_now.range > 13)
				{
					if((point_front.z - point_now.z) > 0.01 && (point_front.z - point_now.z) < 0.12 && (point_front.range - point_now.range) < 0.3 && (point_front.range - point_now.range) > 0.08)
					{
						if((point_now.range - point_back.range) > 0.8 && (point_now.range - point_back.range) < 5 && (point_now.range-point_back.range) / (point_front.range - point_now.range) > 1.5)
						{
							//                			if(((point_now.y-pointcloud.points[ori_index3].y) > 0.3 && (point_now.y-pointcloud.points[ori_index3].y) < 0.7)
							//                			                	|| ((point_now.y-pointcloud.points[ori_index4].y) > 0.3 && (point_now.y-pointcloud.points[ori_index4].y) < 0.7))
							{
								pointcloud.points[ori_index[0]].passibility = 4.0;
								pointcloud.points[ori_index[1]].passibility = 4.0;
								pointcloud.points[ori_index[2]].passibility = 4.0;
							}
						}
					}
				}
				else if(point_now.range > 3)
				{
					if((point_front.z - point_now.z) > 0.01  && (point_front.z - point_now.z) < 0.12 && (point_front.range - point_now.range) < 0.3 && (point_front.range - point_now.range) > 0.06)//0.05 0.2 0
					{
						if((point_now.range-point_back.range)  > 0.6 && (point_now.range-point_back.range)  < 5 && point_back.z - point_now.z > 0.01) //&& (point_now.range-point_back.range)  < 2)//&& (temppointj.range-pointcloud.points[ori_index4].range) < 1.6)
						{
							//                			if(((point_now.range - pointcloud.points[ori_index3].range) > 0.3 && (point_now.range-pointcloud.points[ori_index3].range) < 0.8)
							//                					|| ((point_now.range - pointcloud.points[ori_index4].range) > 0.3 && (point_now.range-pointcloud.points[ori_index4].range) < 0.8))//4*temppointj.range/10))//0.7   *temppointj.range/10
							if((point_now.range-point_back.range) / (point_front_2.range - point_now.range) > 3 && (point_now.range-point_back.range) / (point_front.range - point_now.range) > 3)
							{
								pointcloud.points[ori_index[0]].passibility = 4.0;
								pointcloud.points[ori_index[1]].passibility = 4.0;
								pointcloud.points[ori_index[2]].passibility = 4.0;
							}
							//                			else if((point_now.range-point_back.range) / (point_front.range - point_now.range) > 1.5)
							//                			{
							//                				double theta1 = fabs((point_now.y - pointcloud.points[ori_index5].y) / (point_now.x - pointcloud.points[ori_index5].x));
							//                				double theta2 = fabs((point_now.y - pointcloud.points[ori_index6].y) / (point_now.x - pointcloud.points[ori_index6].x));
							//                				if(fabs(theta1 - theta2) < 1)
							//                				{
							//                					pointcloud.points[ori_index[1]].passibility = 4.0;
							//                					pointcloud.points[ori_index[0]].passibility = 4.0;
							//                					pointcloud.points[ori_index[2]].passibility = 4.0;
							//                				}
							//                			}

						}
					}
				}
			}
		}

	}

}

/*!
 * \brief 对负障碍的预处理（实际使用时未用到，只是调试时用）
 * \param initialCloud 输入点云
 * \param outputCloud 输出点云
 */
void Obstacle_Detection::pretreatForNegative(Cloud& initialCloud, Cloud& outputCloud)
{
	//	cout<<"start = "<<start<<endl;
	 int start = 0;
	int LASER_LAYER = 16;//laser number
	int col_count = initialCloud.size() / LASER_LAYER;//point number of each laser
	for(int i = 0; i < LASER_LAYER; i ++)
	{
		int count[LASER_LAYER];
		memset(count,0,sizeof(int) * LASER_LAYER);
		for(int j = 0; j < col_count; j ++)
		{
			int ori_index = j * LASER_LAYER + i;
			int ori_index_back;
			if(j == 0)
				ori_index_back = (col_count - 1) * LASER_LAYER + i;
			else
				ori_index_back = (j - 1) * LASER_LAYER + i;	start += 1;
			pcl::PointXYZI point = initialCloud.points[ori_index];
			pcl::PointXYZI backpoint = initialCloud.points[ori_index_back];
			if(point.y > 5 && point.y < 10 && point.z < 0.3)
			{
				float delta_range = point.y - backpoint.y;
				if(delta_range > 0.08 && i == 2)
				{
					cout<<"i ============="<<i<<endl;
					cout<<"point.y = "<<point.y<<endl;
					cout<<"delta_range =  "<<delta_range<<endl;
					initialCloud.points[ori_index].passibility = 4.0;
				}

			}
		}
	}
	//	start += 1;
}

/*!
 * \brief 针对于速腾聚创侧面安装的16线雷达（左侧）的负障碍检测
 * \param pointcloud 输入点云
 */
void Obstacle_Detection::negativeDetectionForLeftRslidar(Cloud& pointcloud)
{
	//j+1是上面的那个点，j-1是下面那个点
	int LASER_LAYER = 16;//laser number
	int col_count = pointcloud.size() / LASER_LAYER;//point number of each laser
	//set different parameters by different ranges
	double back_point_delta_range,front_point_delta_range;
	for(int i = 0; i < pointcloud.size();i++)
	{
		pointcloud.points[i].intensity = 0;
	}
	int delta_point_num = 5;//10
	int dist_threshold = 10;//distance threshold, different distances for different thresholds
	double test_point_delta_range;
	int point_front_count[pointcloud.size()];
	int point_back_count[pointcloud.size()];
	double front_delta_azimuth[pointcloud.size()];
	double back_delta_azimuth[pointcloud.size()];
	memset(point_front_count,0,sizeof(int) * pointcloud.size());
	memset(point_back_count,0,sizeof(int) * pointcloud.size());
	memset(front_delta_azimuth,0,sizeof(double) * pointcloud.size());
	memset(back_delta_azimuth,0,sizeof(double) * pointcloud.size());
	for(int i = 0; i < LASER_LAYER; i ++)
	{
		for(int j = delta_point_num; j < col_count - 1 - delta_point_num; j ++)
		{
			pcl::PointXYZI point,backpoint,point_back,point_back_more,point_front,point_front_more,point_test_front,point_test_back,test,test_2;
			int ori_index = j * LASER_LAYER + i;
			int ori_index_back = (j - 1) * LASER_LAYER + i;
			//滤除一些杂点
			point = pointcloud.points[ori_index];
			backpoint = pointcloud.points[(j - 1) * LASER_LAYER + i];//当前点下面那个点
			if(pointcloud.points[ori_index].range < 0)
				continue;
			if(abs(point.x) > 4)
				continue;
			//			if(backpoint.range < 0)
			//			{
			//				if( pointcloud.points[(j - 2) * LASER_LAYER + i].range > 0 && pointcloud.points[(j - 3) * LASER_LAYER + i].range > 0)
			//				{
			//					backpoint.x = 2 * pointcloud.points[(j - 2) * LASER_LAYER + i].x - pointcloud.points[(j - 3) * LASER_LAYER + i].x;
			//									backpoint.y = 2 * pointcloud.points[(j - 2) * LASER_LAYER + i].y - pointcloud.points[(j - 3) * LASER_LAYER + i].y;
			//									backpoint.z = 2 * pointcloud.points[(j - 2) * LASER_LAYER + i].z - pointcloud.points[(j - 3) * LASER_LAYER + i].z;
			//									backpoint.range = 2 * pointcloud.points[(j - 2) * LASER_LAYER + i].range - pointcloud.points[(j - 3) * LASER_LAYER + i].range;
			//
			//				}
			//				else
			//					continue;
			//			}
			if(backpoint.range < 0)
				continue;
			if(point.range - backpoint.range < 0)//排除了车辆后方?zmr?
				continue;
			if(point.z > 0.3 || backpoint.z > 0.3)
				continue;
			//当前点与下面那个点的距离差
			double delta_point_range = sqrt(((point.x - backpoint.x) * (point.x - backpoint.x) + (point.y - backpoint.y) * (point.y - backpoint.y)));
			//如果当前点的y在15和10m之间，考虑到根据不同距离设置不同阈值
			if(point.y > 5 && point.y < 17)
			{
				back_point_delta_range = 0.1;
				front_point_delta_range = 0.1;
				test_point_delta_range = 0.5;
				//当两点距离差大于1米并且这两个点是相邻的 0.6米吧zmr?
				if(abs(point.azimuth - backpoint.azimuth) - 0.18 < 0.2 && delta_point_range > 0.6)
				{
					//针对下面点及更下面的点
					for(int delta = 2; delta < delta_point_num + 2; delta ++)
					{
						if(pointcloud.points[(j - 1) * LASER_LAYER + i].range < 0)
							continue;
						//后面的相邻的两个点
						point_back_more = pointcloud.points[((j - delta) * LASER_LAYER) + i];
						point_back = pointcloud.points[(j - delta + 1) * LASER_LAYER + i];
						if(point_back_more.range < 1 || point_back.range < 1)
						{
							point_back_count[ori_index] = 100;
							break;
						}
						double delta_range = 0;
						double delta_test_range = 0;
						delta_test_range = backpoint.range - point_back.range;
						if(delta_test_range < -0.1)
						{
							point_back_count[ori_index] = 100;
							break;
						}
						//后面相邻两个点的距离差值
						delta_range = sqrt(((point_back.x - point_back_more.x) * (point_back.x - point_back_more.x) + (point_back.y - point_back_more.y) * (point_back.y - point_back_more.y)));
						if(delta_range < back_point_delta_range)
							point_back_count[ori_index] ++;
					}
					//针对上面的点及更上面的点
					for(int delta = 0; delta < delta_point_num; delta ++)
					{
						point_front = pointcloud.points[((j + delta) * LASER_LAYER) + i];
						point_front_more = pointcloud.points[((j + delta + 1) * LASER_LAYER) + i];
						if(point_front.range < 1 || point_front_more.range < 1)
						{
							point_front_count[ori_index] = -100;
							break;
						}
						double delta_range = 0;
						//向上的距离差值
						delta_range = sqrt(((point_front_more.x - point_front.x) * (point_front_more.x - point_front.x) + (point_front_more.y - point_front.y) * (point_front_more.y - point_front.y)));
						if(delta_range < front_point_delta_range)
							point_front_count[ori_index] ++;
					}
				}
				if(point_front_count[ori_index] > 4 && point_back_count[ori_index] < 1)
				{
					for(int k = 1;k < 20; k ++)
					{
						if(j + k > col_count - 1)
							point_test_front = pointcloud.points[(j + k - col_count) * LASER_LAYER + i];
						else
							point_test_front = pointcloud.points[(j + k) * LASER_LAYER + i];
						double test_point_range = sqrt((point.x - point_test_front.x) * (point.x - point_test_front.x) + (point.y - point_test_front.y) * (point.y - point_test_front.y));
						if(test_point_range > test_point_delta_range)
						{
							front_delta_azimuth[ori_index] = abs(point_test_front.azimuth - point.azimuth);
							break;
						}
					}
					for(int k = 2;k < 21; k ++)
					{
						if(j - k < 0)
						{
							point_test_back = pointcloud.points[(j - k + col_count) * LASER_LAYER + i];
						}
						else
						{
							point_test_back = pointcloud.points[(j - k) * LASER_LAYER + i];
						}
						double test_point_range = sqrt((backpoint.x - point_test_back.x) * (backpoint.x - point_test_back.x) + (backpoint.y - point_test_back.y) * (backpoint.y - point_test_back.y));

						if(test_point_range > test_point_delta_range)
						{
							back_delta_azimuth[ori_index] = abs(point_test_back.azimuth - backpoint.azimuth);
							break;
						}
					}
				}
			}
			else if(point.y >= 17 && point.y < 19)
			{
				back_point_delta_range = 0.15;
				front_point_delta_range = 0.15;
				test_point_delta_range = 0.5;
				//当两点距离差大于1米并且这两个点是相邻的
				if(abs(point.azimuth - backpoint.azimuth) - 0.18 < 0.2 && delta_point_range > 1)
				{
					//针对下面点及更下面的点
					for(int delta = 2; delta < delta_point_num + 2; delta ++)
					{
						if(pointcloud.points[(j - 1) * LASER_LAYER + i].range < 0)
							continue;
						//后面的相邻的两个点
						point_back_more = pointcloud.points[((j - delta) * LASER_LAYER) + i];
						point_back = pointcloud.points[(j - delta + 1) * LASER_LAYER + i];
						if(point_back_more.range < 1 || point_back.range < 1)
						{
							point_back_count[ori_index] = 100;
							break;
						}
						double delta_range = 0;
						double delta_test_range = 0;
						delta_test_range = backpoint.range - point_back.range;
						if(delta_test_range < -0.1)
						{
							point_back_count[ori_index] = 100;
							break;
						}
						//后面相邻两个点的距离差值
						delta_range = sqrt(((point_back.x - point_back_more.x) * (point_back.x - point_back_more.x) + (point_back.y - point_back_more.y) * (point_back.y - point_back_more.y)));
						if(delta_range < back_point_delta_range)
							point_back_count[ori_index] ++;
					}
					//针对上面的点及更上面的点
					for(int delta = 0; delta < delta_point_num; delta ++)
					{
						point_front = pointcloud.points[((j + delta) * LASER_LAYER) + i];
						point_front_more = pointcloud.points[((j + delta + 1) * LASER_LAYER) + i];
						if(point_front.range < 1 || point_front_more.range < 1)
						{
							point_front_count[ori_index] = -100;
							break;
						}
						double delta_range = 0;
						//向上的距离差值
						delta_range = sqrt(((point_front_more.x - point_front.x) * (point_front_more.x - point_front.x) + (point_front_more.y - point_front.y) * (point_front_more.y - point_front.y)));
						if(delta_range < front_point_delta_range)
							point_front_count[ori_index] ++;
					}
				}
				if(point_front_count[ori_index] >= 3  && point_back_count[ori_index] < 1)
				{
					for(int k = 1;k < 20; k ++)
					{
						if(j + k > col_count - 1)
							point_test_front = pointcloud.points[(j + k - col_count) * LASER_LAYER + i];
						else
							point_test_front = pointcloud.points[(j + k) * LASER_LAYER + i];
						double test_point_range = sqrt((point.x - point_test_front.x) * (point.x - point_test_front.x) + (point.y - point_test_front.y) * (point.y - point_test_front.y));
						if(test_point_range > test_point_delta_range)
						{
							front_delta_azimuth[ori_index] = abs(point_test_front.azimuth - point.azimuth);
							break;
						}
					}
					for(int k = 2;k < 21; k ++)
					{
						if(j - k < 0)
						{
							point_test_back = pointcloud.points[(j - k + col_count) * LASER_LAYER + i];
						}
						else
						{
							point_test_back = pointcloud.points[(j - k) * LASER_LAYER + i];
						}
						double test_point_range = sqrt((backpoint.x - point_test_back.x) * (backpoint.x - point_test_back.x) + (backpoint.y - point_test_back.y) * (backpoint.y - point_test_back.y));

						if(test_point_range > test_point_delta_range)
						{
							back_delta_azimuth[ori_index] = abs(point_test_back.azimuth - backpoint.azimuth);
							break;
						}
					}
				}
			}
			if((front_delta_azimuth[ori_index] - back_delta_azimuth[ori_index]) / 0.18 > 3)// && (front_delta_azimuth[ori_index] - back_delta_azimuth[ori_index]) / 0.18 < 8)
			{
//				cout<<"(front_delta_azimuth - back_delta_azimuth) / 0.18 = "<<(front_delta_azimuth[ori_index] - back_delta_azimuth[ori_index]) / 0.18<<endl;
//				cout<<"left_point[ori_index].x = "<<pointcloud.points[ori_index].x<<endl;
//				cout<<"left_point[ori_index].y = "<<pointcloud.points[ori_index].y<<endl;
//				cout<<"left_point[ori_index].z = "<<pointcloud.points[ori_index].z<<endl;
//				cout<<"point[ori_index] = "<<j<<endl;
//				cout<<"left_point[ori_index].range"<<point.range<<endl;
//				cout<<"point_front_count[ori_index] = "<<point_front_count[ori_index]<<endl;
//				cout<<"point_back_count[ori_index] = "<<point_back_count[ori_index]<<endl;
//				cout<<"point_back.x = "<<backpoint.x<<endl;
//				cout<<"point_back.y = "<<backpoint.y<<endl;
//				cout<<"point_back.z = "<<backpoint.z<<endl;
//				cout<<"point_back.range"<<backpoint.range<<endl;
				pointcloud.points[ori_index_back].intensity = delta_point_range;
				pointcloud.points[ori_index_back].passibility = 4.0;
				//				cout<<"delta_point_range = "<<delta_point_range<<endl;
				//					pointcloud.points[(j - 1) * LASER_LAYER + i].passibility = 4.0;
				for(int delta = 0; delta < 5; delta ++)
				{
					int ori_index_front;
					if((j + delta) > (col_count - 1))
						ori_index_front = ((j + delta - col_count) * LASER_LAYER) + i;
					else
						ori_index_front = (j + delta) * LASER_LAYER + i;
//					pointcloud.points[ori_index_front].passibility = 4.0;
//					pointcloud.points[ori_index_front].intensity = delta_point_range;
				}
			}

		}
	}
}

/*!
 * \brief 针对于速腾聚创侧面安装的16线雷达（右侧）的负障碍检测
 * \param pointcloud 输入点云
 */
void Obstacle_Detection::negativeDetectionForRightRslidar(Cloud& pointcloud)
{
	//j+1是上面的那个点，j-1是下面那个点
	int LASER_LAYER = 16;//laser number
	int col_count = pointcloud.size() / LASER_LAYER;//point number of each laser
	//set different parameters by different ranges
	double back_point_delta_range,front_point_delta_range;
	for(int i = 0; i < pointcloud.size();i++)
	{
		pointcloud.points[i].intensity = 0;
	}
	int delta_point_num = 5;//10
	int dist_threshold = 10;//distance threshold, different distances for different thresholds
	double test_point_delta_range;
	int point_front_count[pointcloud.size()];
	int point_back_count[pointcloud.size()];
	double front_delta_azimuth[pointcloud.size()];
	double back_delta_azimuth[pointcloud.size()];
	memset(point_front_count,0,sizeof(int) * pointcloud.size());
	memset(point_back_count,0,sizeof(int) * pointcloud.size());
	memset(front_delta_azimuth,0,sizeof(double) * pointcloud.size());
	memset(back_delta_azimuth,0,sizeof(double) * pointcloud.size());
	for(int i = 0; i < LASER_LAYER; i ++)
	{
		for(int j = delta_point_num; j < col_count - 1 - delta_point_num; j ++)
		{
			pcl::PointXYZI point,backpoint,point_back,point_back_more,point_front,point_front_more,point_test_front,point_test_back,test,test_2;
			int ori_index = j * LASER_LAYER + i;
			//滤除一些杂点
			point = pointcloud.points[ori_index];
			backpoint = pointcloud.points[(j + 1) * LASER_LAYER + i];//当前点下面那个点
			if(pointcloud.points[ori_index].range < 0)
				continue;
			if(abs(point.x) > 4)
				continue;
			//			if(backpoint.range < 0)
			//				backpoint = pointcloud.points[(j - 2) * LASER_LAYER + i];
			if(backpoint.range < 0)
				continue;
			if(point.range - backpoint.range < 0)
				continue;
			if(point.z > 0.3 || backpoint.z > 0.3)
				continue;
			//当前点与下面那个点的距离差
			double delta_point_range = sqrt(((point.x - backpoint.x) * (point.x - backpoint.x) + (point.y - backpoint.y) * (point.y - backpoint.y)));
			//如果当前点的y在15和10m之间，考虑到根据不同距离设置不同阈值
			if(point.y > 10 && point.y < 17)
			{
				back_point_delta_range = 0.1;
				front_point_delta_range = 0.1;
				test_point_delta_range = 0.5;
				//当两点距离差大于1米并且这两个点是相邻的
				if(abs(point.azimuth - backpoint.azimuth) - 0.18 < 0.2 && delta_point_range > 0.6)
				{
					//针对下面点及更下面的点
					for(int delta = 2; delta < delta_point_num + 2; delta ++)
					{
						if(pointcloud.points[(j + 1) * LASER_LAYER + i].range < 0)
							continue;
						//后面的相邻的两个点
						point_back_more = pointcloud.points[((j + delta) * LASER_LAYER) + i];
						point_back = pointcloud.points[(j + delta - 1) * LASER_LAYER + i];
						if(point_back_more.range < 1 || point_back.range < 1)
						{
							point_back_count[ori_index] = 100;
							break;
						}
						double delta_range = 0;
						double delta_test_range = 0;
						delta_test_range = backpoint.range - point_back.range;
						if(delta_test_range < -0.1)
						{
							point_back_count[ori_index] = 100;
							break;
						}
						//后面相邻两个点的距离差值
						delta_range = sqrt(((point_back.x - point_back_more.x) * (point_back.x - point_back_more.x) + (point_back.y - point_back_more.y) * (point_back.y - point_back_more.y)));
						if(delta_range < back_point_delta_range)
							point_back_count[ori_index] ++;
					}
					//针对上面的点及更上面的点
					for(int delta = 0; delta < delta_point_num; delta ++)
					{
						point_front = pointcloud.points[((j - delta) * LASER_LAYER) + i];
						point_front_more = pointcloud.points[((j - delta - 1) * LASER_LAYER) + i];
						if(point_front.range < 1 || point_front_more.range < 1)
						{
							point_front_count[ori_index] = -100;
							break;
						}
						double delta_range = 0;
						//向上的距离差值
						delta_range = sqrt(((point_front_more.x - point_front.x) * (point_front_more.x - point_front.x) + (point_front_more.y - point_front.y) * (point_front_more.y - point_front.y)));
						if(delta_range < front_point_delta_range)
							point_front_count[ori_index] ++;
					}
				}
				if(point_front_count[ori_index] > 4 && point_back_count[ori_index] < 1)
				{
					for(int k = 1;k < 20; k ++)
					{
						if(j - k < 0)
							point_test_front = pointcloud.points[(j - k + col_count) * LASER_LAYER + i];
						else
							point_test_front = pointcloud.points[(j - k) * LASER_LAYER + i];
						double test_point_range = sqrt((point.x - point_test_front.x) * (point.x - point_test_front.x) + (point.y - point_test_front.y) * (point.y - point_test_front.y));
						if(test_point_range > test_point_delta_range)
						{
							front_delta_azimuth[ori_index] = abs(point_test_front.azimuth - point.azimuth);
							break;
						}
					}
					for(int k = 2;k < 21; k ++)
					{
						if(j + k > col_count - 1)
						{
							point_test_back = pointcloud.points[(j + k - col_count) * LASER_LAYER + i];
						}
						else
						{
							point_test_back = pointcloud.points[(j + k) * LASER_LAYER + i];
						}
						double test_point_range = sqrt((backpoint.x - point_test_back.x) * (backpoint.x - point_test_back.x) + (backpoint.y - point_test_back.y) * (backpoint.y - point_test_back.y));

						if(test_point_range > test_point_delta_range)
						{
							back_delta_azimuth[ori_index] = abs(point_test_back.azimuth - backpoint.azimuth);
							break;
						}
					}
				}
			}
			else if(point.y >= 17 && point.y < 19)
			{
				//				if(i == 7)
				//				{
				//					pointcloud.points[ori_index].passibility = 4.0;
				//					cout<<"delta_range ======== "<<delta_point_range<<endl;
				//				}
				back_point_delta_range = 0.15;
				front_point_delta_range = 0.15;
				test_point_delta_range = 0.5;
				//当两点距离差大于1米并且这两个点是相邻的
				if(abs(point.azimuth - backpoint.azimuth) - 0.18 < 0.2 && delta_point_range > 1)
				{
					//针对下面点及更下面的点
					for(int delta = 2; delta < delta_point_num + 2; delta ++)
					{
						if(pointcloud.points[(j + 1) * LASER_LAYER + i].range < 0)
							continue;
						//后面的相邻的两个点
						point_back_more = pointcloud.points[((j + delta) * LASER_LAYER) + i];
						point_back = pointcloud.points[(j + delta - 1) * LASER_LAYER + i];
						if(point_back_more.range < 1 || point_back.range < 1)
						{
							point_back_count[ori_index] = 100;
							break;
						}
						double delta_range = 0;
						double delta_test_range = 0;
						delta_test_range = backpoint.range - point_back.range;
						if(delta_test_range < -0.1)
						{
							point_back_count[ori_index] = 100;
							break;
						}
						//后面相邻两个点的距离差值
						delta_range = sqrt(((point_back.x - point_back_more.x) * (point_back.x - point_back_more.x) + (point_back.y - point_back_more.y) * (point_back.y - point_back_more.y)));
						if(delta_range < back_point_delta_range)
							point_back_count[ori_index] ++;
					}
					//针对上面的点及更上面的点
					for(int delta = 0; delta < delta_point_num; delta ++)
					{
						point_front = pointcloud.points[((j - delta) * LASER_LAYER) + i];
						point_front_more = pointcloud.points[((j - delta - 1) * LASER_LAYER) + i];
						if(point_front.range < 1 || point_front_more.range < 1)
						{
							point_front_count[ori_index] = -100;
							break;
						}
						double delta_range = 0;
						//向上的距离差值
						delta_range = sqrt(((point_front_more.x - point_front.x) * (point_front_more.x - point_front.x) + (point_front_more.y - point_front.y) * (point_front_more.y - point_front.y)));
						if(delta_range < front_point_delta_range)
							point_front_count[ori_index] ++;
					}
				}
				if(point_front_count[ori_index] >= 3  && point_back_count[ori_index] < 1)
				{
					for(int k = 1;k < 20; k ++)
					{
						if(j - k < 0)
							point_test_front = pointcloud.points[(j - k + col_count) * LASER_LAYER + i];
						else
							point_test_front = pointcloud.points[(j - k) * LASER_LAYER + i];
						double test_point_range = sqrt((point.x - point_test_front.x) * (point.x - point_test_front.x) + (point.y - point_test_front.y) * (point.y - point_test_front.y));
						if(test_point_range > test_point_delta_range)
						{
							front_delta_azimuth[ori_index] = abs(point_test_front.azimuth - point.azimuth);
							break;
						}
					}
					for(int k = 2;k < 21; k ++)
					{
						if(j + k < 0)
						{
							point_test_back = pointcloud.points[(j + k - col_count) * LASER_LAYER + i];
						}
						else
						{
							point_test_back = pointcloud.points[(j + k) * LASER_LAYER + i];
						}
						double test_point_range = sqrt((backpoint.x - point_test_back.x) * (backpoint.x - point_test_back.x) + (backpoint.y - point_test_back.y) * (backpoint.y - point_test_back.y));

						if(test_point_range > test_point_delta_range)
						{
							back_delta_azimuth[ori_index] = abs(point_test_back.azimuth - backpoint.azimuth);
							break;
						}
					}
				}
			}
			if((front_delta_azimuth[ori_index] - back_delta_azimuth[ori_index]) / 0.18 > 3 && (front_delta_azimuth[ori_index] - back_delta_azimuth[ori_index]) / 0.18 < 15)
			{
//				cout<<"(front_delta_azimuth - back_delta_azimuth) / 0.18 = "<<(front_delta_azimuth[ori_index] - back_delta_azimuth[ori_index]) / 0.18<<endl;
//				cout<<"right_point[ori_index].x = "<<pointcloud.points[ori_index].x<<endl;
//				cout<<"right_point[ori_index].y = "<<pointcloud.points[ori_index].y<<endl;
//				cout<<"right_point[ori_index].z = "<<pointcloud.points[ori_index].z<<endl;
//				cout<<"point[ori_index] = "<<j<<endl;
//				cout<<"right_point[ori_index].range"<<point.range<<endl;
//				cout<<"point_front_count[ori_index] = "<<point_front_count[ori_index]<<endl;
//				cout<<"point_back_count[ori_index] = "<<point_back_count[ori_index]<<endl;
//				cout<<"point_back.x = "<<backpoint.x<<endl;
//				cout<<"point_back.y = "<<backpoint.y<<endl;
//				cout<<"point_back.z = "<<backpoint.z<<endl;
//				cout<<"point_back.range"<<backpoint.range<<endl;
				pointcloud.points[ori_index].intensity = delta_point_range;
				pointcloud.points[ori_index].passibility = 4.0;
//				cout<<"delta_point_range = "<<delta_point_range<<endl;
//				cout<<"pointcloud.points[ori_index].intensity = "<<pointcloud.points[ori_index].intensity<<endl;
				pointcloud.points[(j - 1) * LASER_LAYER + i].passibility = 4.0;
				for(int delta = 0; delta < 5; delta ++)
				{
					int ori_index_front;
					if((j - delta) < 0)
						ori_index_front = ((j - delta + col_count) * LASER_LAYER) + i;
					else
						ori_index_front = (j - delta) * LASER_LAYER + i;
					pointcloud.points[ori_index_front].passibility = 4.0;
					pointcloud.points[ori_index_front].intensity = delta_point_range;
				}
			}

		}
	}
}



