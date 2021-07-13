#include "roadboundary.h"

//#define MIN(a,b) ((a)>(b)?(b):(a))
/*
void RoadPaths::getRoadPath(const std::vector<CloudPointSections>& cloudsections,double *anglerange)
{
    int LASER_LAYER = cloudsections.size();
    paths.clear();
    std::vector<RoadPath> temproadpaths;
    const CloudPointSections* tempsections = &cloudsections[0];
    if(cloudsections[0].sections.size()>0)
    {
        for(int i=0;i<cloudsections[0].sections.size();i++)
        {
			if(fabs(cloudsections[0].sections[i].height)>0.3)
				continue;
            RoadPath temproad(LASER_LAYER);
			RoadNode tempnode;
			tempnode.index = i;
			tempnode.valid=true;

			tempnode.anglebegin=cloudsections[0].sections[i].minpoint.angle;
			tempnode.angleend=cloudsections[0].sections[i].maxpoint.angle;
			tempnode.anglerange=cloudsections[0].sections[i].anglerange;
			tempnode.distance=cloudsections[0].sections[i].distance;
			tempnode.height=cloudsections[0].sections[i].height;
            tempnode.angle = tempnode.anglebegin>tempnode.angleend
               ?((tempnode.anglebegin+tempnode.angleend)>360?(tempnode.anglebegin+tempnode.angleend-360)/2:(tempnode.anglebegin+tempnode.angleend+360)/2)
               :(tempnode.anglebegin+tempnode.angleend)/2;


			temproad.nodes.push_back(tempnode);
            temproadpaths.push_back(temproad);
//            std::cout<<"capacity:"<<roadpath[i].capacity()<<std::endl;
        }

        for(int j=1 ;cloudsections[j].sections.size()>0;j++)
        {

            for(std::vector<RoadPath>::iterator it_path=temproadpaths.begin() ; it_path!=temproadpaths.end();it_path++)
            {
                RoadPath path(*it_path);
//                if(path.size()<j )
                if(path.nodes.size()<j)
                    continue;
                std::vector<RoadNode> next_nodes ;
                int this_node_index = path.nodes[j-1].index;     //the last node of this path now;
//                std::cout<<"this_node_index="<<this_node_index<<"\tj="<<j-1<<"\t"<<&tempsections[size_t(j-1)]<<"\t"<<&tempsections[0]<<"\t"<<sizeof(size_t)<<std::endl;
                std::cout<<"this_node_index="<<this_node_index<<"\tj="<<j-1<<"\t"<<&cloudsections[j-1]<<std::endl;
                std::cout<<"this_node_index="<<this_node_index<<"\tj="<<j<<"\t"<<&cloudsections[j]<<std::endl;
                if(&cloudsections[j-1] +1 != &cloudsections[j] )
                    continue;
                const CloudPointSection& this_section = cloudsections[j-1].sections[this_node_index];
                int num = 0;
                std::vector<RoadPath> nextpaths;
                for(int i=0;i<cloudsections[j].sections.size();i++)
                {
                    std::cout<<"i="<<i<<"\tsize="<<cloudsections[j].sections.size()<<std::endl;
                    const CloudPointSection& tempsection = cloudsections[j].sections[i];
                    float s1anglebegin=this_section.minpoint.angle;
                    float s1angleend=this_section.maxpoint.angle;
                    float s2anglebegin=tempsection.minpoint.angle;
                    float s2angleend=tempsection.maxpoint.angle;

					std::vector<float> beginangle_vec,endangle_vec;   //there may be two overlap section.
                    float minangle = this_section.anglerange>tempsection.anglerange?tempsection.anglerange:this_section.anglerange;
                    std::vector<float> anglediff_vec = getAngleOverlap(s1anglebegin,s1angleend,s2anglebegin,s2angleend,beginangle_vec,endangle_vec);
					for(int vec_i=0;vec_i<anglediff_vec.size();vec_i++)
					{

                        std::cout<<"anglediffsize="<<anglediff_vec.size()<<std::endl;
						float anglediff = anglediff_vec[vec_i];
						float beginangle = beginangle_vec[vec_i];
						float endangle = endangle_vec[vec_i];
                        float angletotaldiff = anglediff;
                        if(anglediff_vec.size()>1)
                            angletotaldiff = anglediff_vec[0]+anglediff_vec[1];
							std::cout<<j<<" "<<this_node_index<<" "<<i<<" begin:"<<beginangle<<" end:"<<endangle<<" overlap:"<<anglediff<<" minangle"<<minangle<<" capacity:"<<it_path->nodes.capacity()<<std::endl;
						if(fabs(anglediff)>anglerange[j-1]&&fabs(angletotaldiff/minangle)>0.5)
						{
							RoadNode tempnode;
							tempnode.index=i;
							//float anglerange = getAngleOverlap(path.nodes[j-1].anglebegin,path.nodes[j-1].angleend,beginangle,endangle,tempnode.anglebegin,tempnode.angleend);

					        std::vector<float> beginangle_vec1,endangle_vec1;   //there may be two overlap section.
                            std::vector<float> anglediff_vec1 = getAngleOverlap(path.nodes[j-1].anglebegin,path.nodes[j-1].angleend,beginangle,endangle,beginangle_vec1,endangle_vec1);

                            if(anglediff_vec1.size()==0)
                            {
                                tempnode.valid = true;
                                tempnode.anglebegin = beginangle;
                                tempnode.angleend = endangle;
                                tempnode.anglerange = 0;
                                tempnode.angle = tempnode.anglebegin>tempnode.angleend
                                   ?((tempnode.anglebegin+tempnode.angleend)>360?(tempnode.anglebegin+tempnode.angleend-360)/2:(tempnode.anglebegin+tempnode.angleend+360)/2)
                                   :(tempnode.anglebegin+tempnode.angleend)/2;

								RoadPath temppath(path);
								temppath.nodes.push_back(tempnode);
                                nextpaths.push_back(temppath);
                            }
					        for(int vec_j=0;vec_j<anglediff_vec1.size();vec_j++)
							{
                                double tempanglebegin = beginangle_vec1[vec_j];
                                double tempangleend = endangle_vec1[vec_j];
                                float heightdiff,disdiff,this_height,this_distance,last_height,last_distance;
                                if(tempangleend>=tempanglebegin)
                                {
                                    int laser_num_this = grabber_H_.indexmaptable[j].number;
                                    int laser_num_last = grabber_H_.indexmaptable[j-1].number;
                                    int mat_begin = tempanglebegin*10;
                                    int mat_end = tempangleend*10;
                                    int this_countsum = passable_count_integral[laser_num_this][mat_end] - passable_count_integral[laser_num_this][mat_begin];
                                    int last_countsum = passable_count_integral[laser_num_last][mat_end] - passable_count_integral[laser_num_last][mat_begin];
                                    double this_heightsum = passable_height_integral[laser_num_this][mat_end] - passable_height_integral[laser_num_this][mat_begin];
                                    double last_heightsum = passable_height_integral[laser_num_last][mat_end] - passable_height_integral[laser_num_last][mat_begin];
                                    double this_distancesum = passable_distance_integral[laser_num_this][mat_end] - passable_distance_integral[laser_num_this][mat_begin];
                                    double last_distancesum = passable_distance_integral[laser_num_last][mat_end] - passable_distance_integral[laser_num_last][mat_begin];

                                    this_height = this_heightsum/ this_countsum;
                                    last_height = last_heightsum/ last_countsum;

                                    this_distance = this_heightsum/ this_countsum;
                                    last_distance = last_heightsum/ last_countsum;
                                    heightdiff = (this_height - last_height);
                                    disdiff = (this_distance - last_distance);
                                    std::cout<<"this_countsum:"<<this_countsum<<"\tlast_countsum"<<last_countsum<<std::endl;

                                }
                                else
                                {
                                    int laser_num_this = grabber_H_.indexmaptable[j].number;
                                    int laser_num_last = grabber_H_.indexmaptable[j-1].number;
                                    int mat_begin = tempanglebegin*10;
                                    int mat_end = tempangleend*10;
                                    int this_countsum = passable_count_integral[laser_num_this][mat_end] + passable_count_integral[laser_num_this][3600] - passable_count_integral[laser_num_this][mat_begin];
                                    int last_countsum = passable_count_integral[laser_num_last][mat_end] + passable_count_integral[laser_num_last][3600] - passable_count_integral[laser_num_last][mat_begin];
                                    double this_heightsum = passable_height_integral[laser_num_this][mat_end] + passable_height_integral[laser_num_this][3600] - passable_height_integral[laser_num_this][mat_begin];
                                    double last_heightsum = passable_height_integral[laser_num_last][mat_end] + passable_height_integral[laser_num_last][3600] - passable_height_integral[laser_num_last][mat_begin];
                                    double this_distancesum = passable_distance_integral[laser_num_this][mat_end] + passable_distance_integral[laser_num_this][3600] - passable_distance_integral[laser_num_this][mat_begin];
                                    double last_distancesum = passable_distance_integral[laser_num_last][mat_end] + passable_distance_integral[laser_num_last][3600] - passable_distance_integral[laser_num_last][mat_begin];

                                    this_height = this_heightsum/ this_countsum;
                                    last_height = last_heightsum/ last_countsum;

                                    this_distance = this_heightsum/ this_countsum;
                                    last_distance = last_heightsum/ last_countsum;
                                    heightdiff = (this_height - last_height);
                                    disdiff = (this_distance - last_distance);
                                    std::cout<<"this_countsum:"<<this_countsum<<"\tlast_countsum"<<last_countsum<<std::endl;

                                }

                                std::cout<<"heightdiff:"<<heightdiff<<"\tbegin:"<<tempanglebegin<<"\tend:"<<tempangleend<<std::endl;

                                if(fabs(heightdiff)>0.3&&fabs(heightdiff/disdiff)>0.2)
                                    continue;

                                tempnode.anglebegin = beginangle_vec1[vec_j];
                                tempnode.angleend = endangle_vec1[vec_j];
                                tempnode.anglerange = anglediff_vec1[vec_j];
                                tempnode.distance = this_distance;
							    tempnode.height = this_height;

                                float anglerange = anglediff_vec1[vec_j];
								RoadPath temppath(path);
								if(fabs(anglerange)>0)
								{
									temppath.nodes[j-1].valid=false;
									tempnode.valid = true;
									tempnode.angle = tempnode.anglebegin>tempnode.angleend
									   ?((tempnode.anglebegin+tempnode.angleend)>360?(tempnode.anglebegin+tempnode.angleend-360)/2:(tempnode.anglebegin+tempnode.angleend+360)/2)
					    			   :(tempnode.anglebegin+tempnode.angleend)/2;
								}
								else
								{
									tempnode.valid = true;
									tempnode.anglebegin = beginangle;
									tempnode.angleend = endangle;
									tempnode.angle = tempnode.anglebegin>tempnode.angleend
									   ?((tempnode.anglebegin+tempnode.angleend)>360?(tempnode.anglebegin+tempnode.angleend-360)/2:(tempnode.anglebegin+tempnode.angleend+360)/2)
									   :(tempnode.anglebegin+tempnode.angleend)/2;

								}
								temppath.nodes.push_back(tempnode);
	//                            std::cout<<"iter num:"<<it_path-temproadpaths.begin()<<" size"<<temproadpaths.size()<<std::endl;
                                nextpaths.push_back(temppath);
								//it_path=temproadpaths.insert(it_path+1,temppath);

	//                            std::cout<<"iter num:"<<it_path-temproadpaths.begin()<<" size"<<temproadpaths.size()<<std::endl;
							}
							//next_nodes.push_back(i);
						}
					}
                }

                float maxheight=-2;
                float heightdiff_thre=0.3;
                float anglediff_thre = 20;
                float totalbeginangle = 0;
                float lastheight;
                float lastendangle;
                float lastangle;
                int lastnum = 0;
                if(nextpaths.size()>0)
                {
                    int pathnum = nextpaths.size();
                    unsigned char isvalid[pathnum];
                    memset(isvalid,1,pathnum);
//                    std::cout<<j<<" "<<pathnum<<std::endl;
                    for(std::vector<RoadPath>::iterator it=nextpaths.begin();it!=nextpaths.end();it++)
                    {
                        int num = it - nextpaths.begin();
//                        std::cout<<"num:"<<j<<std::endl;
 //                       std::cout<<"j:"<<j<<"\tsize:"<<(int)pathnum<<"\tnum:"<<(int)num<<"\tlastnum:"<<(int)lastnum<<"\tlastangle"<<lastangle<<std::endl;
                        if(num==0)
                        {
                            lastheight = it->nodes.at(j).height;
                            lastendangle = it->nodes.at(j).angleend;
                            totalbeginangle = it->nodes.at(j).anglebegin;
                            lastangle = it->nodes.at(j).angle;
                            lastnum = 0;
                        }
                        else
                        {
                            if((lastheight - it->nodes.at(j).height)>heightdiff_thre )
                            {
                                if(it->nodes.at(j).angle - lastendangle< anglediff_thre )
                                {
                                    isvalid[num] = 0;
                                }
                                else
                                {
                                    lastheight = it->nodes.at(j).height;
                                    lastendangle = it->nodes.at(j).angleend;
                                    lastangle = it->nodes.at(j).angle;
                                    lastnum = num;

                                }
                            }
                            else if((it->nodes.at(j).height - lastheight)>heightdiff_thre)
                            {
                                float endangle = it->nodes.at(j).anglebegin;
                                float beginangle = lastangle;
		                        float anglediff = endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;
                                if(anglediff < anglediff_thre)
                                {
                                    isvalid[lastnum] = 0;
                                }
                                lastheight = it->nodes.at(j).height;
                                lastendangle = it->nodes.at(j).angleend;
                                lastangle = it->nodes.at(j).angle;
                                lastnum = num;

                            }
                            else
                            {
                                lastheight = it->nodes.at(j).height;
                                lastendangle = it->nodes.at(j).angleend;
                                lastangle = it->nodes.at(j).angle;
                                lastnum = num;

                            }
                        }
                        if(it+1==nextpaths.end()&&lastnum!=0)
                        {
                            if((lastheight - nextpaths[0].nodes.at(j).height)>heightdiff_thre )
                            {
                                float endangle = nextpaths[0].nodes.at(j).angle;
                                float beginangle = lastendangle;
		                        float anglediff = endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;

                                if(anglediff < anglediff_thre)
                                {
                                    isvalid[0] = 0;
                                }
                            }
                            else if((nextpaths[0].nodes.at(j).height - lastheight)>heightdiff_thre)
                            {
                                float endangle = nextpaths[0].nodes.at(j).anglebegin;
                                float beginangle = lastangle;
		                        float anglediff = endangle>=beginangle?endangle-beginangle:360+endangle-beginangle;

                                if(anglediff < anglediff_thre)

                                if(nextpaths[0].nodes.at(j).angle - lastendangle< anglediff_thre )
                                {
                                    isvalid[lastnum] = 0;
                                }

                            }

                        }


                    }


                    int i=0;
                    for(;i<nextpaths.size();i++)
                    {
                        if(isvalid[i])
                        {
                            (*it_path)=nextpaths[i];
                            i++;
                            break;
                        }
                    }
//                    std::cout<<j<<" "<<(*it_path)[j]<<std::endl;
                    for(std::vector<RoadPath>::iterator it=nextpaths.begin()+i;it!=nextpaths.end();it++)
                    {

                        if(isvalid[it-nextpaths.begin()])
                            it_path=temproadpaths.insert(it_path+1,*it);
                    }

                }


            }

        }
        std::cout<<temproadpaths.size()<<std::endl;

        for(std::vector<RoadPath>::iterator it_path=temproadpaths.begin() ; it_path!=temproadpaths.end();it_path++)
        {
            RoadPath path(*it_path);
            if(path.nodes.size()>16)
            {

                for(std::vector<RoadNode>::iterator it=path.nodes.begin();it!=path.nodes.end();it++)
                {
                    //if(it->index<0)
                    //    break;

                    std::cout<<it->index;
                    if(it->valid)
                    {
                        path.validnum.push_back(it-path.nodes.begin());
                        std::cout<<"("<<it->anglebegin<<","<<it->angle<<","<<it->angleend<<","<<it->distance<<")";
                    }
                    std::cout<<"->";
                }
                std::cout<<std::endl;

                if(path.validnum[0]<10)
                    continue;

                paths.push_back(path);
            }
        }

        try{

        std::sort(paths.begin(),paths.end(),std::greater<RoadPath>());
        }
        catch(std::exception &e){
            std::cerr<<"sort err"<<std::endl;
        }

        std::cout<<paths.size()<<std::endl;

        for(std::vector<RoadPath>::iterator it_path=paths.begin() ; it_path!=paths.end();it_path++)
        {
            RoadPath path(*it_path);
            {

                for(std::vector<RoadNode>::iterator it=path.nodes.begin();it!=path.nodes.end();it++)
                {
                    //if(it->index<0)
                    //    break;

                    std::cout<<it->index;
                    if(it->valid)
                    {
                        path.validnum.push_back(it-path.nodes.begin());
                        std::cout<<"("<<it->anglebegin<<","<<it->angle<<","<<it->angleend<<","<<it->distance<<")";
                    }
                    std::cout<<"->";
                }
                std::cout<<std::endl;
            }
        }

    }

}
*/



void RoadBoundaries::clear()
{
    mainroad_leftboundary.points.clear();
    mainroad_rightboundary.points.clear();
    boundaries.clear();
    left_found = false;
    right_found = false;

}

void RoadBoundaries::RotatePoint(double srcx,double srcy,double& dstx,double& dsty,double theta)
{
    theta = theta*CV_PI/180;
    dstx=srcx*cos(theta)-srcy*sin(theta);
    dsty=srcx*sin(theta)+srcy*cos(theta);
}

float RoadBoundaries::Cal_road_boudary_RANSAC(int iteration_num,float threshold_scale,RoadBoundary& boundary,int sample_num,float bias_threshold,int boud_style,int lineposition )
{
	int inter=0;
	int point_num=boundary.points.size();
	int *choesn_point=new int [point_num];
	int max_matched_point_num=0;
    double max_match_sum = 0;
    double max_match_rate = 0;

    double theta = -boundary.pose.theta;

    RoadBoundary tempboundary = boundary;
    for(int i=0;i<boundary.points.size();i++)
    {
        RotatePoint(boundary.points.at(i).x,boundary.points.at(i).y,boundary.points.at(i).x,boundary.points.at(i).y,theta);
    }


	RoadBoundary  boundary_ransac;
	while(inter<iteration_num)
	{
		inter++;
		memset(choesn_point,0,sizeof(int)*point_num);
		boundary_ransac.clear();
		int x=0;//随机选取第x个点坐标
		//随机提取sample_num个点
		int num=sample_num<boundary.points.size()?sample_num:boundary.points.size();
		for (int i=0;i<num;i++)
		{
			do
			{
				x=rand()%point_num;
			} while (choesn_point[x]);
			choesn_point[x]=1;
			boundary_ransac.points.push_back(boundary.points.at(x));
		}
		Cal_road_boudary_LSM(boundary_ransac,boud_style);

		float temp_a0 = boundary_ransac.pose.a0;
        float temp_a1 = boundary_ransac.pose.a1;
        float temp_a2 = boundary_ransac.pose.a2;

		if(lineposition==1)
		{
			if (temp_a0>-1||temp_a0<-15)
				continue;
		}
		if(lineposition==2)
		{
			if (temp_a0<1||temp_a0>15)
				continue;
		}
		int matched_point_num=0;
        double match_sum = 0;
        double weight_sum = 0;
		for (int i=0;i<point_num;i++)
		{
            double weight_scale = 1;
			float x=boundary.points.at(i).x;
			float y=boundary.points.at(i).y;
			float predict_x=temp_a0+temp_a1*y+temp_a2*y*y;
			float del_x=fabs(predict_x-x);
            if(y<5)
                weight_scale = 0.5;
			if(del_x<bias_threshold)
			{
                match_sum+=1*weight_scale;
				matched_point_num++;
			}
            else if(del_x < bias_threshold *2)
            {
                match_sum+=0.5*weight_scale;
            }

            weight_sum += weight_scale;
		}
        float temprate=match_sum/(float)weight_sum;
		if(temprate>threshold_scale)
		{
			max_matched_point_num=matched_point_num;
            max_match_sum = match_sum ;
            max_match_rate = temprate ;
			boundary.pose.a0=temp_a0;
			boundary.pose.a1=temp_a1;
			boundary.pose.a2=temp_a2;
			break;
		}
		else if(temprate > max_match_rate)
		{
			max_matched_point_num=matched_point_num;
            max_match_sum = match_sum ;
            max_match_rate = temprate ;
			boundary.pose.a0=temp_a0;
			boundary.pose.a1=temp_a1;
			boundary.pose.a2=temp_a2;
		}


	}
	delete[] choesn_point;
	float matched_rate=max_match_rate;
    boundary.pose.matchrate = matched_rate;

    boundary.points = tempboundary.points;
	return matched_rate;
}


float RoadBoundaries::Cal_road_boudary_LSM(RoadBoundary& boundary,int boud_style)
{
	if (boud_style==2)
	{
		double* px=new double[3*3];
		double* py=new double[3];

        memset(px,0,sizeof(double)*9);
        memset(py,0,sizeof(double)*3);

		for (int i=0;i<boundary.points.size();i++)
		{
			float x=boundary.points.at(i).x;
			float y=boundary.points.at(i).y;
			px[0] +=1;
			px[1] +=y;
			px[2] +=y*y;
			px[5] +=y*y*y;
			px[8] +=y*y*y*y;
			py[0] +=x;
			py[1] +=x*y;
			py[2] +=x*y*y;
		}
		px[3]=px[1];
		px[4]=px[2];
		px[6]=px[2];
		px[7]=px[5];
		CvMat xMat=cvMat(3,3,CV_64FC1,px);
		CvMat yMat=cvMat(3,1,CV_64FC1,py);
		CvMat* result=cvCreateMat(3,1,CV_64FC1);
		cvInvert(&xMat,&xMat,0);
		cvGEMM(&xMat,&yMat,1,NULL,0,result,0);
		boundary.pose.a0=cvmGet(result,0,0);
		boundary.pose.a1=cvmGet(result,1,0);
		boundary.pose.a2=cvmGet(result,2,0);
		delete []px;
		delete []py;
		cvReleaseMat(&result);
	}
	if (boud_style == 1)
	{
		double* px=new double[2*2];
		double* py=new double[2];
        memset(px,0,sizeof(double)*4);
        memset(py,0,sizeof(double)*2);
		for (int i=0;i<boundary.points.size();i++)
		{
			float x=boundary.points.at(i).x;
			float y=boundary.points.at(i).y;
			px[0] +=1;
			px[1] +=y;
			px[2] +=y;
			px[3] +=y*y;
			py[0] +=x;
			py[1] +=x*y;
		}
		CvMat xMat=cvMat(2,2,CV_64FC1,px);
		CvMat yMat=cvMat(2,1,CV_64FC1,py);
		CvMat* result=cvCreateMat(2,1,CV_64FC1);
		cvInvert(&xMat,&xMat,0);
		cvGEMM(&xMat,&yMat,1,NULL,0,result,0);
		boundary.pose.a0=cvmGet(result,0,0);
		boundary.pose.a1=cvmGet(result,1,0);
		boundary.pose.a2=0;
		delete []px;
		delete []py;
		cvReleaseMat(&result);
	}
	return 0;
}


bool RoadBoundaries::getRoadBoundary(const RoadPaths& roadpaths,const std::vector<CloudPointSections>& cloudsections ,double front_angle )
{
    clear();
    float mainroadfrontangle=0;
    int mainroadfrontnum = -1;
    RoadPath mainroadfront;
    bool front_found = false;

    double minfrontanglediff = 360;
    float mainroadbackangle=0;
    int mainroadbacknum = -1;
    RoadPath mainroadback;
    bool back_found = false;
    for(std::vector<RoadPath>::const_iterator it_path=roadpaths.paths.begin() ; it_path!=roadpaths.paths.end();it_path++)
    {
        RoadPath path(*it_path);
        if(path.validnum.size()<3&&path.nodes.size()>=18)
        {
            RoadNode tempnode = path.nodes[path.nodes.size()-1];

            if((tempnode.angleend >= tempnode.anglebegin&& tempnode.anglebegin < front_angle && tempnode.angleend > front_angle)||
                   (tempnode.angleend < tempnode.anglebegin && tempnode.angleend > front_angle))
            {
                front_found = true;
                mainroadfront = path;
                mainroadfrontnum = it_path - roadpaths.paths.begin();
                mainroadfrontangle = tempnode.angle;
                continue;
            }

            if(tempnode.anglebegin < 270 && tempnode.angleend >270)
            {
                back_found = true;
                mainroadback = path;
                mainroadbacknum = it_path - roadpaths.paths.begin();
                mainroadbackangle = tempnode.angle;
                continue;
            }

            if(!front_found)
            {
                double tempbeginanglediff = tempnode.anglebegin> 270?tempnode.anglebegin-360-front_angle:tempnode.anglebegin-front_angle;
                double tempendanglediff = tempnode.angleend> 270?tempnode.angleend-360-front_angle:tempnode.angleend-front_angle;
                double tempanglediff = fabs(tempbeginanglediff)>fabs(tempendanglediff)?tempendanglediff:tempbeginanglediff;
                if(fabs(tempanglediff)<fabs(minfrontanglediff))
                {
                    minfrontanglediff = tempanglediff;
                    mainroadfrontangle = tempnode.angle ;

                    mainroadfrontnum = it_path - roadpaths.paths.begin();
                }
            }

            if(!back_found && fabs(tempnode.angle - 270)<fabs(mainroadbackangle - 270))
            {
                mainroadbackangle = tempnode.angle ;
                mainroadbacknum = it_path - roadpaths.paths.begin();
            }

//            std::cout<<"angle:"<<tempnode.angle<<std::endl;
        }
    }

    if(!front_found && mainroadfrontnum>=0 && fabs(minfrontanglediff)<90)
    {
        mainroadfront = roadpaths.paths.at(mainroadfrontnum);
        front_found = true;
    }

    if(!back_found && mainroadbacknum>=0 && fabs(mainroadbackangle - 270)<90)
    {
        mainroadback = roadpaths.paths.at(mainroadbacknum);
        back_found = true;
    }

    if(front_found && back_found)
    {

        for(std::vector<RoadNode>::iterator it=mainroadfront.nodes.begin();it!=mainroadfront.nodes.end();it++)
        {
            RoadNode tempnode = *it;
            int j = it - mainroadfront.nodes.begin();
            int index = tempnode.index;
            CloudPointSection tempsection = cloudsections[j].sections[index];
            if(tempsection.minpoint.angle > 179 && tempsection.minpoint.angle < 181 &&
                    tempsection.maxpoint.angle > 179 && tempsection.maxpoint.angle < 181)
                continue;
            float anglediff= mainroadfrontangle>tempsection.minpoint.angle ?mainroadfrontangle-tempsection.minpoint.angle :360+mainroadfrontangle-tempsection.minpoint.angle ;
//            std::cout<<"anglediff:"<<anglediff<<"\tfrontangle:"<<mainroadfrontangle<<"\tminpointangle"<<tempsection.minpoint.angle<<std::endl;
            if(anglediff < 120  &&!(fabs(tempsection.minpoint.x)<1&&fabs(tempsection.minpoint.y)<1.5&&tempsection.minpoint.height == 0.0))
            {
        	tempsection.minpoint.layer = j;
                mainroad_rightboundary.points.push_back(tempsection.minpoint);
            }

            anglediff= tempsection.maxpoint.angle > mainroadfrontangle ? tempsection.maxpoint.angle -mainroadfrontangle :360-mainroadfrontangle+tempsection.maxpoint.angle ;
            if(anglediff < 120 &&!(fabs(tempsection.maxpoint.x)<1&&fabs(tempsection.maxpoint.y)<1.5&&tempsection.maxpoint.height == 0.0))
            {
        	tempsection.maxpoint.layer = j;
                mainroad_leftboundary.points.push_back(tempsection.maxpoint);
            }

        }

        for(std::vector<RoadNode>::iterator it=mainroadback.nodes.begin();it!=mainroadback.nodes.end();it++)
        {
            RoadNode tempnode = *it;
            int j = it - mainroadback.nodes.begin();
            int index = tempnode.index;
            CloudPointSection tempsection = cloudsections[j].sections[index];
            if(tempsection.minpoint.angle > 179 && tempsection.minpoint.angle < 181 &&
                    tempsection.maxpoint.angle > 179 && tempsection.maxpoint.angle < 181)
                continue;

            float anglediff= mainroadbackangle>tempsection.minpoint.angle ?mainroadbackangle-tempsection.minpoint.angle :360+mainroadbackangle-tempsection.minpoint.angle ;
            if(anglediff < 120 &&!(fabs(tempsection.minpoint.x)<1&&fabs(tempsection.minpoint.y)<1.5&&tempsection.minpoint.height == 0.0))
            {
        	tempsection.minpoint.layer = j;
                mainroad_leftboundary.points.push_back(tempsection.minpoint);
            }

            anglediff= tempsection.maxpoint.angle > mainroadbackangle ? tempsection.maxpoint.angle -mainroadbackangle :360-mainroadbackangle+tempsection.maxpoint.angle ;
            if(anglediff < 120 &&!(fabs(tempsection.maxpoint.x)<1&&fabs(tempsection.maxpoint.y)<1.5&&tempsection.maxpoint.height == 0.0))
            {
        	tempsection.maxpoint.layer = j;
                mainroad_rightboundary.points.push_back(tempsection.maxpoint);
            }

        }

        if(0)
	{

//	  std::cout<<"leftboundary:"<<mainroad_leftboundary.points.size()<<std::endl;
//	  std::cout<<"points:";
	  for(std::vector<POINT_2F>::iterator it = mainroad_leftboundary.points.begin();it!=mainroad_leftboundary.points.end();it++)
	  {
	      POINT_2F temppoint = *it;
//	      std::cout<<"("<<temppoint.x<<","<<temppoint.y<<","<<temppoint.weight<<")->";
	  }
//	  std::cout<<std::endl;

//	  std::cout<<"rightboundary:"<<mainroad_rightboundary.points.size()<<std::endl;
//	  std::cout<<"points:";
	  for(std::vector<POINT_2F>::iterator it = mainroad_rightboundary.points.begin();it!=mainroad_rightboundary.points.end();it++)
	  {
	      POINT_2F temppoint = *it;
//	      std::cout<<"("<<temppoint.x<<","<<temppoint.y<<","<<temppoint.weight<<")->";
	  }
//	  std::cout<<std::endl;

	  float l_matched_rate ;
	  float r_matched_rate ;

	  l_matched_rate= Cal_road_boudary_RANSAC(300,0.9,mainroad_leftboundary,3,0.2,1,1/*left*/);
	  left_found = true;
	  mainroad_leftboundary.pose.style = 1;
	  mainroad_leftboundary.pose.theta = 0;
	  r_matched_rate = Cal_road_boudary_RANSAC(300,0.9,mainroad_rightboundary,3,0.2,1,2/*right*/);
	  right_found = true;
	  mainroad_rightboundary.pose.style = 1;
	  mainroad_rightboundary.pose.theta = 0;
//	  std::cout<<"l_match:"<<l_matched_rate<<"\ta0"<<mainroad_leftboundary.pose.a0<<"\ta1"<<mainroad_leftboundary.pose.a1<<"\ta2"<<mainroad_leftboundary.pose.a2<<std::endl;
//	  std::cout<<"r_match:"<<r_matched_rate<<"\ta0"<<mainroad_rightboundary.pose.a0<<"\ta1"<<mainroad_rightboundary.pose.a1<<"\ta2"<<mainroad_rightboundary.pose.a2<<std::endl;

	  float angle_thre = 150;
	  float match_thre = 0.5;
	  if((mainroadbackangle - mainroadfrontangle <angle_thre || mainroadbackangle - mainroadfrontangle > 360 - angle_thre ) &&(l_matched_rate<match_thre && r_matched_rate <match_thre) )
	  {
	      float temptheta = (mainroadfrontangle + mainroadbackangle)/2 - 180;
	      float l_match_rate_max = 0;
	      float r_match_rate_max = 0;
	      RoadBoundary temp_leftboundary=mainroad_leftboundary;
	      RoadBoundary temp_rightboundary=mainroad_rightboundary;
	      for(double d_theta = -10 ; d_theta<=10 ; d_theta+=2)
	      {
		  float theta = temptheta + d_theta;
		  temp_leftboundary.pose.theta = theta;
		  l_matched_rate= Cal_road_boudary_RANSAC(300,0.9,temp_leftboundary,5,0.3,2,1/*left*/);
		  temp_rightboundary.pose.theta = theta;
		  r_matched_rate = Cal_road_boudary_RANSAC(300,0.9,temp_rightboundary,5,0.3,2,2/*right*/);
		  if(l_match_rate_max < l_matched_rate)
		  {
		      l_match_rate_max = l_matched_rate;
		      mainroad_leftboundary = temp_leftboundary;
		  }

		  if(r_match_rate_max < r_matched_rate)
		  {
		      r_match_rate_max = r_matched_rate;
		      mainroad_rightboundary = temp_rightboundary;
		  }

	      }
	      left_found = true;
	      mainroad_leftboundary.pose.style = 2;
	      right_found = true;
	      mainroad_rightboundary.pose.style = 2;


//	      std::cout<<"left_theta="<<mainroad_leftboundary.pose.theta<<"\tright_theta="<<mainroad_rightboundary.pose.theta<<std::endl;
//	      std::cout<<"l_match:"<<l_matched_rate<<"\ta0"<<mainroad_leftboundary.pose.a0<<"\ta1"<<mainroad_leftboundary.pose.a1<<"\ta2"<<mainroad_leftboundary.pose.a2<<std::endl;
//	      std::cout<<"r_match:"<<r_matched_rate<<"\ta0"<<mainroad_rightboundary.pose.a0<<"\ta1"<<mainroad_rightboundary.pose.a1<<"\ta2"<<mainroad_rightboundary.pose.a2<<std::endl;

	  }
	  fs_boundary<<mainroad_leftboundary.pose.theta<<"\t"<<mainroad_rightboundary.pose.theta<<"\t"
	      <<mainroad_leftboundary.pose.a0<<"\t"<<mainroad_rightboundary.pose.a0<<"\t"
	      <<mainroad_leftboundary.pose.a1<<"\t"<<mainroad_rightboundary.pose.a1<<"\t"
	      <<mainroad_leftboundary.pose.a2<<"\t"<<mainroad_rightboundary.pose.a2<<std::endl;



	  MainBoundaryKalman();

	  fs_boundary<<mainroad_leftboundary.pose.theta<<"\t"<<mainroad_rightboundary.pose.theta<<"\t"
	      <<mainroad_leftboundary.pose.a0<<"\t"<<mainroad_rightboundary.pose.a0<<"\t"
	      <<mainroad_leftboundary.pose.a1<<"\t"<<mainroad_rightboundary.pose.a1<<"\t"
	      <<mainroad_leftboundary.pose.a2<<"\t"<<mainroad_rightboundary.pose.a2<<std::endl;

	 }
        return true;

    }
    return false;

}



void RoadBoundaries::MainBoundaryKalman()
{
    double& l_a0 = mainroad_leftboundary.pose.a0;
    double& l_a1 = mainroad_leftboundary.pose.a1;
    double& l_a2 = mainroad_leftboundary.pose.a2;

    double& r_a0 = mainroad_rightboundary.pose.a0;
    double& r_a1 = mainroad_rightboundary.pose.a1;
    double& r_a2 = mainroad_rightboundary.pose.a2;
    double c_a0 , c_a1 , c_a2;

    double& l_theta = mainroad_leftboundary.pose.theta;
    double& r_theta = mainroad_rightboundary.pose.theta;

    bool left_boundary_vaild = false;
    bool right_boundary_valid = false;


    double& l_matched_scale = mainroad_leftboundary.pose.matchrate;
    double& r_matched_scale = mainroad_rightboundary.pose.matchrate;
    if(l_matched_scale > 0.4)
        left_boundary_vaild = true;
    if(r_matched_scale > 0.4)
        right_boundary_valid = true;
	if (1)
	{
		//左有效
		if(left_boundary_vaild&&!right_boundary_valid)
		{
			c_a0=0;
			c_a1=l_a1;
			c_a2=l_a2;
		}
		//右有效
		else if (!left_boundary_vaild&&right_boundary_valid)
		{
			c_a0=0;
			c_a1=r_a1;
			c_a2=r_a2;
		}
		//都有效
		else if(left_boundary_vaild&&right_boundary_valid)
		{
			c_a0=0;
			c_a1=(l_a1+r_a1)/2;
			c_a2=(l_a2+r_a2)/2;
		}
		else
		{
			//比较左右两边界的拟合度进行选择
			if(l_matched_scale>r_matched_scale)
			{
				c_a0=0;
				c_a1=l_a1;
				c_a2=l_a2;
			}
			else
			{
				c_a0=0;
				c_a1=r_a1;
				c_a2=r_a2;
			}

		}

	}
    //初始化
    if (!mainroadkalman_inited)
    {
        mainroad_failed_count=0;
        mainroadkalman=cvCreateKalman(11,11,0);
        const float F[]=
        {1,0,0,0,0,0,0,0,0,0,0,
        0,1,0,0,0,0,0,0,0,0,0,
        -0.5,0.5,0.5,0,0,0,0,0,0,0,0,
        0,0,0,1,0,0,0,0,0,0,0,
        0,0,0,0,1,0,0,0,0,0,0,
        0,0,0,0,0,1,0,0,0,0,0,
        0,0,0,0,0,0,1,0,0,0,0,
        0,0,0,0,0,0,0,1,0,0,0,
        0,0,0,0,0,0,0,0,1,0,0,
        0,0,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,0,0,0,1,
        };//初始化传递矩阵
        memcpy(mainroadkalman->transition_matrix->data.fl,F,sizeof(F));
        const float H[]=
        {
            1,0,0,0,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,0,0,0,
            0,0,1,0,0,0,0,0,0,0,0,
            0,0,0,1,0,0,0,0,0,0,0,
            0,0,0,0,1,0,0,0,0,0,0,
            0,0,0,0,0,1,0,0,0,0,0,
            0,0,0,0,0,0,1,0,0,0,0,
            0,0,0,0,0,0,0,1,0,0,0,
            0,0,0,0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,0,0,0,0,1,
        };
        memcpy(mainroadkalman->measurement_matrix->data.fl,H,sizeof(H));
        const float Q[]=
        {
            1,0,0,0,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,0,0,0,
            0,0,1,0,0,0,0,0,0,0,0,
            0,0,0,0.1,0,0,0,0,0,0,0,
            0,0,0,0,0.01,0,0,0,0,0,0,
            0,0,0,0,0,0.1,0,0,0,0,0,
            0,0,0,0,0,0,0.01,0,0,0,0,
            0,0,0,0,0,0,0,0.1,0,0,0,
            0,0,0,0,0,0,0,0,0.1,0,0,
            0,0,0,0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,0,0,0,0,1,
        };
        memcpy(mainroadkalman->process_noise_cov->data.fl,Q,sizeof(Q));

        const float R[]=
        {
        1,0,0,0,0,0,0,0,0,0,0,
        0,1,0,0,0,0,0,0,0,0,0,
        0,0,1,0,0,0,0,0,0,0,0,
        0,0,0,1,0,0,0,0,0,0,0,
        0,0,0,0,0.1,0,0,0,0,0,0,
        0,0,0,0,0,1,0,0,0,0,0,
        0,0,0,0,0,0,0.1,0,0,0,0,
        0,0,0,0,0,0,0,1,0,0,0,
        0,0,0,0,0,0,0,0,0.1,0,0,
        0,0,0,0,0,0,0,0,0,5,0,
        0,0,0,0,0,0,0,0,0,0,5,
        };
        memcpy(mainroadkalman->measurement_noise_cov->data.fl,R,sizeof(R));

        const float P[]=
        {
        10,0,0,0,0,0,0,0,0,0,0,
        0,10,0,0,0,0,0,0,0,0,0,
        0,0,10,0,0,0,0,0,0,0,0,
        0,0,0,10,0,0,0,0,0,0,0,
        0,0,0,0,10,0,0,0,0,0,0,
        0,0,0,0,0,10,0,0,0,0,0,
        0,0,0,0,0,0,10,0,0,0,0,
        0,0,0,0,0,0,0,10,0,0,0,
        0,0,0,0,0,0,0,0,10,0,0,
        0,0,0,0,0,0,0,0,0,10,0,
        0,0,0,0,0,0,0,0,0,0,10,
        };
        memcpy(mainroadkalman->error_cov_post->data.fl,P,sizeof(P));
        mainroadkalman_inited = true;
        CvMat* measurement = cvCreateMat(11,1,CV_32FC1);
        mainroadkalman->state_post->data.fl[0] =l_a0;
        mainroadkalman->state_post->data.fl[1] = r_a0;
        mainroadkalman->state_post->data.fl[2] = r_a0-l_a0;
        mainroadkalman->state_post->data.fl[3] =l_a1;
        mainroadkalman->state_post->data.fl[4] =l_a2;
        mainroadkalman->state_post->data.fl[5] =r_a1;
        mainroadkalman->state_post->data.fl[6] =r_a2;
        mainroadkalman->state_post->data.fl[7] =c_a1;
        mainroadkalman->state_post->data.fl[8] =c_a2;
        mainroadkalman->state_post->data.fl[9] =l_theta;
        mainroadkalman->state_post->data.fl[10] =r_theta;
        cvKalmanPredict(mainroadkalman,0);
        measurement->data.fl[0] = l_a0;
        measurement->data.fl[1] = r_a0;
        measurement->data.fl[2] = r_a0-l_a0;
        measurement->data.fl[3] = l_a1;
        measurement->data.fl[4] = l_a2;
        measurement->data.fl[5] = r_a1;
        measurement->data.fl[6] = r_a2;
        measurement->data.fl[7] = c_a1;
        measurement->data.fl[8] = c_a2;
        measurement->data.fl[9] = l_theta;
        measurement->data.fl[10] = r_theta;
        cvKalmanCorrect(mainroadkalman,measurement);
        l_a0=mainroadkalman->state_post->data.fl[0];
        l_a1=mainroadkalman->state_post->data.fl[3];
        l_a2=mainroadkalman->state_post->data.fl[4];
        r_a0=mainroadkalman->state_post->data.fl[1];
        r_a1=mainroadkalman->state_post->data.fl[5];
        r_a2=mainroadkalman->state_post->data.fl[6];
        c_a1=mainroadkalman->state_post->data.fl[7];
        c_a2=mainroadkalman->state_post->data.fl[8];
        l_theta=mainroadkalman->state_post->data.fl[9];
        r_theta=mainroadkalman->state_post->data.fl[10];

/*        std::cout<<"init: l_a0:"<<l_a0<<"\tl_a1:"<<l_a1<<"\tl_a2:"<<l_a2<<std::endl;
        std::cout<<"init: r_a0:"<<r_a0<<"\tr_a1:"<<r_a1<<"\tr_a2:"<<r_a2<<std::endl;
        std::cout<<"init: c_a0:"<<c_a0<<"\tc_a1:"<<c_a1<<"\tc_a2:"<<c_a2<<std::endl;*/
        cvReleaseMat(&measurement);
    }
    //mainroadkalman更新
    else
    {
        CvMat* measurement = cvCreateMat(11,1,CV_32FC1);
        cvKalmanPredict(mainroadkalman,0);
        measurement->data.fl[7] = c_a1;
        measurement->data.fl[8] = c_a2;
        measurement->data.fl[9] = l_theta;
        measurement->data.fl[10] = r_theta;
        //两边结果有效
        if (left_boundary_vaild&&right_boundary_valid)
        {
            measurement->data.fl[0] = l_a0;
            measurement->data.fl[1] = r_a0;
            measurement->data.fl[2] = r_a0-l_a0;
            measurement->data.fl[3] = l_a1;
            measurement->data.fl[4] = l_a2;
            measurement->data.fl[5] = r_a1;
            measurement->data.fl[6] = r_a2;
            mainroad_failed_count=0;
        }
        //左边结果有效
        else if (left_boundary_vaild&&!right_boundary_valid)
        {
            measurement->data.fl[0] = l_a0;
            measurement->data.fl[1] = mainroadkalman->state_pre->data.fl[1];
            measurement->data.fl[2] = measurement->data.fl[1]-measurement->data.fl[0];
            measurement->data.fl[3] = l_a1;
            measurement->data.fl[4] = l_a2;
            measurement->data.fl[5] = l_a1;//mainroadkalman->state_pre->data.fl[5];
            measurement->data.fl[6] = l_a2;//mainroadkalman->state_pre->data.fl[6];
            mainroad_failed_count=0;
        }
         //右边结果有效
        else if (!left_boundary_vaild&&right_boundary_valid)
        {
            measurement->data.fl[0] = mainroadkalman->state_pre->data.fl[0];
            measurement->data.fl[1] = r_a0;
            measurement->data.fl[2] =  measurement->data.fl[1]-measurement->data.fl[0];
            measurement->data.fl[3] = r_a1;//mainroadkalman->state_pre->data.fl[3];
            measurement->data.fl[4] = r_a2;//mainroadkalman->state_pre->data.fl[4];
            measurement->data.fl[5] = r_a1;
            measurement->data.fl[6] = r_a2;
            mainroad_failed_count=0;
        }
        else
        {
            measurement->data.fl[0] = mainroadkalman->state_pre->data.fl[0];
            measurement->data.fl[1] = mainroadkalman->state_pre->data.fl[1];
            measurement->data.fl[2] =  measurement->data.fl[1]-measurement->data.fl[0];
            measurement->data.fl[3] = mainroadkalman->state_pre->data.fl[3];
            measurement->data.fl[4] = mainroadkalman->state_pre->data.fl[4];
            measurement->data.fl[5] = mainroadkalman->state_pre->data.fl[5];
            measurement->data.fl[6] =  mainroadkalman->state_pre->data.fl[6];
            mainroad_failed_count++;
        }
        cvKalmanCorrect(mainroadkalman,measurement);
        l_a0=mainroadkalman->state_post->data.fl[0];
        l_a1=mainroadkalman->state_post->data.fl[3];
        l_a2=mainroadkalman->state_post->data.fl[4];
        r_a0=mainroadkalman->state_post->data.fl[1];
        r_a1=mainroadkalman->state_post->data.fl[5];
        r_a2=mainroadkalman->state_post->data.fl[6];
        c_a1=mainroadkalman->state_post->data.fl[7];
        c_a2=mainroadkalman->state_post->data.fl[8];
        l_theta=mainroadkalman->state_post->data.fl[9];
        r_theta=mainroadkalman->state_post->data.fl[10];
        cvReleaseMat(&measurement);
        if (mainroad_failed_count==5)
        {
            mainroadkalman_inited=false;
        }

/*        std::cout<<"l_a0:"<<l_a0<<"\tl_a1:"<<l_a1<<"\tl_a2:"<<l_a2<<std::endl;
        std::cout<<"r_a0:"<<r_a0<<"\tr_a1:"<<r_a1<<"\tr_a2:"<<r_a2<<std::endl;
        std::cout<<"c_a0:"<<c_a0<<"\tc_a1:"<<c_a1<<"\tc_a2:"<<c_a2<<std::endl;*/
    }
}
