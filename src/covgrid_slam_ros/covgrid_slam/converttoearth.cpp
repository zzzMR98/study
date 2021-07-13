/*
 * converttoearth.cpp
 *
 *  Created on: Jan 4, 2018
 *      Author: jkj
 */


#include <cmath>

#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>
#include <ros/package.h>

#include <glog/logging.h>
#include "velodyne/mytime.h"

#include "ivcommon/common/time.h"
#include "ivcommon/common/blocking_queue.h"
#include "ivcommon/common/time_conversion.h"
#include <deque>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <util/boostudp/boostudp.h>
#include <util/playback/iv_data_playback.h>
#include <util/utm/utm.h>
#include <sstream>
#include "sensor_driver_msgs/GpswithHeading.h"
#include "velodyne/HDL32Structure.h"
#include "covgrid_slam/mapping/pose_extrapolator.h"
#include "covgrid_slam/mapping3d/local_trajectory_builder.h"
#include "covgrid_slam/mapping3d/local_trajectory_builder_options.h"
#include "covgrid_slam/sensor/data.h"



  void convertpose(string dir)
  {
	  constexpr static double a=6378137;
	  constexpr static double e2= 0.0818192*0.0818192;//e的平方
	  GridZone zone =UTM_ZONE_AUTO;
	  Hemisphere hemi = HEMI_NORTH;

	  string inputfile = dir + "/input.txt";
	  string gpsfile = dir + "/gpsmetricdata.txt";
	  string outputfile = dir + "/trajectory.txt";
	    std::ifstream input(inputfile.c_str());
	    std::ifstream gpsst(gpsfile.c_str());
	    double gpslongitude , gpslatitude ,heading ,gpsx,gpsy,temp;
	    gpsst>>temp>>temp>>temp>>temp>>heading>>gpslatitude>>gpslongitude>>temp;
	    geographic_to_grid(a, e2, gpslatitude*M_PI/180, gpslongitude*M_PI/180
			       , &zone, &hemi, &gpsy, &gpsx);
	    gpsx -= 500000;
	    std::ofstream output(outputfile.c_str());
	    double x,y,theta,time1;
	    //heading -= 2.7159;
	    while(input>>time1)
	    {
	    	double yout,xout,latitude,longitude;
	    	input>>temp>>theta>>temp>>y>>temp>>x;

		    double headingrad = (heading-2.7159)*M_PI/180;
	    	xout = -sin(headingrad)*y + cos(headingrad)*x + gpsx;
	    	yout = sin(headingrad)*x + cos(headingrad)*y + gpsy;
	    	theta = theta + heading;
	    	if(theta>180)
	    		theta -= 360;
	    	if(theta<-180)
	    		theta += 360;
		    grid_to_geographic(a,e2,zone,hemi,
				       yout,xout + 500000
				       ,&latitude,&longitude);
		    output<<std::fixed<<std::setprecision(3)<<time1
		    		<<" "<<xout<<" "<<yout<<" "
		    		<<0<<" "	<<theta<<" "
					<<std::setprecision(7)
		    		<<latitude*180/M_PI<<" "<<longitude*180/M_PI
					<<" "<<0<<" "<<0<<" "<<0<<std::endl;
	    }


  }



int main(int argc, char** argv)
{

  convertpose(argv[1]);

  return 0;
}
