
#ifndef TRAVERSIBALE_AREA_EXTRACTION_COMMON_TIME_CONVERSION_H_
#define TRAVERSIBALE_AREA_EXTRACTION_COMMON_TIME_CONVERSION_H_

#include "common/time.h"
#include "ros/ros.h"

namespace ivcommon {

//将chrono::time_point格式的时间转换为ros标准时间
::ros::Time ToRos(ivcommon::Time time);
//将ros标准时间转为chrono::time_point格式
ivcommon::Time FromRos(const ::ros::Time& time);

}

#endif  // TRAVERSIBALE_AREA_EXTRACTION_COMMON_TIME_CONVERSION_H_
