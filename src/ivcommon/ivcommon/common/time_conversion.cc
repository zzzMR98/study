
#include "time_conversion.h"

#include "common/time.h"
#include "ros/ros.h"

namespace ivcommon {

::ros::Time ToRos(ivcommon::Time time) {
  int64 uts_timestamp = ivcommon::ToUniversal(time); //转为世界协调时
  //世界协调时以1970年1月1日0:00:00为原点．ros时间以"0001-01-01 00:00:00.0 +0000"为时间原点，正好719162天
  int64 ns_since_unix_epoch =
      (uts_timestamp -
    		  ivcommon::kUtsEpochOffsetFromUnixEpochInSeconds *
           10000000ll) *
      100ll;
  ros::Time ros_time;
  ros_time.fromNSec(ns_since_unix_epoch);
  return ros_time;
}

// TODO(pedrofernandez): Write test.
ivcommon::Time FromRos(const ::ros::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ivcommon::FromUniversal(
      (time.sec +
    		  ivcommon::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}

}
