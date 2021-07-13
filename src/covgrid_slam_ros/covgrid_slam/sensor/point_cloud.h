/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "ivcommon/transform/rigid_transform.h"
#include "glog/logging.h"

namespace sensor {

class Point : public Eigen::Vector3d{
public:
  enum class Type { KVoid =0 ,kFlat=1, kLine=2, kCluster=3 };
  Point(const Eigen::Vector3d& vector,const float tempringandtime=0,const float tempintensity=0,const Type temptype = Type::KVoid)
  :Eigen::Vector3d(vector)
  ,ringandtime(tempringandtime)
  ,intensity(tempintensity)
  ,type(temptype)
  {

  }
  Point():type(Type::KVoid)
  {
    ringandtime = 0;
    intensity = 0;
  }

  Point(const Point& point) = default;
//  {
//    intensity = point.intensity;
//  }
  Point& operator=(const Point& point) = default;
//  {
//    Eigen::Vector3d::operator = (point);
//    intensity = point.intensity;
//    return *this;
//  }

  float ringandtime;
  float intensity;
  Type type;
};

template <typename FloatType>
Point operator*(
    const ivcommon::transform::Rigid3<FloatType>& rigid,
    const Point& point) {
  return {rigid.rotation() * point + rigid.translation(),point.ringandtime,point.intensity,point.type};
}

typedef std::vector<Point> PointCloud;

struct PointCloudWithIntensities {
  PointCloud points;
  std::vector<float> intensities;
};

// Transforms 'point_cloud' according to 'transform'.
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const ivcommon::transform::Rigid3d& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
PointCloud Crop(const PointCloud& point_cloud, float min_z, float max_z);

}  // namespace sensor

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
