/*
 * Copyright 2016 The covgrid_slam Authors
 *
	function: rotation delta cost function
 */

#ifndef MAPPING_3D_SCAN_G2O_MATCHING_ROTATION_DELTA_COST_FUNCTOR_H_
#define MAPPING_3D_SCAN_G2O_MATCHING_ROTATION_DELTA_COST_FUNCTOR_H_

#include "Eigen/Core"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/types_slam3d.h"
//#include "g2o/core/base_binary_edge.h"

namespace mapping3d {
namespace scan_matching {

namespace types_icp {
  void init();
}

// Computes the cost of inserting occupied space described by the point cloud
// into the map. The cost increases with the amount of free space that would be
// replaced by occupied space.
class VertexSE3DeltaEdge: public g2o::BaseUnaryEdge<6,g2o::Isometry3D,g2o::VertexSE3>{
 public:


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  // Creates an VertexSE3DeltaEdge using the specified grid, 'rotation' to
  // add to all poses, and point cloud.
  explicit VertexSE3DeltaEdge(const g2o::Isometry3D& measurement)
    {
	  setMeasurement(measurement);
    }


     // I/O functions
     bool read(std::istream& is){}
     bool write(std::ostream& os) const{}

     void computeError() override
     {


       // from <ViewPoint> to <Point>
       const g2o::VertexSE3 *vp = static_cast<const g2o::VertexSE3*>(_vertices[0]);

       g2o::Isometry3D delta=_measurement_inverse_ * vp ->estimate();
       _error = g2o::internal::toVectorMQT(delta);
       // get vp1 point into vp0 frame
       // could be more efficient if we computed this transform just once


     }

     // try analytic jacobians

     void linearizeOplus() override;

     virtual void setMeasurement(const g2o::Isometry3D& m){
       _measurement = m;
       _measurement_inverse_ = m.inverse();
     }

 private:
  g2o::Isometry3D _measurement_inverse_;

};

}  // namespace scan_matching
}  // namespace mapping_3d

#endif  // MAPPING_3D_SCAN_G2O_MATCHING_ROTATION_DELTA_COST_FUNCTOR_H_
