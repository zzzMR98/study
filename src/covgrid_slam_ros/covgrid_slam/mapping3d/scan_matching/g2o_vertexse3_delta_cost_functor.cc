
#include "covgrid_slam/mapping3d/scan_matching/g2o_vertexse3_delta_cost_functor.h"

#include "Eigen/Core"

#include <g2o/types/slam3d/isometry3d_gradients.h>

#include "glog/logging.h"
namespace mapping3d {
namespace scan_matching {


void VertexSE3DeltaEdge::linearizeOplus()
{
	g2o::VertexSE3* vp0 = static_cast<g2o::VertexSE3*>(_vertices[0]);
    g2o::Isometry3D E;
    g2o::Isometry3D Z, X, P;
    X=vp0->estimate();
    P.setIdentity();

    Z=_measurement;
    g2o::internal::computeEdgeSE3PriorGradient(E, _jacobianOplusXi, Z, X,P);
//    LOG(INFO)<<_jacobianOplusXi;

}

}  // namespace scan_matching
}  // namespace mapping_3d

