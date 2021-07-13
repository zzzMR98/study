/*!
* \file feature_grid.h
* \brief 特征栅格匹配相关函数
*
* 搜索最近邻特征栅格，并计算特征栅格损失函数
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_FEATURE_GRID_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_FEATURE_GRID_H_

#include <cmath>

#include "covgrid_slam/mapping3d/hybrid_grid.h"

namespace mapping3d {
namespace scan_matching {

// Interpolates between HybridGrid probability voxels. We use the tricubic
// interpolation which interpolates the values and has vanishing derivative at
// these points.
//
// This class is templated to work with the autodiff that Ceres provides.
// For this reason, it is also important that the interpolation scheme be
// continuously differentiable.
/// \brief 特征栅格匹配相关
///	搜索最近邻特征栅格，并计算特征栅格损失函数
class FeatureGrid {
 public:
  explicit FeatureGrid(const FeatureHybridGrid& hybrid_grid)
      : feature_hybrid_grid_(hybrid_grid) {}

  FeatureGrid(const FeatureGrid&) = delete;
  FeatureGrid& operator=(const FeatureGrid&) = delete;

  // Returns the interpolated probability at (x, y, z) of the HybridGrid
  // used to perform the interpolation.
  //
  // This is a piecewise, continuously differentiable function. We use the
  // scalar part of Jet parameters to select our interval below. It is the
  // tensor product volume of piecewise cubic polynomials that interpolate
  // the values, and have vanishing derivative at the interval boundaries.

  template <typename T>
  const T GetMinDistance(const T& x, const T& y, const T& z) const {
    double x1, y1, z1, x2, y2, z2;
    ComputeInterpolationDataPoints(x, y, z, &x1, &y1, &z1, &x2, &y2, &z2);//1,2 分别代表
    T mindis = T(1.);
    double bel_scale = 1.;
    const Eigen::Array3i index1 =
        feature_hybrid_grid_.GetCellIndex(Eigen::Vector3d(x1, y1, z1));
    for(int k=0;k<2;k++)
      {
		for(int j=0;j<2;j++)
		  {
			for(int i=0;i<2;i++)
			  {

				auto v = feature_hybrid_grid_.GetProbability(index1+Eigen::Array3i(i,j,k));
				if(v<0.501)
				  continue;
				auto v1 = feature_hybrid_grid_.value(index1+Eigen::Array3i(i,j,k));

				if(v1.type!=FeatureElement::Type::KVoid&&v1.type!=FeatureElement::Type::KInit)
				  {

					double tempbel = 1.;
					T dis = v1.getdistance(x,y,z,tempbel);
//					dis = dis * (1.-v);
					if(mindis > dis)
					  {
						bel_scale = tempbel * v;
						mindis = dis;
					  }

				  }

			  }
		  }
      }

    mindis = mindis * bel_scale;
    limitmax(mindis,1.);
    return mindis;
  }

  const Eigen::Vector3d GetMinDistancesquare(const double& x, const double& y, const double& z,Eigen::Matrix<double,3,3>& information) const {
    double x1, y1, z1, x2, y2, z2;
    ComputeInterpolationDataPoints(x, y, z, &x1, &y1, &z1, &x2, &y2, &z2);//1,2 分别代表
    Eigen::Vector3d mindis (1.,1.,1.);
    double bel_scale = 1.;
    information = Eigen::Matrix<double,3,3>::Zero();

    const Eigen::Array3i index1 =
        feature_hybrid_grid_.GetCellIndex(Eigen::Vector3d(x1, y1, z1));
    for(int k=0;k<2;k++)
      {
		for(int j=0;j<2;j++)
		  {
			for(int i=0;i<2;i++)
			  {

				auto v = feature_hybrid_grid_.GetProbability(index1+Eigen::Array3i(i,j,k));
				if(v<0.501)
				  continue;
				auto v1 = feature_hybrid_grid_.value(index1+Eigen::Array3i(i,j,k));

				if(v1.type!=FeatureElement::Type::KVoid&&v1.type!=FeatureElement::Type::KInit)
				  {
					Eigen::Matrix<double,3,3> tempinformation = Eigen::Matrix<double,3,3>::Zero();
					double tempbel = 1.;
					Eigen::Vector3d dis = v1.getdistancesquare(x,y,z,tempbel,tempinformation);
//					dis = dis * (1.-v);
					if(std::fabs(mindis.norm()) > std::fabs(dis.norm()))
					  {
						information = tempinformation;
						bel_scale = tempbel * v;
						mindis = dis;
					  }

				  }

			  }
		  }
      }
//    double orimindis = mindis;
    limitmax(bel_scale,10.);
//    mindis = mindis * bel_scale*bel_scale;
//    limitmax(mindis,1.);
    information *= bel_scale;
    return mindis;
  }
  /// \brief 计算点与栅格特征的距离（损失）
  ///
  /// \param x 点的x坐标
  /// \param y 点的y坐标
  /// \param z 点的z坐标
  /// \param derivative 损失函数的导数
  /// \return 距离损失
  const double GetMinDistancesquare(const double& x, const double& y, const double& z,Eigen::Matrix<double,1,3>& derivative) const {
    double x1, y1, z1, x2, y2, z2;
    ComputeInterpolationDataPoints(x, y, z, &x1, &y1, &z1, &x2, &y2, &z2);//1,2 分别代表
    double mindis = 1.;
    double bel_scale = 1.;
    derivative = Eigen::Matrix<double,1,3>::Zero();

    const Eigen::Array3i index1 =
        feature_hybrid_grid_.GetCellIndex(Eigen::Vector3d(x1, y1, z1));
    for(int k=0;k<2;k++)
      {
		for(int j=0;j<2;j++)
		  {
			for(int i=0;i<2;i++)
			  {

				auto v = feature_hybrid_grid_.GetProbability(index1+Eigen::Array3i(i,j,k));
				if(v<0.501)
				  continue;
				auto v1 = feature_hybrid_grid_.value(index1+Eigen::Array3i(i,j,k));

				if(v1.type!=FeatureElement::Type::KVoid&&v1.type!=FeatureElement::Type::KInit)
				  {
					Eigen::Matrix<double,1,3> tempderivative = Eigen::Matrix<double,1,3>::Zero();
					double tempbel = 1.;
					double dis = v1.getdistancesquare(x,y,z,tempbel,tempderivative);
//					dis = dis * (1.-v);
					if((mindis) > (dis))
					  {
						derivative = tempderivative;
						bel_scale = tempbel * v;
						mindis = dis;
					  }

				  }

			  }
		  }
      }
    double orimindis = mindis;
    mindis = mindis * bel_scale;
    limitmax(mindis,1.);
    derivative *= mindis/orimindis/orimindis;

//    limitmax(mindis,1.);
//    information *= bel_scale;
    return mindis;
  }

  template <typename T>
  const Eigen::Matrix<T,3,1> GetMinDistanceMatrix(const T& x, const T& y, const T& z) const {
    double x1, y1, z1, x2, y2, z2;
    ComputeInterpolationDataPoints(x, y, z, &x1, &y1, &z1, &x2, &y2, &z2);//1,2 分别代表
    Eigen::Matrix<T,3,1> mindis;
    mindis<<T(1.),T(1.),T(1.);
    double bel_scale = 1.;
    const Eigen::Array3i index1 =
        feature_hybrid_grid_.GetCellIndex(Eigen::Vector3d(x1, y1, z1));
    for(int k=0;k<2;k++)
      {
		for(int j=0;j<2;j++)
		  {
			for(int i=0;i<2;i++)
			  {

				auto v = feature_hybrid_grid_.GetProbability(index1+Eigen::Array3i(i,j,k));
				if(v<0.501)
				  continue;
				auto v1 = feature_hybrid_grid_.value(index1+Eigen::Array3i(i,j,k));

				if(v1.type!=FeatureElement::Type::KVoid&&v1.type!=FeatureElement::Type::KInit)
				  {

					double tempbel = 1.;
					Eigen::Matrix<T,3,1> dis = v1.getdistancematrix(x,y,z,tempbel);
//					dis = dis * (1.-v);
					if(ceres::abs(mindis(0,0)) > ceres::abs(dis(0,0)))
					  {
						bel_scale = tempbel * v;
						mindis = dis;
					  }

				  }

			  }
		  }
      }

    mindis = mindis * T(bel_scale);
    limitmax(mindis(0,0),1.);
    limitmax(mindis(1,0),0.5);
    limitmax(mindis(2,0),0.2);
    return mindis;
  }

  template <typename T>
  inline void limitmax(T& val,const double lim) const
  {
    if(val.a > lim)
      val = val/val.a*lim;
  }

  inline void limitmax(double& val,const double lim) const
  {
    if(val > lim)
      val = lim;
  }

  template <typename T>
  T GetResolution() const {
    return T(feature_hybrid_grid_.resolution());
  }

 private:
  template <typename T>
  void ComputeInterpolationDataPoints(const T& x, const T& y, const T& z,
                                      double* x1, double* y1, double* z1,
                                      double* x2, double* y2,
                                      double* z2) const {
    const Eigen::Vector3d lower = CenterOfLowerVoxel(x, y, z);
    *x1 = lower.x();
    *y1 = lower.y();
    *z1 = lower.z();
    *x2 = lower.x() + feature_hybrid_grid_.resolution();
    *y2 = lower.y() + feature_hybrid_grid_.resolution();
    *z2 = lower.z() + feature_hybrid_grid_.resolution();
  }

  // Center of the next lower voxel, i.e., not necessarily the voxel containing
  // (x, y, z). For each dimension, the largest voxel index so that the
  // corresponding center is at most the given coordinate.
  Eigen::Vector3d CenterOfLowerVoxel(const double x, const double y,
                                     const double z) const {
    // Center of the cell containing (x, y, z).
    Eigen::Vector3d center = feature_hybrid_grid_.GetCenterOfCell(
        feature_hybrid_grid_.GetCellIndex(Eigen::Vector3d(x, y, z)));
    // Move to the next lower voxel center.
    if (center.x() > x) {
      center.x() -= feature_hybrid_grid_.resolution();
    }
    if (center.y() > y) {
      center.y() -= feature_hybrid_grid_.resolution();
    }
    if (center.z() > z) {
      center.z() -= feature_hybrid_grid_.resolution();
    }
    return center;
  }

  // Uses the scalar part of a Ceres Jet.
  template <typename T>
  Eigen::Vector3d CenterOfLowerVoxel(const T& jet_x, const T& jet_y,
                                     const T& jet_z) const {
    return CenterOfLowerVoxel(jet_x.a, jet_y.a, jet_z.a);
  }

  // Uses the scalar part of a Ceres Jet.
  template <typename T>
  Eigen::Vector3d makevector(const T& jet_x, const T& jet_y,
                                     const T& jet_z) const {
    return std::move(makevector(jet_x.a, jet_y.a, jet_z.a));
  }


  Eigen::Vector3d makevector(const double x, const double y,
                             const double z) const {
    return std::move(Eigen::Vector3d(x, y, z));
  }

  const FeatureHybridGrid& feature_hybrid_grid_;
};

}  // namespace scan_matching
}  // namespace mapping_3d

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_FEATURE_GRID_H_
