/*!
* \file hybrid_grid.h
* \brief 类八叉树栅格地图管理类
*
* 类八叉树栅格地图管理类，多层嵌套
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/
#ifndef CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_
#define CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_

#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <map>
#include <mutex>

#include "Eigen/Core"
#include <Eigen/Eigenvalues>
#include "ivcommon/common/make_unique.h"
#include "ivcommon/common/math.h"
#include "ivcommon/common/port.h"
#include "ivcommon/transform/transform.h"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include <iostream>
#include <fstream>
#include "covgrid_slam/mapping/probability_values.h"
#include "covgrid_slam/sensor/point_cloud.h"
#include "covgrid_slam/mapping3d/proto/hybrid_grid.pb.h"
#include "covgrid_slam/mapping3d/proto/feature_hybrid_grid.pb.h"
#include "covgrid_slam/mapping3d/proto/intensity_hybrid_grid.pb.h"
namespace mapping3d {
/// \brief 三维特征栅格
///
/// 特征栅格的更新及匹配距离计算。这里不存储点云，而是根据每帧的点云实时更新栅格特征。特征包括，线、面、点云分布特征。
  class FeatureElement{
  public:
    static const int KVarietiesThreshold = 5;
    enum class Type { KVoid =0 ,KFlat=1, KLine=2, KCluster=3 ,KInit=4};

    /// \brief 栅格协方差计算中间参数
    ///
    /// 为提高计算效率，不保存点云，而是每帧来了更新计算协方差所需的中间参数。这样可以避免每次重新计算所有点云的各元素和平方和等。
    struct CovariancePara{
      float sum_x_x = 0;
      float sum_x_y = 0;
      float sum_x_z = 0;
      float sum_y_y = 0;
      float sum_y_z = 0;
      float sum_z_z = 0;
      float sum_x = 0;
      float sum_y = 0;
      float sum_z = 0;
      int num = 0;
    };

/// \brief 协方差结果
///
    struct CovarianceResult{
      Eigen::Matrix3d covmat;//!<协方差矩阵
      Eigen::Matrix3d covmat_inv;//!<协方差矩阵的逆
      Eigen::Vector3d average;//!<点云位置期望
      Eigen::Vector3d D ;//!<特征值
      Eigen::Matrix3d V ;//!<特征向量
      double bel_scale;//!<缩放比例
    };

    FeatureElement():type(Type::KVoid)
    ,pointvarieties(0)
    ,lastlasernum(0)
    ,resolution(0)
    ,probability(0)
    ,updated(false)
    ,confirmed(false)
    ,missed(false)
    {
      memset(&covpara,0,sizeof(covpara));
      covresult.covmat = Eigen::Matrix3d::Zero();

    }

    FeatureElement(const FeatureElement&) = default;

    FeatureElement& operator=(const FeatureElement&) = default;


    void clear();


    void push_back(const sensor::Point& point);

    /// \brief 计算点与栅格特征的距离（损失）
    ///
    /// \param x 点的x坐标
    /// \param y 点的y坐标
    /// \param z 点的z坐标
    /// \param bel_scale 缩放尺度
    /// \return 距离损失
    template<class T>
    inline T getdistance(T x,T y,T z,double& bel_scale)
    {
      if(type==Type::KVoid||type==Type::KInit)
        return T(1.);
      else
        {
		Eigen::Matrix<T,3,1> vec;
		vec[0]=x;
		vec[1]=y;
		vec[2]=z;
		vec = vec - covresult.average.cast<T>();
		T dis = (vec.transpose()*covresult.covmat_inv.cast<T>()*vec);
		bel_scale = covresult.bel_scale;
		return ceres::sqrt(dis);
        }
    }

    Eigen::Vector3d getdistancesquare(double x,double y,double z,double& bel_scale,Eigen::Matrix<double,3,3>& information);

    double getdistancesquare(double x,double y,double z,double& bel_scale,Eigen::Matrix<double,1,3>& derivative);

    template<class T>
    Eigen::Matrix<T,3,1> getdistancematrix(T x,T y,T z,double& bel_scale)
    {

      if(type==Type::KVoid||type==Type::KInit)
      {
    	  Eigen::Matrix<T,3,1> dis;
    	  dis << T(1.),T(1.),T(1.);
          return dis;
      }
      else
        {
		Eigen::Matrix<T,3,1> vec;
		vec[0]=x;
		vec[1]=y;
		vec[2]=z;
		vec = vec - covresult.average.cast<T>();
		bel_scale = covresult.bel_scale;

		return (covresult.D.asDiagonal() * covresult.V.transpose()).cast<T>()*vec;
        }
    }

    bool finishupdate();

    friend inline std::ostream & operator << (std::ostream & os, FeatureElement &rhs){
        os << rhs.probability<<" "<<" "<<static_cast<int>(rhs.type) <<" "
            <<rhs.pointvarieties <<" "<<rhs.covresult.average[0]<<" "
	    <<rhs.covresult.average[1]<<" "<<rhs.covresult.average[2]<<" "
	    <<rhs.covresult.covmat_inv(0,0)<<" "<<rhs.covresult.covmat_inv(0,1)<<" "
	    <<rhs.covresult.covmat_inv(0,2)<<" "<<rhs.covresult.covmat_inv(1,0)<<" "
	    <<rhs.covresult.covmat_inv(1,1)<<" "<<rhs.covresult.covmat_inv(1,2)<<" "
	    <<rhs.covresult.covmat_inv(2,0)<<" "<<rhs.covresult.covmat_inv(2,1)<<" "
	    <<rhs.covresult.covmat_inv(2,2)<<" "<<rhs.covresult.bel_scale
            <<  std::endl;
        return os;
    }

    friend inline std::istream & operator >> (std::istream & is, FeatureElement &rhs){
      is >> rhs.probability>>*(reinterpret_cast<int*>(&rhs.type))
          >>rhs.pointvarieties >>rhs.covresult.average[0]
	    >>rhs.covresult.average[1]>>rhs.covresult.average[2]
	    >>rhs.covresult.covmat_inv(0,0)>>rhs.covresult.covmat_inv(0,1)
	    >>rhs.covresult.covmat_inv(0,2)>>rhs.covresult.covmat_inv(1,0)
	    >>rhs.covresult.covmat_inv(1,1)>>rhs.covresult.covmat_inv(1,2)
	    >>rhs.covresult.covmat_inv(2,0)>>rhs.covresult.covmat_inv(2,1)
	    >>rhs.covresult.covmat_inv(2,2)>>rhs.covresult.bel_scale;

        return is;
    }

    Type type;//!< 特征类型
    CovariancePara covpara;//!<协方差参数

    int pointvarieties;//!<点的种类数目
    int lastlasernum;//!<上一帧雷达线编号
    CovarianceResult covresult;//!<特征结果
    Eigen::Array3i index;//!<栅格索引
    float resolution;//!<栅格分辨率

    uint16 probability;//!<栅格概率
    bool updated;//!<更新标志
    bool confirmed;//!<特征固化标志
    bool missed;//!<概率向下更新标志
  };

  inline bool operator==(const FeatureElement& lhs,const FeatureElement& rhs)
  {
    return lhs.type==rhs.type;
  }
// Converts an 'index' with each dimension from 0 to 2^'bits' - 1 to a flat
// z-major index.
inline int ToFlatIndex(const Eigen::Array3i& index, const int bits) {
  DCHECK((index >= 0).all() && (index < (1 << bits)).all()) << index;
  return (((index.z() << bits) + index.y()) << bits) + index.x();
}

// Converts a flat z-major 'index' to a 3-dimensional index with each dimension
// from 0 to 2^'bits' - 1.
inline Eigen::Array3i To3DIndex(const int index, const int bits) {
  DCHECK_LT(index, 1 << (3 * bits));
  const int mask = (1 << bits) - 1;
  return Eigen::Array3i(index & mask, (index >> bits) & mask,
                        (index >> bits) >> bits);
}

// A function to compare value to the default value. (Allows specializations).
template <typename TValueType>
bool IsDefaultValue(const TValueType& v) {
  return v == TValueType();
}

// Specialization to compare a std::vector to the default value.
template <typename TElementType>
bool IsDefaultValue(const std::vector<TElementType>& v) {
  return v.empty();
}

// A flat grid of '2^kBits' x '2^kBits' x '2^kBits' voxels storing values of
// type 'ValueType' in contiguous memory. Indices in each dimension are 0-based.
template <typename TValueType, int kBits>
class FlatGrid {
 public:
  using ValueType = TValueType;

  // Creates a new flat grid with all values being default constructed.
  FlatGrid() {
    for (ValueType& value : cells_) {
      value = ValueType();
    }
  }

  FlatGrid(const FlatGrid&) = delete;
  FlatGrid& operator=(const FlatGrid&) = delete;

  // Returns the number of voxels per dimension.
  static int grid_size() { return 1 << kBits; }

  // Returns the value stored at 'index', each dimension of 'index' being
  // between 0 and grid_size() - 1.
  ValueType value(const Eigen::Array3i& index) const {
    return cells_[ToFlatIndex(index, kBits)];
  }

  // Returns a pointer to a value to allow changing it.
  ValueType* mutable_value(const Eigen::Array3i& index) {
    return &cells_[ToFlatIndex(index, kBits)];
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  class Iterator {
   public:
    Iterator() : current_(nullptr), end_(nullptr) {}

    explicit Iterator(const FlatGrid& flat_grid)
        : current_(flat_grid.cells_.data()),
          end_(flat_grid.cells_.data() + flat_grid.cells_.size()) {
      while (!Done() && IsDefaultValue(*current_)) {
        ++current_;
      }
    }

    void Next() {
      DCHECK(!Done());
      do {
        ++current_;
      } while (!Done() && IsDefaultValue(*current_));
    }

    bool Done() const { return current_ == end_; }

    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int index = (1 << (3 * kBits)) - (end_ - current_);
      return To3DIndex(index, kBits);
    }

    const ValueType& GetValue() const {
      DCHECK(!Done());
      return *current_;
    }

   private:
    const ValueType* current_;
    const ValueType* end_;
  };

 private:
  std::array<ValueType, 1 << (3 * kBits)> cells_;
};

// A feature flat grid of '2^kBits' x '2^kBits' x '2^kBits' voxels storing values of
// type 'ValueType' in contiguous memory. Indices in each dimension are 0-based.
template <typename TValueType, int kBits>
class FeatureFlatGrid {
 public:
  using ValueType = TValueType;

  // Creates a new flat grid with all values being default constructed.
  FeatureFlatGrid() {
    for (ValueType& value : cells_) {
      value = ValueType();
    }
  }

  FeatureFlatGrid(const FeatureFlatGrid&) = delete;
  FeatureFlatGrid& operator=(const FeatureFlatGrid&) = delete;

  // Returns the number of voxels per dimension.
  static int grid_size() { return 1 << kBits; }

  // Returns the value stored at 'index', each dimension of 'index' being
  // between 0 and grid_size() - 1.
  ValueType value(const Eigen::Array3i& index) const {
    return cells_[ToFlatIndex(index, kBits)];
  }

  // Returns a pointer to a value to allow changing it.
  ValueType* mutable_value(const Eigen::Array3i& index) {
    return &cells_[ToFlatIndex(index, kBits)];
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  class Iterator {
   public:
    Iterator() : current_(nullptr), end_(nullptr) {}

    explicit Iterator(const FeatureFlatGrid& flat_grid)
        : current_(flat_grid.cells_.data()),
          end_(flat_grid.cells_.data() + flat_grid.cells_.size()) {
      while (!Done() && IsDefaultValue(*current_)) {
        ++current_;
      }
    }

    void Next() {
      DCHECK(!Done());
      do {
        ++current_;
      } while (!Done() && IsDefaultValue(*current_));
    }

    bool Done() const { return current_ == end_; }

    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int index = (1 << (3 * kBits)) - (end_ - current_);
      return To3DIndex(index, kBits);
    }

    const ValueType& GetValue() const {
      DCHECK(!Done());
      return *current_;
    }

   private:
    const ValueType* current_;
    const ValueType* end_;
  };

 private:
  std::array<ValueType, 1 << (3 * kBits)> cells_;
};

// A grid consisting of '2^kBits' x '2^kBits' x '2^kBits' grids of type
// 'WrappedGrid'. Wrapped grids are constructed on first access via
// 'mutable_value()'.
template <typename WrappedGrid, int kBits>
class NestedGrid {
 public:
  using ValueType = typename WrappedGrid::ValueType;

  // Returns the number of voxels per dimension.
  static int grid_size() { return WrappedGrid::grid_size() << kBits; }

  // Returns the value stored at 'index', each dimension of 'index' being
  // between 0 and grid_size() - 1.
  ValueType value(const Eigen::Array3i& index) const {
    const Eigen::Array3i meta_index = GetMetaIndex(index);
    const WrappedGrid* const meta_cell =
        meta_cells_[ToFlatIndex(meta_index, kBits)].get();
    if (meta_cell == nullptr) {
      return ValueType();
    }
    const Eigen::Array3i inner_index =
        index - meta_index * WrappedGrid::grid_size();
    return meta_cell->value(inner_index);
  }

  // Returns a pointer to the value at 'index' to allow changing it. If
  // necessary a new wrapped grid is constructed to contain that value.
  ValueType* mutable_value(const Eigen::Array3i& index) {
    const Eigen::Array3i meta_index = GetMetaIndex(index);
    std::unique_ptr<WrappedGrid>& meta_cell =
        meta_cells_[ToFlatIndex(meta_index, kBits)];
    if (meta_cell == nullptr) {
      meta_cell = ::ivcommon::make_unique<WrappedGrid>();
    }
    const Eigen::Array3i inner_index =
        index - meta_index * WrappedGrid::grid_size();
    return meta_cell->mutable_value(inner_index);
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  class Iterator {
   public:
    Iterator() : current_(nullptr), end_(nullptr), nested_iterator_() {}

    explicit Iterator(const NestedGrid& nested_grid)
        : current_(nested_grid.meta_cells_.data()),
          end_(nested_grid.meta_cells_.data() + nested_grid.meta_cells_.size()),
          nested_iterator_() {
      AdvanceToValidNestedIterator();
    }

    void Next() {
      DCHECK(!Done());
      nested_iterator_.Next();
      if (!nested_iterator_.Done()) {
        return;
      }
      ++current_;
      AdvanceToValidNestedIterator();
    }

    bool Done() const { return current_ == end_; }

    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int index = (1 << (3 * kBits)) - (end_ - current_);
      return To3DIndex(index, kBits) * WrappedGrid::grid_size() +
             nested_iterator_.GetCellIndex();
    }

    const ValueType& GetValue() const {
      DCHECK(!Done());
      return nested_iterator_.GetValue();
    }

   private:
    void AdvanceToValidNestedIterator() {
      for (; !Done(); ++current_) {
        if (*current_ != nullptr) {
          nested_iterator_ = typename WrappedGrid::Iterator(**current_);
          if (!nested_iterator_.Done()) {
            break;
          }
        }
      }
    }

    const std::unique_ptr<WrappedGrid>* current_;
    const std::unique_ptr<WrappedGrid>* end_;
    typename WrappedGrid::Iterator nested_iterator_;
  };

 private:
  // Returns the Eigen::Array3i (meta) index of the meta cell containing
  // 'index'.
  Eigen::Array3i GetMetaIndex(const Eigen::Array3i& index) const {
    DCHECK((index >= 0).all()) << index;
    const Eigen::Array3i meta_index = index / WrappedGrid::grid_size();
    DCHECK((meta_index < (1 << kBits)).all()) << index;
    return meta_index;
  }

  std::array<std::unique_ptr<WrappedGrid>, 1 << (3 * kBits)> meta_cells_;
};

// A grid consisting of 2x2x2 grids of type 'WrappedGrid' initially. Wrapped
// grids are constructed on first access via 'mutable_value()'. If necessary,
// the grid grows to twice the size in each dimension. The range of indices is
// (almost) symmetric around the origin, i.e. negative indices are allowed.
template <typename WrappedGrid>
class DynamicGrid {
 public:
  using ValueType = typename WrappedGrid::ValueType;
  class CopiableMutex{
  public:
	  CopiableMutex(){}
	  CopiableMutex(CopiableMutex&&){}
	  std::recursive_mutex mutex;
  };
  DynamicGrid() : bits_(1), meta_cells_(8){}
  DynamicGrid(DynamicGrid&&) = default;
  DynamicGrid& operator=(DynamicGrid&&) = default;

  // Returns the current number of voxels per dimension.
  int grid_size() const { return WrappedGrid::grid_size() << bits_; }

  // Returns the value stored at 'index'.
  ValueType value(const Eigen::Array3i& index) const {
    const Eigen::Array3i shifted_index = index + (grid_size() >> 1);
    // The cast to unsigned is for performance to check with 3 comparisons
    // shifted_index.[xyz] >= 0 and shifted_index.[xyz] < grid_size.
    if ((shifted_index.cast<unsigned int>() >= grid_size()).any()) {
      return ValueType();
    }
    const Eigen::Array3i meta_index = GetMetaIndex(shifted_index);
    const WrappedGrid* const meta_cell =
        meta_cells_[ToFlatIndex(meta_index, bits_)].get();
    if (meta_cell == nullptr) {
      return ValueType();
    }
    const Eigen::Array3i inner_index =
        shifted_index - meta_index * WrappedGrid::grid_size();
    return meta_cell->value(inner_index);
  }

  // Returns a pointer to the value at 'index' to allow changing it, dynamically
  // growing the DynamicGrid and constructing new WrappedGrids as needed.
  ValueType* mutable_value(const Eigen::Array3i& index) {
    const Eigen::Array3i shifted_index = index + (grid_size() >> 1);
    // The cast to unsigned is for performance to check with 3 comparisons
    // shifted_index.[xyz] >= 0 and shifted_index.[xyz] < grid_size.
    if ((shifted_index.cast<unsigned int>() >= grid_size()).any()) {
      Grow();
      return mutable_value(index);
    }
    const Eigen::Array3i meta_index = GetMetaIndex(shifted_index);
    std::unique_ptr<WrappedGrid>& meta_cell =
        meta_cells_[ToFlatIndex(meta_index, bits_)];
    if (meta_cell == nullptr) {
      meta_cell = ::ivcommon::make_unique<WrappedGrid>();
    }
    const Eigen::Array3i inner_index =
        shifted_index - meta_index * WrappedGrid::grid_size();
    return meta_cell->mutable_value(inner_index);
  }


  void LockMutex()
  {
	  mutex_.mutex.lock();
  }

  void UnlockMutex()
  {
	  mutex_.mutex.unlock();
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  class Iterator {
   public:
    explicit Iterator(const DynamicGrid& dynamic_grid)
        : dynamic_grid_(dynamic_grid),
		  bits_(dynamic_grid.bits_),
          current_(dynamic_grid.meta_cells_.data()),
          end_(dynamic_grid.meta_cells_.data() +
               dynamic_grid.meta_cells_.size()),
          nested_iterator_() ,
		  mutex_(dynamic_grid.mutex_.mutex){
      AdvanceToValidNestedIterator();
    }

    void Next() {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if(dynamic_grid_.bits_!=bits_)
    	  return;
      DCHECK(!Done());
      nested_iterator_.Next();
      if (!nested_iterator_.Done()) {
        return;
      }
      ++current_;
      AdvanceToValidNestedIterator();
    }

    bool Done() const { return dynamic_grid_.bits_!=bits_||current_ == end_; }

    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int outer_index = (1 << (3 * bits_)) - (end_ - current_);
      const Eigen::Array3i shifted_index =
          To3DIndex(outer_index, bits_) * WrappedGrid::grid_size() +
          nested_iterator_.GetCellIndex();
      return shifted_index - ((1 << (bits_ - 1)) * WrappedGrid::grid_size());
    }

    const ValueType& GetValue() const {
      DCHECK(!Done());
      return nested_iterator_.GetValue();
    }

    void AdvanceToEnd() { current_ = end_; }

    const std::pair<Eigen::Array3i, ValueType> operator*() const {
      return std::pair<Eigen::Array3i, ValueType>(GetCellIndex(), GetValue());
    }

    Iterator& operator++() {
      Next();
      return *this;
    }

    bool operator!=(const Iterator& it) const {
      return it.current_ != current_;
    }

   private:
    void AdvanceToValidNestedIterator() {
      for (; !Done(); ++current_) {
        if (*current_ != nullptr) {
          nested_iterator_ = typename WrappedGrid::Iterator(**current_);
          if (!nested_iterator_.Done()) {
            break;
          }
        }
      }
    }

    int bits_;
    const std::unique_ptr<WrappedGrid>* current_;
    const std::unique_ptr<WrappedGrid>* const end_;
    typename WrappedGrid::Iterator nested_iterator_;
    std::recursive_mutex& mutex_;
    const DynamicGrid& dynamic_grid_;

  };

 private:
  // Returns the Eigen::Array3i (meta) index of the meta cell containing
  // 'index'.
  Eigen::Array3i GetMetaIndex(const Eigen::Array3i& index) const {
    DCHECK((index >= 0).all()) << index;
    const Eigen::Array3i meta_index = index / WrappedGrid::grid_size();
    DCHECK((meta_index < (1 << bits_)).all()) << index;
    return meta_index;
  }

  // Grows this grid by a factor of 2 in each of the 3 dimensions.
  void Grow() {
	std::lock_guard<std::recursive_mutex> lock(mutex_.mutex);
    const int new_bits = bits_ + 1;
//    CHECK_LE(new_bits, 8);
    std::vector<std::unique_ptr<WrappedGrid>> new_meta_cells_(
        8 * meta_cells_.size());
    for (int z = 0; z != (1 << bits_); ++z) {
      for (int y = 0; y != (1 << bits_); ++y) {
        for (int x = 0; x != (1 << bits_); ++x) {
          const Eigen::Array3i original_meta_index(x, y, z);
          const Eigen::Array3i new_meta_index =
              original_meta_index + (1 << (bits_ - 1));  //保持中心点不变，向四周扩展
          new_meta_cells_[ToFlatIndex(new_meta_index, new_bits)] =
              std::move(meta_cells_[ToFlatIndex(original_meta_index, bits_)]);
        }
      }
    }
    meta_cells_ = std::move(new_meta_cells_);
    bits_ = new_bits;
  }

  int bits_;
  std::vector<std::unique_ptr<WrappedGrid>> meta_cells_;
  mutable CopiableMutex mutex_;
};

template <typename ValueType>
using Grid = DynamicGrid<NestedGrid<FlatGrid<ValueType, 3>, 3>>;

// Represents a 3D grid as a wide, shallow tree.
template <typename ValueType>
class HybridGridBase : public Grid<ValueType> {
 public:
  using Iterator = typename Grid<ValueType>::Iterator;

  // Creates a new tree-based probability grid with voxels having edge length
  // 'resolution' around the origin which becomes the center of the cell at
  // index (0, 0, 0).
  explicit HybridGridBase(const float resolution) : resolution_(resolution) {}

  inline float resolution() const { return resolution_; }

  // Returns the index of the cell containing the 'point'. Indices are integer
  // vectors identifying cells, for this the coordinates are rounded to the next
  // multiple of the resolution.
  Eigen::Array3i GetCellIndex(const Eigen::Vector3d& point) const {
    Eigen::Array3d index = point.array() / resolution_;
    return Eigen::Array3i(::ivcommon::RoundToInt(index.x()),
                          ::ivcommon::RoundToInt(index.y()),
                          ::ivcommon::RoundToInt(index.z()));
  }

  // Returns one of the octants, (0, 0, 0), (1, 0, 0), ..., (1, 1, 1).
  static Eigen::Array3i GetOctant(const int i) {
    DCHECK_GE(i, 0);
    DCHECK_LT(i, 8);
    return Eigen::Array3i(static_cast<bool>(i & 1), static_cast<bool>(i & 2),
                          static_cast<bool>(i & 4));
  }

  // Returns the center of the cell at 'index'.
  Eigen::Vector3d GetCenterOfCell(const Eigen::Array3i& index) const {
    return index.matrix().cast<double>() * resolution_;
  }

  // Iterator functions for range-based for loops.
  Iterator begin() const { return Iterator(*this); }

  Iterator end() const {
    Iterator it(*this);
    it.AdvanceToEnd();
    return it;
  }

 private:
  // Edge length of each voxel.
  const float resolution_;
};

// A grid containing probability values stored using 15 bits, and an update
// marker per voxel.
class HybridGrid : public HybridGridBase<uint16> {
 public:
  explicit HybridGrid(const float resolution)
      : HybridGridBase<uint16>(resolution) {}

  explicit HybridGrid(const proto::HybridGrid& proto)
      : HybridGrid(proto.resolution()) {
    CHECK_EQ(proto.values_size(), proto.x_indices_size());
    CHECK_EQ(proto.values_size(), proto.y_indices_size());
    CHECK_EQ(proto.values_size(), proto.z_indices_size());
    for (int i = 0; i < proto.values_size(); ++i) {
      // SetProbability does some error checking for us.
      SetProbability(Eigen::Vector3i(proto.x_indices(i), proto.y_indices(i),
                                     proto.z_indices(i)),
                     mapping::ValueToProbability(proto.values(i)));
    }
  }

  // Sets the probability of the cell at 'index' to the given 'probability'.
  void SetProbability(const Eigen::Array3i& index, const float probability) {
    *mutable_value(index) = mapping::ProbabilityToValue(probability);
  }

  // Finishes the update sequence.
  void FinishUpdate() {
    while (!update_indices_.empty()) {
      DCHECK_GE(*update_indices_.back(), mapping::kUpdateMarker);
      *update_indices_.back() -= mapping::kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'index' if the cell has not already been
  // updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(const Eigen::Array3i& index,
                        const std::vector<uint16>& table) {
    DCHECK_EQ(table.size(), mapping::kUpdateMarker);
    uint16* const cell = mutable_value(index);
    if (*cell >= mapping::kUpdateMarker) {
      return false;
    }
    update_indices_.push_back(cell);
    *cell = table[*cell];
    DCHECK_GE(*cell, mapping::kUpdateMarker);
    return true;
  }

  // Returns the probability of the cell with 'index'.
  float GetProbability(const Eigen::Array3i& index) const {
    return mapping::ValueToProbability(value(index));
  }

  // Returns true if the probability at the specified 'index' is known.
  bool IsKnown(const Eigen::Array3i& index) const { return value(index) != 0; }


  void save(const char* filename) const
  {
    std::fstream file(filename,std::ios::out);
    file<<this->resolution()<<" "<<this->grid_size()<<std::endl;
    for(auto cell:*this)
      {
	float probability = mapping::ValueToProbability(cell.second);
	if(probability > 0.501f)
	  {
	    file<<cell.first[0]<<" "<<cell.first[1]<<" "<<cell.first[2]<<" "<<probability<<std::endl;
	  }
      }
  }

  void load(const char* filename)
  {
    std::ifstream file(filename);
    int gridsize;
    float resolution;
    file>>resolution>>gridsize;
    DCHECK(resolution == this->resolution());
    Eigen::Array3i index;
    while(file>>index[0])
      {
	float probability;
	file>>index[1]>>index[2]>>probability;
	auto* cell = mutable_value(index);
	*cell = mapping::ProbabilityToValue(probability);
      }
  }

  proto::HybridGrid ToProto() const {
    CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                      "not supported. Finish the update first.";
    proto::HybridGrid result;
    result.set_resolution(resolution());
    for (const auto it : *this) {
      float probability = mapping::ValueToProbability(it.second);
      if(probability<0.501)
         continue;
      result.add_x_indices(it.first.x());
      result.add_y_indices(it.first.y());
      result.add_z_indices(it.first.z());
      result.add_values(it.second);
    }
    return result;
  }

 private:
  // Markers at changed cells.
  std::vector<ValueType*> update_indices_;
};


// A grid containing probability values stored using 15 bits, and an update
// marker per voxel.

struct IntensityElement{
	IntensityElement():
		average(0),
		sum(0),
		num(0),
		updated(false){}
	void push_back(sensor::Point point)
	{
		sum += point.intensity;
		num ++;
		average = sum/num;
	}
	float average;
	float sum;
	int num;
	bool updated;
};
inline bool operator==(const IntensityElement& lhs,const IntensityElement& rhs)
{
  return lhs.average==rhs.average;
}

class IntensityHybridGrid : public HybridGridBase<IntensityElement> {
 public:
  explicit IntensityHybridGrid(const float resolution)
      : HybridGridBase<IntensityElement>(resolution) {}

  explicit IntensityHybridGrid(const proto::IntensityHybridGrid& proto)
      : HybridGridBase<IntensityElement>(proto.resolution()) {
    CHECK_EQ(proto.values_size(), proto.x_indices_size());
    CHECK_EQ(proto.values_size(), proto.y_indices_size());
    CHECK_EQ(proto.values_size(), proto.z_indices_size());
    for (int i = 0; i < proto.values_size(); ++i) {
      // SetProbability does some error checking for us.
    	SetValue(Eigen::Vector3i(proto.x_indices(i), proto.y_indices(i),
                                     proto.z_indices(i)),proto.values(i));
    }
  }

  // Sets the average of the cell at 'index' to the given 'probability'.
  void SetValue(const Eigen::Array3i& index, const float value) {
    mutable_value(index)->average = value;
  }

  // Finishes the update sequence.
  void FinishUpdate() {
    while (!update_indices_.empty()) {

      update_indices_.pop_back();
    }
  }

  bool addPoint(const Eigen::Array3i& index,
		const sensor::Point& point)
  {
  	LockMutex();
	auto* const cell = mutable_value(index);
	UnlockMutex();

	cell->push_back(point);
	if (cell->updated) {
	return false;
	}
	cell->updated = true;

	update_indices_.push_back(cell);
	return true;
  }

  // Returns the probability of the cell with 'index'.
  float GetAverage(const Eigen::Array3i& index) const {
    return value(index).average;
  }
  // Returns the center of the cell at 'index'.
  Eigen::Vector3d GetCenterOfCell(const Eigen::Array3i& index) const {
    return index.matrix().cast<double>() * resolution();
  }
  // Returns true if the probability at the specified 'index' is known.
  bool IsKnown(const Eigen::Array3i& index) const { return value(index).average >= 0.001; }


  proto::IntensityHybridGrid ToProto() const {
    CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                      "not supported. Finish the update first.";
    proto::IntensityHybridGrid result;
    result.set_resolution(resolution());
    for (const auto it : *this) {
      float average = (it.second).average;
      if(average<0.001)
    	  continue;
      result.add_x_indices(it.first.x());
      result.add_y_indices(it.first.y());
      result.add_z_indices(it.first.z());
      result.add_values(it.second.average);
    }
    return result;
  }

 private:
  // Markers at changed cells.
  std::vector<ValueType*> update_indices_;
};
// A grid containing probability values stored using 15 bits, and an update
// marker per voxel.

template <typename ValueType>
using FeatureGrid = DynamicGrid<NestedGrid<FlatGrid<ValueType, 2>, 3>>;

// Represents a 3D grid as a wide, shallow tree.
template <typename ValueType>
class FeatureHybridGridBase : public FeatureGrid<ValueType> {
 public:
  using Iterator = typename FeatureGrid<ValueType>::Iterator;

  // Creates a new tree-based probability grid with voxels having edge length
  // 'resolution' around the origin which becomes the center of the cell at
  // index (0, 0, 0).
  explicit FeatureHybridGridBase(const float resolution) : resolution_(resolution) {}

  inline float resolution() const { return resolution_; }

  // Returns the index of the cell containing the 'point'. Indices are integer
  // vectors identifying cells, for this the coordinates are rounded to the next
  // multiple of the resolution.
  Eigen::Array3i GetCellIndex(const Eigen::Vector3d& point) const {
    Eigen::Array3d index = point.array() / resolution_;
    return Eigen::Array3i(::ivcommon::RoundToInt(index.x()),
                          ::ivcommon::RoundToInt(index.y()),
                          ::ivcommon::RoundToInt(index.z()));
  }

  // Returns one of the octants, (0, 0, 0), (1, 0, 0), ..., (1, 1, 1).
  static Eigen::Array3i GetOctant(const int i) {
    DCHECK_GE(i, 0);
    DCHECK_LT(i, 8);
    return Eigen::Array3i(static_cast<bool>(i & 1), static_cast<bool>(i & 2),
                          static_cast<bool>(i & 4));
  }

  // Returns the center of the cell at 'index'.
  Eigen::Vector3d GetCenterOfCell(const Eigen::Array3i& index) const {
    return index.matrix().cast<double>() * resolution_;
  }

  // Iterator functions for range-based for loops.
  Iterator begin() const { return Iterator(*this); }

  Iterator end() const {
    Iterator it(*this);
    it.AdvanceToEnd();
    return it;
  }

 private:
  // Edge length of each voxel.
  const float resolution_;

};

/// \brief 特征地图类
///
/// 进行特征地图更新、搜索、保存、加载等
class FeatureHybridGrid : public FeatureHybridGridBase<FeatureElement> {
 public:
  explicit FeatureHybridGrid(const float resolution)
      : FeatureHybridGridBase<FeatureElement>(resolution) {}


  explicit FeatureHybridGrid(const proto::FeatureHybridGrid& proto);

  // Finishes the update sequence.
  void FinishUpdate() ;

  bool addPoint(const Eigen::Array3i& index,
		const sensor::Point& point,const std::vector<uint16>& table) ;

  // Returns the probability of the cell with 'index'.
  inline float GetProbability(const Eigen::Array3i& index) const {
    return mapping::ValueToProbability(value(index).probability);
  }

  bool ApplyLookupTable(const Eigen::Array3i& index,
                        const std::vector<uint16>& table) ;

  int get_confirmed_voxel_number() const
  {
      return confirmed_voxel_num_;
  }

  // Returns the probability of the cell with 'index'.
  //不能这样返回引用，临时变量
//  const FeatureElement& GetValue(const Eigen::Array3i& index) const {
//    return value(index);
//  }

  void save(const char* filename) const;

  void load(const char* filename);
  void clearindices();
  proto::FeatureHybridGrid ToProto() const ;
  std::vector<Eigen::Vector3d> addpoint_indices_;
  std::vector<Eigen::Vector3d> missgrid_indices_;
 private:
  // Markers at changed cells.
  std::vector<ValueType*> update_indices_;
  int confirmed_voxel_num_ = 0;


};
}  // namespace mapping_3d

#endif  // CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_
