/*!
* \file hybrid_grid.cc
* \brief 特征栅格
*
* 特征栅格地图类，进行特征地图的更新，损失函数的计算等
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/

#include "covgrid_slam/mapping3d/hybrid_grid.h"



namespace mapping3d {

    void FeatureElement::clear()
    {
      type=Type::KVoid;
      memset(&covpara,0,sizeof(covpara));

      updated = false;
      pointvarieties = 0;
      confirmed = false;
    }

    /// \brief 插入点,更新协方差
    ///
    /// \param point 点
    void FeatureElement::push_back(const sensor::Point& point)
    {
      if(confirmed)
        return;
////      points.push_back(point);
//      int ind_x = (point[0]/resolution -index[0] + 0.5*resolution)*sectionwidth;
//      int ind_y = (point[1]/resolution -index[1] + 0.5*resolution)*sectionwidth;
//      int ind_z = (point[2]/resolution -index[2] + 0.5*resolution)*sectionwidth;
//      if(ind_x>=0 && ind_x<sectionwidth && ind_y>=0 && ind_y<sectionwidth && ind_z>=0 && ind_z<sectionwidth)
//      {
//    	  int ind = ind_z*sectionwidth*sectionwidth + ind_y*sectionwidth +ind_x;
//          subelement[ind].num++;
//          subelement[ind].sum_x += point[0];
//          subelement[ind].sum_y += point[1];
//          subelement[ind].sum_z += point[2];
//          subelement[ind].sum_x_x += point[0] * point[0];
//          subelement[ind].sum_x_y += point[0] * point[1];
//          subelement[ind].sum_x_z += point[0] * point[2];
//          subelement[ind].sum_y_y += point[1] * point[1];
//          subelement[ind].sum_y_z += point[1] * point[2];
//          subelement[ind].sum_z_z += point[2] * point[2];
//      }

      covpara.num++;
      covpara.sum_x += point[0];
      covpara.sum_y += point[1];
      covpara.sum_z += point[2];
      covpara.sum_x_x += point[0] * point[0];
      covpara.sum_x_y += point[0] * point[1];
      covpara.sum_x_z += point[0] * point[2];
      covpara.sum_y_y += point[1] * point[1];
      covpara.sum_y_z += point[1] * point[2];
      covpara.sum_z_z += point[2] * point[2];

      int lasernum = point.ringandtime;

      if(pointvarieties==0||lasernum!=lastlasernum)
	{
	  pointvarieties++;
	  lastlasernum = lasernum;
	}
    }

    Eigen::Vector3d FeatureElement::getdistancesquare(double x,double y,double z,double& bel_scale,Eigen::Matrix<double,3,3>& information)
    {
      if(type==Type::KVoid||type==Type::KInit)
        return Eigen::Vector3d(1,1,1);
      else
        {
		Eigen::Matrix<double,3,1> vec;
		vec[0]=x;
		vec[1]=y;
		vec[2]=z;
		vec = vec - covresult.average;
		information = (covresult.covmat_inv);
//		double dis = 0.5*(derivative*vec)(0,0);
		bel_scale = covresult.bel_scale;
//		if(dis<0)
//			LOG(WARNING)<<"dis="<<dis;
		return vec;
        }
    }

    /// \brief 计算点与栅格特征的距离（损失）
    ///
    /// \param x 点的x坐标
    /// \param y 点的y坐标
    /// \param z 点的z坐标
    /// \param bel_scale 缩放尺度
    /// \param derivative 损失函数的导数
    /// \return 距离损失
    double FeatureElement::getdistancesquare(double x,double y,double z,double& bel_scale,Eigen::Matrix<double,1,3>& derivative)
    {
      if(type==Type::KVoid||type==Type::KInit)
        return 1.;
      else
        {
		Eigen::Matrix<double,3,1> vec;
		vec[0]=x;
		vec[1]=y;
		vec[2]=z;
		vec = vec - covresult.average;
		derivative = vec.transpose()*(covresult.covmat_inv);
		double dis = (derivative*vec);
		bel_scale = covresult.bel_scale;
//		if(dis<0)
//			LOG(WARNING)<<"dis="<<dis;
		return std::sqrt(dis);
        }
    }


    /// \brief 当前帧点云插入完后，更新特征
    ///
    bool FeatureElement::finishupdate()
    {
      updated = false;

      if(missed)
      {
    	  missed=false;
    	  return false;
      }

      if(confirmed)
        return false;



      if(covpara.num >= 5&&pointvarieties>1)
        {
//			if(pointvarieties > KVarietiesThreshold || subelement.size()>5)
//			{
//				memset(&covpara,0,sizeof(covpara));
//			  for(auto it = subelement.begin();it!=subelement.end();it++)
//			  {
//				  if(it->second.num<2)
//					  continue;
//			      covpara.num += it->second.num;
//			      covpara.sum_x += it->second.sum_x;
//			      covpara.sum_y += it->second.sum_y;
//			      covpara.sum_z += it->second.sum_z;
//			      covpara.sum_x_x += it->second.sum_x_x;
//			      covpara.sum_x_y += it->second.sum_x_y;
//			      covpara.sum_x_z += it->second.sum_x_z;
//			      covpara.sum_y_y += it->second.sum_y_y;
//			      covpara.sum_y_z += it->second.sum_y_z;
//			      covpara.sum_z_z += it->second.sum_z_z;
//			  }
//			}
			covresult.average[0] = covpara.sum_x / covpara.num;
			covresult.average[1] = covpara.sum_y / covpara.num;
			covresult.average[2] = covpara.sum_z / covpara.num;
			covresult.covmat(0,0) = (covpara.sum_x_x - covresult.average[0] * covpara.sum_x)/(covpara.num-1);
			covresult.covmat(0,1) = (covpara.sum_x_y - covresult.average[0] * covpara.sum_y)/(covpara.num-1);
			covresult.covmat(0,2) = (covpara.sum_x_z - covresult.average[0] * covpara.sum_z)/(covpara.num-1);
            covresult.covmat(1,0) = covresult.covmat(0,1);
			covresult.covmat(1,1) = (covpara.sum_y_y - covresult.average[1] * covpara.sum_y)/(covpara.num-1);
			covresult.covmat(1,2) = (covpara.sum_y_z - covresult.average[1] * covpara.sum_z)/(covpara.num-1);
            covresult.covmat(2,0) = covresult.covmat(0,2);
            covresult.covmat(2,1) = covresult.covmat(1,2);
			covresult.covmat(2,2) = (covpara.sum_z_z - covresult.average[2] * covpara.sum_z)/(covpara.num-1);
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covresult.covmat);
			Eigen::Vector3d D = es.eigenvalues();
			Eigen::Matrix3d V = es.eigenvectors();


			if((D[0]<0)||(D[1]<0)||(D[2]<0))
			  {
				clear();
				return false;
			  }
			covresult.D = D;
			covresult.V = V;

			covresult.bel_scale = 1./ std::sqrt(std::sqrt(D[0]));

			if((D[2]<=10*D[1]&&D[1]>15*D[0]))
			  {
				D[1]=D[2]=0;
				D[0]=1.;
				covresult.covmat_inv = V * D.asDiagonal() * V.transpose();
				//covresult.covmat_inv = covresult.covmat.inverse();
				type = Type::KFlat;
			  }
			else if(D[2]>10*D[1])
			  {
				D[2]=0;
				D[1] = D[0]/D[1];
				D[0] = 1.;
				covresult.covmat_inv = V * D.asDiagonal() * V.transpose();
		  //	    covresult.covmat_inv = covresult.covmat.inverse()*D[0];

				type = Type::KLine;
			  }
			else
			  {
//				D[2]= pow(D[0]/D[2],0.25);
//				D[1] = pow(D[0]/D[1],0.25);
//				D[0] = 1.;

		//  	  LOG(INFO)<<"D="<<D;
//				covresult.covmat_inv = V * D.asDiagonal() * V.transpose();
				covresult.covmat_inv = covresult.covmat.inverse()*(D[0]) ;

				type = Type::KCluster;
			  }
		//  	covresult.D[0] = 1.;
		//  	covresult.D[1] = std::sqrt(D[1]);
		//  	covresult.D[2] = std::sqrt(D[2]);
		//  	covresult.V = V;
        }
      else
    	  type = Type::KInit;
      if(pointvarieties > KVarietiesThreshold)
      {
    	confirmed = true;
    	return true;
//    	subelement.clear();
      }
      return false;

    }

   FeatureHybridGrid::FeatureHybridGrid(const proto::FeatureHybridGrid& proto)
        : FeatureHybridGridBase<FeatureElement>(proto.resolution()) {
      confirmed_voxel_num_ = proto.confirmed_voxel_number();
      CHECK_EQ(proto.values_size(), proto.x_indices_size());
      CHECK_EQ(proto.values_size(), proto.y_indices_size());
      CHECK_EQ(proto.values_size(), proto.z_indices_size());
      for (int i = 0; i < proto.values_size(); ++i) {
        // SetProbability does some error checking for us.
      	Eigen::Vector3i index(proto.x_indices(i), proto.y_indices(i),
                  proto.z_indices(i));
      	FeatureElement* cell = mutable_value(index);
      	cell->probability = proto.values(i).probability();
      	cell->pointvarieties = proto.values(i).pointvarieties();
      	cell->covpara.num = proto.values(i).pointnum();
      	cell->covpara.sum_x = proto.values(i).sum_x();
      	cell->covpara.sum_y = proto.values(i).sum_y();
      	cell->covpara.sum_z = proto.values(i).sum_z();
      	cell->covpara.sum_x_x = proto.values(i).sum_x_x();
      	cell->covpara.sum_y_y = proto.values(i).sum_y_y();
      	cell->covpara.sum_z_z = proto.values(i).sum_z_z();
      	cell->covpara.sum_x_y = proto.values(i).sum_x_y();
      	cell->covpara.sum_y_z = proto.values(i).sum_y_z();
      	cell->covpara.sum_x_z = proto.values(i).sum_x_z();
      	cell->finishupdate();
      }
    }


   proto::FeatureHybridGrid FeatureHybridGrid::ToProto() const {
     CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                       "not supported. Finish the update first.";
     proto::FeatureHybridGrid result;
     result.set_resolution(resolution());
     result.set_confirmed_voxel_number(get_confirmed_voxel_number());
     for (const auto it : *this) {
 	  float probability = mapping::ValueToProbability(it.second.probability);
 	  if(probability<0.501)
 	    continue;
       result.add_x_indices(it.first.x());
       result.add_y_indices(it.first.y());
       result.add_z_indices(it.first.z());
       proto::FeatureElement* feature_element = result.add_values();
       feature_element->set_probability(it.second.probability);
       feature_element->set_pointvarieties(it.second.pointvarieties);
       feature_element->set_pointnum(it.second.covpara.num);
       feature_element->set_sum_x(it.second.covpara.sum_x);
       feature_element->set_sum_y(it.second.covpara.sum_y);
       feature_element->set_sum_z(it.second.covpara.sum_z);
       feature_element->set_sum_x_x(it.second.covpara.sum_x_x);
       feature_element->set_sum_y_y(it.second.covpara.sum_y_y);
       feature_element->set_sum_z_z(it.second.covpara.sum_z_z);
       feature_element->set_sum_x_y(it.second.covpara.sum_x_y);
       feature_element->set_sum_x_z(it.second.covpara.sum_x_z);
       feature_element->set_sum_y_z(it.second.covpara.sum_y_z);
     }
     return result;
   }
  // Finishes the update sequence.
  void FeatureHybridGrid::FinishUpdate() {
    while (!update_indices_.empty()) {
      CHECK(update_indices_.back()->updated);
      if(update_indices_.back()->finishupdate())
          confirmed_voxel_num_++;
      update_indices_.pop_back();
    }
  }
  /// \brief 插入点，更新栅格
  ///
  /// \param index 栅格索引
  /// \param point 点
  /// \param table 用于概率更新的表格
  /// \return 更新是否成功
  bool FeatureHybridGrid::addPoint(const Eigen::Array3i& index,
		const sensor::Point& point,const std::vector<uint16>& table) {
	LockMutex();
    FeatureElement* const cell = mutable_value(index);
    UnlockMutex();
    if(!cell->updated && cell->type!=FeatureElement::Type::KVoid)
    {
		cell->resolution = resolution();
		cell->index = index;
    }

    cell->push_back(point);
    if (cell->updated) {
      return false;
    }
    cell->updated = true;

    update_indices_.push_back(cell);
    addpoint_indices_.push_back(this->GetCenterOfCell(index));
    cell->probability = table[cell->probability] - mapping::kUpdateMarker;
//    LOG(INFO)<<mapping::ValueToProbability(cell->probability);
    return true;
  }


  /// \brief 将栅格放入更新列表，并更新其概率
  ///
  /// \param index 栅格索引
  /// \param table 概率更新表格
  /// \return 是否已经更新过
  bool FeatureHybridGrid::ApplyLookupTable(const Eigen::Array3i& index,
                        const std::vector<uint16>& table) {

    FeatureElement* const cell = mutable_value(index);
    if (cell->updated) {
      return false;
    }
    if(cell->type != FeatureElement::Type::KVoid)
      missgrid_indices_.push_back(this->GetCenterOfCell(index));
    cell->updated = true;
    cell->missed = true;
    update_indices_.push_back(cell);
    cell->probability = table[cell->probability] - mapping::kUpdateMarker;
//    if(mapping::ValueToProbability(cell->probability)<0.3)
//      *cell = FeatureElement();
    return true;
  }


  void FeatureHybridGrid::save(const char* filename) const
  {
    std::fstream file(filename,std::ios::out);
    file<<this->resolution()<<" "<<this->grid_size()<<std::endl;
    for(auto cell:*this)
      {
	if(cell.second.type!=FeatureElement::Type::KVoid&&cell.second.type!=FeatureElement::Type::KInit
	    && mapping::ValueToProbability(cell.second.probability)>0.5)
	  {
	  file<<cell.first[0]<<" "<<cell.first[1]<<" "<<cell.first[2]<<std::endl;
	  file<<cell.second;
	  }
      }
  }

  void FeatureHybridGrid::load(const char* filename)
  {
    std::ifstream file(filename);
    int gridsize;
    float resolution;
    file>>resolution>>gridsize;
    CHECK(resolution == this->resolution());
    Eigen::Array3i index;
    while(file>>index[0])
      {
	file>>index[1]>>index[2];
	FeatureElement* cell = mutable_value(index);
	file>>*cell;
      }
  }

  void FeatureHybridGrid::clearindices()
  {
    addpoint_indices_.clear();
    missgrid_indices_.clear();
  }



}  // namespace mapping_3d

