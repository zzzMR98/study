/*!
* \file submap_manager.cc
* \brief 子地图管理程序
*
* 进行子地图的管理、加载、保存，子地图切换，模式切换等
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/
#include "covgrid_slam/mapping3d/submap_manager.h"

#include <iostream>
#include <iomanip>
//#include <serial/serial.h>


namespace mapping3d{
/// \brief 构造函数
///
/// \param options 子地图相关参数
/// \param global_pose_init 初始化位姿
SubmapManager::SubmapManager(proto::SubmapsOptions& options,const mapping3d::PosewithGps& global_pose_init):
		ActiveSubmaps(options),
		options_(options),
		submap_io_thread_(boost::bind(&SubmapManager::SubmapThread,this))
{
	if(options_.readmap_flag())
	  {
		LoadSubmapList();
		set_global_pose({submaplists_.global_pose,submaplists_.gpsdata});
	  }
	else
	{
		set_global_pose(global_pose_init);
		submaplists_.global_pose = global_pose_init.pose;
		submaplists_.gpsdata = global_pose_init.gps;
	}

	if(options_.automode_flag())
	{
//		LoadSubmapList();
		options_.set_update_flag(true);
		options_.set_readmap_flag(false);
	}

	reset();
    if(!options_.readmap_flag())
    {
        AddNewSubmap(global_init_pose().pose.inverse()*global_pose_init.pose,global_pose_init.pose);
    }
}
/// \brief 析构函数
///
/// 进行子地图保存及线程退出

SubmapManager::~SubmapManager()
{
	LOG(INFO)<<"begin ~SubmapManager";
	FinishAndSaveSubmap();
	if(options_.savemap_flag())
		SaveSubmapList();
	LOG(INFO)<<"~SubmapManager end";
}

/// \brief 完成和保存子地图
///
/// 进行子地图保存及线程退出
void SubmapManager::FinishAndSaveSubmap()
{
	submap_process_queue_.Push(std::make_shared<SaveSubmapSignal>(SignalType::Return,submaps().front()));
	submap_io_thread_.join();
}

/// \brief 判断位姿是否在子地图中
///
/// \param pose 当前位姿
/// \param submap_index 子地图索引

bool SubmapManager::IsInTheSubmap(const ivcommon::transform::Rigid3d& pose,int submap_index)
{
	if(submaplists_.lists.find(submap_index)==submaplists_.lists.end())
		return false;
	constexpr float kGapThreshold = 10.;
//	constexpr float kBeginOrEndThreshold = 20.;
	const auto& trajectory = submaplists_.lists[submap_index].trajectory;
//	if((trajectory.front().estimatepose.inverse()*pose.translation()).head(2).norm() > kBeginOrEndThreshold
//			&&(trajectory.back().estimatepose.inverse()*pose.translation()).head(2).norm() > kBeginOrEndThreshold)
//		return false;
	for(const auto& node:trajectory)
	{
//		double diffwithgps = (node.gpspose.inverse()*pose.translation()).head(2).norm();
		double diffwithmap = (node.estimatepose.inverse()*pose.translation()).norm();
		if(diffwithmap<kGapThreshold)
			return true;
	}
	return false;
}
/// \brief 判断gps位姿是否在子地图中
///
/// \param pose 当前gps位姿
/// \param submap_index 子地图索引

bool SubmapManager::IsInTheSubmapForGps(const ivcommon::transform::Rigid3d& pose,int submap_index)
{
	if(submaplists_.lists.find(submap_index)==submaplists_.lists.end())
		return false;
	constexpr float kGapThreshold = 10.;
	constexpr float kMidThreshold = 20.;
	auto trajectory = submaplists_.lists[submap_index].trajectory;
	if(trajectory.size()==0)
	{
		LOG(ERROR)<<submap_index<<" the num of trajectory node is zero!";
		return false;
	}
	auto ref_node = trajectory.at(trajectory.size()/2);
	auto ref_pose = ref_node.gpspose;
	if(ref_node.withfusepose)
		ref_pose = ref_node.fusepose;
	if((ref_pose.inverse()*pose.translation()).head(2).norm() > kMidThreshold)
//			&&(trajectory.back().gpspose.inverse()*pose.translation()).head(2).norm() > kBeginOrEndThreshold)
		return false;
	for(auto node:trajectory)
	{
		auto ref_pose = node.gpspose;
		if(node.withfusepose)
			ref_pose = node.fusepose;
		double diffwithgps = (ref_pose.inverse()*pose.translation()).head(2).norm();

		if(diffwithgps<kGapThreshold)//||diffwithmap<kGapThreshold)
			return true;
	}
	return false;
}
/// \brief 子地图操作线程
///
/// 进行子地图的加载保存、退出
void SubmapManager::SubmapThread()
{
  while(1)
    {
        auto signalpair = submap_process_queue_.Pop();

        switch (signalpair->signaltype)
        {
			case SignalType::Read:
			{
				LoadSubmap(std::dynamic_pointer_cast<ReadSubmapSignal>(signalpair)->submapindex);
				break;
			}
			case SignalType::Save:
			{
			  CHECK(options_.savemap_flag());
			  SaveSubmap(std::dynamic_pointer_cast<SaveSubmapSignal>(signalpair)->submap);
			  break;
			}
			case SignalType::Return:
			{
	//		  if(options_.savemap_flag()&&options_.update_flag())
	//			SaveSubmap(std::dynamic_pointer_cast<SaveSubmapSignal>(signalpair)->submap);
			  LOG(INFO)<<"SubmapThread end";
			  return;
			}
        }

    }

}
/// \brief 弹出并保存子地图
///

void SubmapManager::PopAndSaveFinishedSubmap()
{
	if(submaps().at(0)->finished())
	{
	     if(options_.savemap_flag())
	    	  submap_process_queue_.Push(std::make_shared<SaveSubmapSignal>(SignalType::Save,submaps().front()));

		  PopFront();
	}
}
/// \brief 利用gps位姿更新最近邻子地图
///
/// \param pose gps 位姿
/// \return 最近邻子地图
int SubmapManager::UpdateNearestSubmapGps(const ivcommon::transform::Rigid3d& pose)
{
	int nearest_index = -1;
	double mindistance = 20.;
	for(auto& submapheader:submaplists_.lists)
	{
		if(submaps().front()->header().linkindexs.find(submapheader.first)!=submaps().front()->header().linkindexs.end())//防止匹配到刚刚保存的地图
			continue;
		auto trajectory = submapheader.second.trajectory;
		if(trajectory.size()==0)
		{
			LOG(ERROR)<<submapheader.first<<" trajectory node is zero!";
			continue;
		}
		auto node = trajectory.at(trajectory.size()/2);
		auto ref_pose = node.gpspose;
		if(node.withfusepose)
			ref_pose = node.fusepose;
		double distance = (ref_pose.inverse()*pose.translation()).head(2).norm();
		if(distance < mindistance)
		{
			mindistance = distance;
			nearest_index = submapheader.first;
		}
	}
	if(nearest_index>0)
	{
		LOG(WARNING)<<"gps_nearest_index:"<<nearest_index<<" distance:"<<mindistance;
//		AddSubmap(submaplists_.lists[nearest_index],options_.readmap_dir());
	}

	return nearest_index;
}
/// \brief 利用离线匹配位姿更新最近邻子地图
///
/// \param pose 离线匹配位姿

void SubmapManager::UpdateNearestSubmap(const ivcommon::transform::Rigid3d& pose)
{
	nearest_index_ = -1;

	double mindistance = 20.;
	for(const auto& submapheader:submaplists_.lists)
	{
		const auto& trajectory = submapheader.second.trajectory;
		if(trajectory.size()==0)
		{
			LOG(ERROR)<<submapheader.first<<" trajectory node is zero!";
			continue;
		}
		double distance = (trajectory.at(trajectory.size()/2).estimatepose.inverse()*pose.translation()).norm();
		LOG_IF(WARNING,distance < mindistance)<<"index:"<<submapheader.first<<" distance:"<<distance;
		if(distance < mindistance && IsInTheSubmap(pose, submapheader.first))
		{
			mindistance = distance;
			nearest_index_ = submapheader.first;

		}
	}
	if(nearest_index_>0)
	{
		LOG(WARNING)<<"nearest_index:"<<nearest_index_<<" distance:"<<mindistance;
	}

	return ;
}
/// \brief 请求加载子地图到缓存
///
/// \param index 子地图索引
/// \return 子地图列表中无该索引时返回false
bool SubmapManager::RequestSubmap(int index)
{
	if(submaps_buffer_.find(index)!=submaps_buffer_.end())
		return true;
	else{
		if(submaplists_.lists.find(index)==submaplists_.lists.end())
			return false;

		submap_process_queue_.Push(std::make_shared<ReadSubmapSignal>(SignalType::Read,index));
		return true;
	}
}

/// \brief 根据索引从缓存区拉取子地图
///
/// \param index 子地图索引
/// \return 子地图共享指针
std::shared_ptr<Submap>  SubmapManager::PullSubmap(int index)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if(submaps_buffer_.find(index)==submaps_buffer_.end())
		return nullptr;
	return submaps_buffer_[index];
}

/// \brief 切换到离线匹配模式
///
/// 切换到离线匹配模式，并根据输入索引值切换当前匹配地图
/// \param submap_index 子地图索引
void SubmapManager::SwitchToReadMode(int submap_index)
{
	CHECK(submaps_buffer_.find(submap_index)!=submaps_buffer_.end());
	LOG(WARNING)<<"SwitchToReadMode";
//    if(options_.savemap_flag())
//   	  submap_process_queue_.Push(std::make_shared<SaveSubmapSignal>(SignalType::Save,submaps().front()));
	options_.set_readmap_flag(true);
	options_.set_update_flag(false);

	resetsubmap(submap_index);
	PushFront(submaps_buffer_[submap_index]);

	set_global_pose({submaplists_.global_pose,submaplists_.gpsdata});
	UpdateSubmapBuffer();
}

/// \brief 切换到在线更新模式
///
/// 切换到在线更新模式，当rebuild为true时新建空地图，否则在原离线地图基础上进行更新
/// \param global_pose 车辆的全局位姿
/// \param rebuild 是否重建空地图
void SubmapManager::SwitchToUpdateMode(const ivcommon::transform::Rigid3d global_pose,bool rebuild)
{
	std::shared_ptr<Submap> tempsubmap = submaps().front();
	reset(true);

	options_.set_readmap_flag(false);
	options_.set_update_flag(true);
	int new_submap_index = submaplists_.lists.size();
	resetsubmap(new_submap_index);
	if(rebuild)
	{
		AddNewSubmap(global_init_pose().pose.inverse()*global_pose, global_pose);
	}
	else
	{
		int tempsubmap_index = tempsubmap->index();
		if(options_.savemap_flag())
		{
			LOG(WARNING)<<"new_submap_index:"<<new_submap_index;
			submaplists_.lists[tempsubmap_index].linkindexs.insert(new_submap_index);
		}
		tempsubmap->resetstate();
		tempsubmap->SetIndex(new_submap_index);
		tempsubmap->AddLinkIndex(tempsubmap_index);
		PushFront(tempsubmap);
	}

}

/// \brief 获取距离指定索引指定深度范围的子地图集合
///
/// \param index 目标子地图索引
/// \param maxdepth 最大深度
/// \return 指定子地图集合
std::set<int>  SubmapManager::GetNearSubmapSet(int index,int maxdepth)
{
	CHECK_GE(maxdepth,0);
	std::set<int> result;
	if(maxdepth<=0||submaplists_.lists.find(index)==submaplists_.lists.end())
		return result;
	const auto& linkindexs = submaplists_.lists[index].linkindexs;
	result.insert(linkindexs.begin(), linkindexs.end());
	for(auto linkindex:linkindexs)
	{
		const auto nearindexs = GetNearSubmapSet(linkindex,maxdepth-1);
		result.insert(nearindexs.begin(), nearindexs.end());
	}
	return result;
}

/// \brief 判断是否两个子地图相隔是否在指定深度范围内
///
/// \param index_base 子地图1索引
/// \param index 子地图2索引
/// \param maxdepth 最大深度阈值
/// \return 返回是否为相隔指定范围
bool SubmapManager::IsNearSubmap(int index_base,int index,int maxdepth)
{
	auto nearindexs = GetNearSubmapSet(index_base,maxdepth);
	if(nearindexs.find(index)!=nearindexs.end())
		return true;
	else
		return false;
}

/// \brief 切换子地图
///
/// \param pose_estimate 当前位姿

void SubmapManager::SwitchSubmap(const ivcommon::transform::Rigid3d& pose_estimate)
{
	  if(options_.readmap_flag())
		{
			static ivcommon::transform::Rigid3d last_pose_estimate = pose_estimate;
			Eigen::Vector3d moveorient = last_pose_estimate.rotation() *
					  (last_pose_estimate.inverse()*pose_estimate).translation();
			if(fabs(moveorient.norm()) < 1.)
				return;
			last_pose_estimate = pose_estimate;
			this->UpdateNearestSubmap(submaplists_.global_pose*pose_estimate);
			int matching_index = this->matching_index();
			bool isnear = IsNearSubmap(nearest_index_,matching_index,2);

			if((nearest_index_==-1||!isnear)
					&&!IsInTheSubmap(submaplists_.global_pose*pose_estimate, matching_index))
			{
				LOG(WARNING)<<"be not in the scope of map, exit matching mode ";
				LOG(WARNING)<<"nearest_index_:"<<nearest_index_
						<<" isnear:"<<isnear<<" matching_index:"<<matching_index;
				SwitchToUpdateMode(submaplists_.global_pose*pose_estimate,false);
			}
			else if(nearest_index_!=matching_index&&isnear
					&&this->RequestSubmap(nearest_index_))
			{
				auto submap = this->PullSubmap(nearest_index_);
				if(submap)
				{
					LOG(WARNING)<<"switch index:"<<nearest_index_;

					submaps().front()->Finish();
//					int index = active_submaps.submaps().front()->index();
					resetsubmap(submap->index());
					submap->resetstate();
					PushFront(submap);
					UpdateSubmapBuffer();
				}
			}

		}
	  else
	  {
			if(submaps().at(0)->finished())
			{
			     if(options_.savemap_flag())
			    	  submap_process_queue_.Push(std::make_shared<SaveSubmapSignal>(SignalType::Save,submaps().front()));

				  PopFront();
			}
	  }
}

/// \brief 更新子地图缓存区
///
/// 为了减少内存占用，每次切换地图后都需要更新子地图缓存区，删除距离远的地图

void SubmapManager::UpdateSubmapBuffer()
{
	  std::map<int,std::shared_ptr<Submap>> tmp_buffer;
	  mutex_.lock();
	  tmp_buffer.swap(submaps_buffer_);
	  int matching_index = this->matching_index();
	  submaps_buffer_[matching_index] = tmp_buffer[matching_index];
	  auto linkindexs = submaplists_.lists[matching_index].linkindexs;
	  for(const auto& index:linkindexs)
	  {
		  if(tmp_buffer.find(index)!=tmp_buffer.end())
			  submaps_buffer_[index] = tmp_buffer[index];
		  else
			  this->RequestSubmap(index);
	  }
	  mutex_.unlock();
}

/// \brief 创建新子地图
///
/// \param local_pose 当前车辆全局位姿
/// \param global_pose 被调整过的位姿（还未使用）
void SubmapManager::AddNewSubmap(const ivcommon::transform::Rigid3d& local_pose,const ivcommon::transform::Rigid3d& global_pose)
{
	ActiveSubmaps::AddSubmap(local_pose);
	submaps().back()->set_global_pose_adjusted(global_pose);
}

/// \brief 加载指定索引的子地图到缓存区
///
/// \param index 子地图索引

void SubmapManager::LoadSubmap(int index)
{
  CHECK(options_.readmap_flag()||options_.automode_flag());
  if(submaps_buffer_.find(index) != submaps_buffer_.end())
	  return;
  const mapping::SubmapHeader& submapheader = submaplists_.lists[index];
  const string& mapdir = options_.readmap_dir();

//  std::shared_ptr<Submap> temp_submap =std::make_shared<Submap>(options_.high_resolution(),options_.low_resolution(), options_.feature_resolution(),
//                                     options_.rotational_histogram_size(), submapheader.local_pose);
//  temp_submap->SetDepaturePose(submapheader.departure_pose);
////  submaps_.back()->SetIndex(submapheader.index);
//  temp_submap->load(submapheader.index,mapdir);

  std::stringstream sstr;
  sstr<<"/"<<submapheader.index;
  string filename = mapdir + sstr.str() + ".proto";
  ::ivcommon::io::ProtoStreamReader protoreader(filename);
  proto::Submap3D proto;
  protoreader.ReadProto(&proto);
  std::shared_ptr<Submap> temp_submap =std::make_shared<Submap>(proto);
//  string pcd_dir = mapdir + sstr.str() + ".pcd";
//  temp_submap->set_point_cloud_fromPCD(pcd_dir);
//  temp_submap->SetDepaturePose(submapheader.departure_pose);
  temp_submap->set_global_pose_adjusted(submapheader.global_pose_adjusted);
  for(const auto& linkindex:submapheader.linkindexs)
	  temp_submap->AddLinkIndex(linkindex);
  mutex_.lock();
  submaps_buffer_[submapheader.index] = temp_submap;
  mutex_.unlock();
//  LOG(INFO) << "Added submap " << matching_submap_index_ + submaps_.size();
}

std::shared_ptr<Submap> SubmapManager::GetCompleteSubmap(int index)
{
    const mapping::SubmapHeader& submapheader = submaplists_.lists[index];
    const string& mapdir = options_.readmap_dir();

    std::stringstream sstr;
    sstr<<"/"<<submapheader.index;
    string filename = mapdir + sstr.str() + ".proto";
    ::ivcommon::io::ProtoStreamReader protoreader(filename);
    proto::Submap3D proto;
    protoreader.ReadProto(&proto);
    std::shared_ptr<Submap> temp_submap =std::make_shared<Submap>(proto);
    string pcd_dir = mapdir + sstr.str() + ".pcd";
    temp_submap->set_point_cloud_fromPCD(pcd_dir);
    temp_submap->SetHeader(submapheader);
    return temp_submap;
}

//void SubmapManager::PushIntoSubmapList()
//{
//	mapping::SubmapHeader submapheader;
//	submapheader.index = active_submaps_.submaps().front()->index();
//	if(last_index_!=-1)
//	{
//		DCHECK(submaplists_.lists.find(last_index_)!=submaplists_.lists.end());
//		if(submaplists_.lists[last_index_].next_index==-1)
//			submaplists_.lists[last_index_].next_index =submapheader.index;
//	}
//
//	submapheader.last_index = last_index_;
//
//	if(active_submaps_.submaps().size()>1)
//		submapheader.next_index = active_submaps_.submaps()[1]->index();
//	else
//		submapheader.next_index = -1;
//}

/// \brief 保存子地图及索引到硬盘
///
/// \param submap_ptr 子地图指针
void SubmapManager::SaveSubmap(std::shared_ptr<Submap> submap_ptr)
{
  if(submap_ptr == nullptr)
    return;
  static std::ofstream filestream((options_.savemap_dir()+"/tempmaps.ind").c_str());
  static int last_index = -1;

//  SubmapHeader submapheader;
//  submapheader
  if(last_index<0)
    {

      ivcommon::transform::Rigid3d global_pose = global_init_pose().pose;
      filestream<<std::fixed<<std::setprecision(7)<<global_init_pose().gps.latitude
	  <<" "<<global_init_pose().gps.longitude<<" "<<global_init_pose().gps.heading
	  <<" "<<global_pose.rotation().x()<<" "<<global_pose.rotation().y()
	  <<" "<<global_pose.rotation().z()<<" "<<global_pose.rotation().w()
	  <<" "<<global_pose.translation()[0]<<" "<<global_pose.translation()[1]
	  <<" "<<global_pose.translation()[2]<<" "<<entrance_pose_.rotation().x()
	  <<" "<<entrance_pose_.rotation().y()<<" "<<entrance_pose_.rotation().z()
	  <<" "<<entrance_pose_.rotation().w()<<" "<<entrance_pose_.translation()[0]
	  <<" "<<entrance_pose_.translation()[1]<<" "<<entrance_pose_.translation()[2]<<std::endl;
    }

  int index = submap_ptr->index();
  CHECK_GE(index,0);
  if(submaplists_.lists.find(index)==submaplists_.lists.end())
  {
	  submap_ptr->saveToProto(options_.savemap_dir());
	  for(auto linkindex:submap_ptr->header().linkindexs)
	  {
		  if(submaplists_.lists.find(linkindex)!=submaplists_.lists.end())
		  {
			  submaplists_.lists[linkindex].linkindexs.insert(index); //补充相连header的连接节点
		  }
		  else
			  LOG(ERROR)<<"not find linkindex:"<<linkindex;
	  }
	  submaplists_.lists[index] = submap_ptr->header();
  }
  else
	  return;

  const ivcommon::transform::Rigid3d& pose = global_init_pose().pose*submap_ptr->local_pose();
  const ivcommon::transform::Rigid3d& global_pose_adjusted = submap_ptr->global_pose_adjusted();

  filestream<<std::fixed<<index<<" "<<submaplists_.lists[index].linkindexs.size()<<" ";
  for(auto temp_index:submaplists_.lists[index].linkindexs)
	  filestream<<temp_index<<" ";
  filestream<<std::setprecision(7)<<pose.rotation().x()<<" "<<pose.rotation().y()<<" "<<pose.rotation().z()<<" "<<pose.rotation().w()<<" "
	<<std::setprecision(3)<<pose.translation()[0]<<" "<<pose.translation()[1]<<" "<<pose.translation()[2]<<" "
	<<std::setprecision(7)<<global_pose_adjusted.rotation().x()<<" "<<global_pose_adjusted.rotation().y()
							<<" "<<global_pose_adjusted.rotation().z()<<" "<<global_pose_adjusted.rotation().w()<<" "
	<<std::setprecision(3)<<global_pose_adjusted.translation()[0]<<" "
							<<global_pose_adjusted.translation()[1]<<" "<<global_pose_adjusted.translation()[2]
	<<std::endl;



  auto time3 = ::ivcommon::now();

  last_index = index;
}

/// \brief 保存子地图列表
///
/// 保存子地图列表，列表中存有子地图索引及相互连接关系，位姿等信息
void SubmapManager::SaveSubmapList()
{
	  std::string filename = options_.savemap_dir()+ "/maps.ind";
	  std::ofstream filestream(filename.c_str());
	  sensor::GpsInsData gpsdata;
      ivcommon::transform::Rigid3d global_pose = global_init_pose().pose;
      filestream<<std::fixed<<std::setprecision(7)<<global_init_pose().gps.latitude
	  <<" "<<global_init_pose().gps.longitude<<" "<<global_init_pose().gps.heading
	  <<" "<<global_pose.rotation().x()<<" "<<global_pose.rotation().y()
	  <<" "<<global_pose.rotation().z()<<" "<<global_pose.rotation().w()
	  <<" "<<global_pose.translation()[0]<<" "<<global_pose.translation()[1]
	  <<" "<<global_pose.translation()[2]<<" "<<entrance_pose_.rotation().x()
	  <<" "<<entrance_pose_.rotation().y()<<" "<<entrance_pose_.rotation().z()
	  <<" "<<entrance_pose_.rotation().w()<<" "<<entrance_pose_.translation()[0]
	  <<" "<<entrance_pose_.translation()[1]<<" "<<entrance_pose_.translation()[2]<<std::endl;

      for(auto header:submaplists_.lists)
      {
    	  DCHECK_EQ(header.first,header.second.index);

          filestream<<std::fixed<<header.first<<" "<<header.second.linkindexs.size()<<" ";
          for(auto linkindex:header.second.linkindexs)
          {
        	  CHECK(submaplists_.lists.find(linkindex)!=submaplists_.lists.end())<<linkindex;
        	  filestream<<linkindex<<" ";
          }

          const ivcommon::transform::Rigid3d& pose = global_init_pose().pose*header.second.local_pose; //global pose
          const ivcommon::transform::Rigid3d& global_pose_adjusted = header.second.global_pose_adjusted;
          filestream<<std::setprecision(7)<<pose.rotation().x()<<" "<<pose.rotation().y()<<" "<<pose.rotation().z()<<" "<<pose.rotation().w()<<" "
        	<<std::setprecision(3)<<pose.translation()[0]<<" "<<pose.translation()[1]<<" "<<pose.translation()[2]<<" "
        	<<std::setprecision(7)<<global_pose_adjusted.rotation().x()<<" "<<global_pose_adjusted.rotation().y()
        							<<" "<<global_pose_adjusted.rotation().z()<<" "<<global_pose_adjusted.rotation().w()<<" "
        	<<std::setprecision(3)<<global_pose_adjusted.translation()[0]<<" "
        							<<global_pose_adjusted.translation()[1]<<" "<<global_pose_adjusted.translation()[2]
        	<<std::endl;
      }
      filestream.close();

}

/// \brief 加载子地图列表
///
void SubmapManager::LoadSubmapList()
{
  LOG(INFO)<<"LoadSubmapList";
  std::string filename = options_.readmap_dir() + "/maps.ind";
  std::ifstream file(filename.c_str());
  CHECK(file)<<"not find file:"<<filename;

  Eigen::Quaterniond globalrotation;
  Eigen::Vector3d globaltranslation;
  Eigen::Quaterniond entrancerotation;
  Eigen::Vector3d entrancetranslation;
  sensor::GpsInsData gpsdata;
  file>>gpsdata.latitude>>gpsdata.longitude>>gpsdata.heading //所有自地图的基位姿（即所有自地图的本地位姿均是相对于该位姿）
	>>globalrotation.x()>>globalrotation.y()>>globalrotation.z()>>globalrotation.w()
	>>globaltranslation[0]>>globaltranslation[1]>>globaltranslation[2]
	>>entrancerotation.x()>>entrancerotation.y()>>entrancerotation.z()>>entrancerotation.w()
	>>entrancetranslation[0]>>entrancetranslation[1]>>entrancetranslation[2];

  //transform to ros coordination
  submaplists_.global_pose = ivcommon::transform::Rigid3d(globaltranslation,globalrotation);
  submaplists_.entrance_pose = submaplists_.global_pose * ivcommon::transform::Rigid3d(entrancetranslation,entrancerotation);
  submaplists_.gpsdata = std::move(gpsdata);
  int index;
  while(file>>index) //子地图的索引
    {
	  mapping::SubmapHeader header;
	  header.index = index;
      Eigen::Quaterniond rotation;  //子地图的本地位姿，旋转变量（四元数），相对与全局坐标
      Eigen::Vector3d translation;//子地图的本地位姿，旋转变量（四元数），相对与全局坐标
      Eigen::Quaterniond nowrotation;
      Eigen::Vector3d nowtranslation;
      int linknum = 2;
      file>>linknum;
      for(int i=0;i<linknum;i++)
      {
    	  int linkindex = -1;
    	  file>>linkindex;
    	  DCHECK_GE(linkindex,0);
    	  header.linkindexs.insert(linkindex);
      }
      file>>rotation.x()>>rotation.y()>>rotation.z()>>rotation.w()
		>>translation[0]>>translation[1]>>translation[2]
		>>nowrotation.x()>>nowrotation.y()>>nowrotation.z()>>nowrotation.w()
		>>nowtranslation[0]>>nowtranslation[1]>>nowtranslation[2];
      header.local_pose = submaplists_.global_pose.inverse() * ivcommon::transform::Rigid3d(translation,rotation);
      header.global_pose_adjusted= ivcommon::transform::Rigid3d(nowtranslation,nowrotation);

      std::ifstream trajectory_reader;
      std::stringstream sstr;
      sstr<<"/"<<header.index;
      std::string trajfilename = options_.readmap_dir() +sstr.str()+ ".traj";
      trajectory_reader.open(trajfilename.c_str(),std::ios::in);
      if(trajectory_reader.is_open())
      {
    	  int size = 0;
    	  trajectory_reader>>size;

    	  for(int i=0;i<size;i++)
    	  {
              Eigen::Quaterniond estimaterotation;  //子地图的本地位姿，旋转变量（四元数），相对与全局坐标
              Eigen::Vector3d estimatetranslation;//子地图的本地位姿，旋转变量（四元数），相对与全局坐标
              Eigen::Quaterniond gpsrotation;
              Eigen::Vector3d gpstranslation;
              Eigen::Quaterniond fuserotation;
              Eigen::Vector3d fusetranslation;
        	  mapping::trajectoryPose node;
        	  trajectory_reader>>estimaterotation.x()>>estimaterotation.y()>>estimaterotation.z()>>estimaterotation.w()
        	  				>>estimatetranslation[0]>>estimatetranslation[1]>>estimatetranslation[2]
        	  				>>gpsrotation.x()>>gpsrotation.y()>>gpsrotation.z()>>gpsrotation.w()
        	  				>>gpstranslation[0]>>gpstranslation[1]>>gpstranslation[2];
        	  bool withfusenode = false;
        	  trajectory_reader>>withfusenode
			  	  	  	  	>>fuserotation.x()>>fuserotation.y()>>fuserotation.z()>>fuserotation.w()
			          	  	>>fusetranslation[0]>>fusetranslation[1]>>fusetranslation[2];
        	  node.estimatepose = ivcommon::transform::Rigid3d(estimatetranslation,estimaterotation);
        	  node.gpspose = ivcommon::transform::Rigid3d(gpstranslation,gpsrotation);
        	  node.withfusepose = withfusenode;
        	  node.fusepose = ivcommon::transform::Rigid3d(fusetranslation,fuserotation);
        	  header.trajectory.push_back(node);
    	  }

      }
      submaplists_.lists[header.index] = header;
    }
//  submaplists_.lists[header.index].next_index = -1;
  LOG(INFO)<<"LoadSubmapList finished";
}

}//mapping3d

