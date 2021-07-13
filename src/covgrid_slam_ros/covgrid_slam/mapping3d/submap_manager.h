/*!
* \file submap_manager.h
* \brief 子地图管理程序
*
* 进行子地图的管理、加载、保存，子地图切换，模式切换等
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/
#ifndef COVGRID_SLAM_MAPPING3D_SUBMAP_MANAGER_H_
#define COVGRID_SLAM_MAPPING3D_SUBMAP_MANAGER_H_
#include "covgrid_slam/mapping3d/submaps.h"
#include "ivcommon/common/blocking_queue.h"
#include <boost/bind.hpp>
#include <boost/thread.hpp>

/// \brief 三维里程计及建图
///
namespace mapping3d {
/// \brief 子地图信号类型
///
enum class SignalType{
  Save =0, Read=1, Insert=4, Return=5, Match=6
};

/// \brief 子地图信号基类
///
class SubmapSignal{
public:
	  SubmapSignal(SignalType type):signaltype(type){}
	virtual ~SubmapSignal(){}
	SignalType signaltype;
};
/// \brief 保存匹配子地图信号及数据
///
class SaveSubmapSignal:public SubmapSignal{
public:
	SaveSubmapSignal(SignalType type,std::shared_ptr<Submap> data):SubmapSignal(type),submap(data){}
	std::shared_ptr<Submap> submap;
};
/// \brief 加载子地图信号及数据
///
class ReadSubmapSignal:public SubmapSignal{
public:
	ReadSubmapSignal(SignalType type,int index):SubmapSignal(type),submapindex(index){}
	int submapindex;
};
/// \brief 插入子地图信号及数据
///
class InsertSubmapSignal:public SubmapSignal{
public:
	InsertSubmapSignal(SignalType type,std::shared_ptr<sensor::RangeData> data):SubmapSignal(type),rangedata(data){}
	std::shared_ptr<sensor::RangeData> rangedata;
};


/// \brief 匹配子地图信号及数据
///
class MatchSubmapSignal:public SubmapSignal{
public:
	MatchSubmapSignal(std::shared_ptr<sensor::RangeData> data):SubmapSignal(SignalType::Match),rangedata(data){}
	std::shared_ptr<sensor::RangeData> rangedata;
};


/// \brief 子地图管理类
///
/// 进行子地图的管理、加载、保存，子地图切换，模式切换等，该类继承ActiveSubmaps
class SubmapManager:public ActiveSubmaps{

public:
	explicit SubmapManager(proto::SubmapsOptions& options,const mapping3d::PosewithGps& global_pose_init);
	~SubmapManager();

	void FinishAndSaveSubmap();
//	void reset()
//	{
//	  if(options_.readmap_flag())
//		{
//		  LoadNextSubmap();
//		  submap_process_queue_.Push(std::make_shared<SubmapSignal>(SignalType::ReadFront));
//		}
//	}

	void SwitchSubmap(const ivcommon::transform::Rigid3d& pose_estimate);

	void PopAndSaveFinishedSubmap();
	void UpdateNearestSubmap(const ivcommon::transform::Rigid3d& pose);
	int UpdateNearestSubmapGps(const ivcommon::transform::Rigid3d& pose);
	void UpdateSubmapBuffer();
	::ivcommon::transform::Rigid3d TransformPoseInWorldToSubmap(const ivcommon::transform::Rigid3d& pose,int submap_index)
	{
		return (submaplists_.global_pose*submaplists_.lists[submap_index].local_pose).inverse()*pose;
	}

	::ivcommon::transform::Rigid3d TransformPoseInSubmapToWorld(const ivcommon::transform::Rigid3d& pose,int submap_index)
	{
		return (submaplists_.global_pose*submaplists_.lists[submap_index].local_pose)*pose;
	}

	::ivcommon::transform::Rigid3d TransformMapGlobalPoseToGpsPose(const ivcommon::transform::Rigid3d& pose)
	{
		CHECK(options_.readmap_flag());
		int matching_index = this->matching_index();
		CHECK(submaplists_.lists.find(matching_index)!=submaplists_.lists.end());
		auto nodeind = submaplists_.lists[matching_index].FindNearstNode(pose);
		CHECK_GE(nodeind,0);
		auto node = submaplists_.lists[matching_index].trajectory[nodeind];
		auto ref_pose = node.gpspose;

		if(node.withfusepose)
			ref_pose = node.fusepose;
//		auto error = (ref_pose*node.estimatepose.inverse()*pose).inverse() * (ref_pose*(node.estimatepose.inverse()*pose));
//		LOG(ERROR)<<error;
//		CHECK(error.translation().norm()<1.2);
		return ref_pose*(node.estimatepose.inverse()*pose);
	}

	int nearest_submap_index()
	{
		return nearest_index_;
	}

	bool RequestSubmap(int index);
	std::shared_ptr<Submap>  PullSubmap(int index);
	void SwitchToReadMode(int submap_index);
	void SwitchToUpdateMode(const ivcommon::transform::Rigid3d global_pose,bool rebuild=false);
	void UpdateSubmapGlobalPose();

	const SubmapLists& submap_lists()
	{
		return submaplists_;
	}

	void reset(bool resetsubmaps = true)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		submap_process_queue_.Clear();
//		set_matching_index(submaplists_.lists.size());
		nearest_index_ = -1;
		submaps_buffer_.clear();
		if(resetsubmaps)
			resetsubmap(submaplists_.lists.size());
	}

	std::set<int> GetNearSubmapSet(int index,int maxdepth);
	bool IsNearSubmap(int index_base,int index,int maxdepth);
	std::shared_ptr<Submap> GetCompleteSubmap(int index); //for offline multi-map integration
	void SaveSubmap(std::shared_ptr<Submap> submap_ptr);
	void SaveSubmapList();
private:
	bool IsInTheSubmap(const ivcommon::transform::Rigid3d& pose,int submap_index);
	bool IsInTheSubmapForGps(const ivcommon::transform::Rigid3d& pose,int submap_index);
	void SubmapThread();
	void LoadSubmapList();
	void LoadSubmap(int index);
	void AddNewSubmap(const ivcommon::transform::Rigid3d& local_pose,const ivcommon::transform::Rigid3d& global_pose);
	proto::SubmapsOptions& options_;//!< 子地图相关参数
	SubmapLists submaplists_;//!<子地图列表
	std::map<int,std::shared_ptr<Submap>> submaps_buffer_;//!<子地图缓存器
	int nearest_index_;//!<最近邻位姿

	::ivcommon::BlockingQueue<std::shared_ptr<SubmapSignal>> submap_process_queue_;//!<子地图处理队列
	std::mutex mutex_;//!<子地图线程同步锁
	boost::thread submap_io_thread_;//!<子地图读写线程
	::ivcommon::transform::Rigid3d entrance_pose_;//!<子地图入口位姿，已弃用. now using submap->local_pose

};

}//mapping3d
#endif /* COVGRID_SLAM_MAPPING3D_SUBMAP_MANAGER_H_ */
