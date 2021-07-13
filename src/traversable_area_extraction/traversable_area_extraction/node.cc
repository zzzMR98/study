#include "node.h"
#include "ivcommon/common/common.h"
#include <fstream>
#include <memory>
#include <chrono>
// #include <time.h>
/**************地图定义**************/
/*
栅格数值含义：
0：未知(0-黑色)		1：可通行(100-浅绿色)		2：障碍物(110-红色)
3：正障碍(170-白色)	4：负障碍(130-黄色)		5：悬崖(250-蓝色)
6：水（绿色）		7：正斜坡(70-紫色) 		８：负斜坡(70-紫色)
8:车辆(170-红色)   9:非平坦区域　0<uneven<=1:　值越小越不平坦0.15 以下为非平坦区域（靛色）

分辨率：0.2m

X方向：201个栅格
Y方向：351个栅格

车辆位置：100,100
*/
/**************地图定义**************/
std::array<int, 3> color[] = { {0,0,0},{125,125,125},{0,0,255},
    {255,255,255},{0,255,255},{255,0,0},
    {0,255,0},{255,0,255},{125,0,125},{255,255,0},{50,50,255} };
std::vector<std::string> DataTypeString{ "PrimaryTraversableArea", "Positive", "KNegative", "Stiff", "Water",
    "PositiveSlope", "NegativeSlope", "UnevenArea", "LidarOdometry", "BackOGM", "RefinePositive" };

namespace traversable_area_extraction{
//该程序多用到了::，看情况而定，::有代表全局意义的功能。
//根据接收ros消息的函数写了一个接收消息的模板函数 返回一个订阅者对象
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
  void (Node::* handler)(const typename MessageType::ConstPtr&),
  const string& topic,
  ::ros::NodeHandle* const node_handle, Node* const node){
  return node_handle->subscribe<MessageType>(
    topic, kInfiniteSubscriberQueueSize,
    boost::function<void(const typename MessageType::ConstPtr&)>(
      [node, handler](const typename MessageType::ConstPtr& msg){
        (node->*handler)(msg);
      }));
}


Node::Node(ros::NodeHandle& nh, TraversableAreaOption& traversable_area_option) :
  nh_(nh), traversable_area_option_(traversable_area_option){
  Init();
  LOG(INFO) << "traversablearea_option loaded ";// << traversable_area_inited;
  StartSubscriber();
}

void Node::Init(){
  ros::NodeHandle pnh("~");
  pnh.param<bool>("use_gps_to_match_when_lidarodometry_invalid", use_gps_to_match_when_lidarodometry_invalid, false);
  // 加载先验地图开关
  if (traversable_area_option_.load_priormap){
    initial_localtime = traversable_area_option_.priormap_file_name_time; // 先验地图文件夹名称
    LoadPriorMapProfile(); // 加载先验地图文件夹的所有信息  
  }
  else{
    initial_localtime = ::ivcommon::GetCurrentTime();//std::string
    global_origin_pose = ::ivcommon::transform::Rigid3d::Identity();//这个是第一帧的位姿，也就是之后所有local pose坐标系的参考位姿
    traversabalmap_headers.clear();//std::map<int, MapHeader>
  }
  priormapprocessor_running = false;
  latest_map_index = -1; // 先验地图读取时候备用的在线的index
  last_priormapindex = -1;
  last_priormapwrite_index = -1;
  integrate_current_primarymapdata = -1;
  for (int i = 0; i < kObstacletypes; i++){
    map_size_updating[i] = false;
    map_size_switching[i] = false;
    common_processsor_running[i] = false;
  }
  use_location_module = false;
  TraversableAreaProcessor_running = false;

  traversable_area_inited = false;
  traversable_area_finished = true;
  map_init_time = ::ivcommon::Time::min(); // 返回一个低于1970.01.01的数。
  gps_zone = ::ivcommon::transform::UTM_ZONE_AUTO;
  latest_vechicle_pose.time = ::ivcommon::Time::min();//LidarOdometryData
  // ::ivcommon::transform::Hemisphere 
  hemi = ::ivcommon::transform::HEMI_NORTH;
  // 发布消息 std::map<std::string, ros::Publisher> publishers
  publishers[KFinalTraversableAreaVehiclePoseTopicName] =
    nh_.advertise<nav_msgs::Odometry>(KFinalTraversableAreaVehiclePoseTopicName, 1);

  publishers[KFinalTraversableAreaTopicName] =
    nh_.advertise<iv_slam_ros_msgs::TraversableArea>(KFinalTraversableAreaTopicName, 1);

  publishers[KSingleTraversableAreaTopicName] =
    nh_.advertise<iv_slam_ros_msgs::TraversableArea>(KSingleTraversableAreaTopicName, 1);

  publishers[KOptimizedFinalTraversableAreaTopicName] =
    nh_.advertise<iv_slam_ros_msgs::TraversableArea>(KOptimizedFinalTraversableAreaTopicName, 1);

  publishers[KOptimizedSingleTraversableAreaTopicName] =
    nh_.advertise<iv_slam_ros_msgs::TraversableArea>(KOptimizedSingleTraversableAreaTopicName, 1);

  if (traversable_area_option_.display_on){
    visualize_thread = std::make_shared<boost::thread>(boost::thread(boost::bind(&Node::visualizeTraversableArea, this)));
    keep_visualize_thread_running = true;
  }
  else{
    keep_visualize_thread_running = false;
    visualize_thread = nullptr;
  }

  LOG(INFO) << "traversablearea_option loaded";
  wall_timers_.push_back(nh_.createWallTimer(
    ::ros::WallDuration(traversable_area_option_.traversable_area_publish_period_sec),//std::vector<::ros::WallTimer> wall_timers_
    &Node::PublishFinalTraversableArea, this)); // 设置 wall clock 每0.1s执行一次
// 这里有个差别是walltime和普通的ros::Timer的区别，一个是实际意义上的时间，一个是ros的时间。
}

void Node::StartSubscriber(){
  //订阅各种话题
  LOG(INFO) << "StartSubscriber run start";
  //融合正负障碍物
  if (traversable_area_option_.integrate_negativeobeject){
    subscribers_.push_back(SubscribeWithHandler<negative_msgs::NegativeOGM>(
      &Node::HandleNegativeObjectMessage, KNegativeObjectTopicName, &nh_, this)); //yes
  }
  if (traversable_area_option_.integtate_slopeobeject){
    subscribers_.push_back(SubscribeWithHandler<slopeogm_msgs::SlopeOGM>(
      &Node::HandleSlopeObjectMessage, KSlopeObjectTopicName, &nh_, this));
  }
  if (traversable_area_option_.integtate_stiffobeject){
    subscribers_.push_back(SubscribeWithHandler<stiff_msgs::stiffwater>(
      &Node::HandleStiffObjectMessage, KStiffObjectTopicName, &nh_, this));
  }
  if (traversable_area_option_.integtate_unevenareaobeject){
    subscribers_.push_back(SubscribeWithHandler<uneven_area_msgs::HeightMap>(
      &Node::HandleUnevenAreaObjectMessage, KUnevenAreaTopicName, &nh_, this));//todo
  }
  if (traversable_area_option_.wiping_dynamicobejectflag){
    subscribers_.push_back(SubscribeWithHandler<iv_dynamicobject_msgs::moving_target_send>(
      &Node::HandleDynamicObjectMessage, KDynamicObjectTopicName, &nh_, this));
  }


  subscribers_.push_back(SubscribeWithHandler<covgrid_slam_msgs::LidarOdometryForMapping>(
    &Node::HandleLidarOdometryMessage, KLidarOdometryTopicName, &nh_, this));

  subscribers_.push_back(SubscribeWithHandler<obstacle_msgs::ObstacleOGM>(
    &Node::HandleBackOgmMessage, KBackObstacleTopicName, &nh_, this));
  //最先执行这个primaryTraversableArea
  subscribers_.push_back(SubscribeWithHandler<iv_slam_ros_msgs::PrimarytraversableArea>(
    &Node::HandlePrimaryTraversableAreaMessage, KPrimaryTraversableAreaTopicName, &nh_, this));

  subscribers_.push_back(SubscribeWithHandler<iv_slam_ros_msgs::Traversablevehiclepose>(
    &Node::HandleTraversableAreaVehiclePose, KTraversableAreaVehiclePoseTopicName, &nh_, this));

  subscribers_.push_back(SubscribeWithHandler<signal_msgs::SmogEnvironment>(
    &Node::HandleSmogSignal, KSmogSignalTopicName, &nh_, this));
  subscribers_.push_back(SubscribeWithHandler<sensor_driver_msgs::GpswithHeading>(
    &Node::HandleSensorFusionOutput, KSensorFusionOutputTopicName, &nh_, this));

  if (use_gps_to_match_when_lidarodometry_invalid) {
    subscribers_.push_back(SubscribeWithHandler<sensor_driver_msgs::GpswithHeading>(
      &Node::HandleGPSMsg,"/gpsdata" ,&nh_, this));
  }

  LOG(INFO) << "StartSubscriber run over";
}

Node::~Node(){
  keep_visualize_thread_running = false;
  PriorMapheadersWriter();////<缓存先验地图头信息
  if (traversable_area_option_.display_on && visualize_thread != nullptr && visualize_thread->joinable()){
    visualize_thread->join(); //等待该线程完成后，才能继续向下运行
  }
}

void Node::visualizeTraversableArea() {
  while (keep_visualize_thread_running){
    // visualize final_traversable_area_img, final_dilated_traversable_area_img,
    // single_traversable_area_img, single_dilated_traversable_area_img
    while (!final_traversable_area_img_mtx.try_lock_for(std::chrono::milliseconds(2))) {//拿到锁返回ture zmr?
      // final_traversable_area_img_mtx.lock();
      if (!final_traversable_area_img.empty()){
        cv::imshow("final_traversable_area", final_traversable_area_img);
      }
      final_traversable_area_img_mtx.unlock();
    }
    while (!final_dilated_traversable_area_img_mtx.try_lock_for(std::chrono::milliseconds(2))) {
      // final_dilated_traversable_area_img_mtx.lock();
      if (!final_dilated_traversable_area_img.empty()){
        cv::imshow("final_dilated_traversable_area", final_dilated_traversable_area_img);
      }
      final_dilated_traversable_area_img_mtx.unlock();
    }
    while (!single_traversable_area_img_mtx.try_lock_for(std::chrono::milliseconds(2))) {
      // single_traversable_area_img_mtx.lock();
      if (!single_traversable_area_img.empty()){
        cv::imshow("single_traversable_area", single_traversable_area_img);
      }
      single_traversable_area_img_mtx.unlock();
    }
    while (!single_dilated_traversable_area_img_mtx.try_lock_for(std::chrono::milliseconds(2))) {
      // single_dilated_traversable_area_img_mtx.lock();
      if (!single_dilated_traversable_area_img.empty()){
        cv::imshow("single_dilated_traversable_area", single_dilated_traversable_area_img);
      }
      single_dilated_traversable_area_img_mtx.unlock();
    }
    if (traversable_area_option_.load_priormap/* && primary_traversable_area_img_mtx.try_lock()*/) {
      // primary_traversable_area_img_mtx.lock();
      // if (!primary_traversable_area_img.empty()) {
      //   cv::imshow("primary_traversable_area", primary_traversable_area_img);
      // }
      if (priormap_globalvalue.Display_Image_ != nullptr) {
        cvShowImage("Priormap", priormap_globalvalue.Display_Image_);
        // cvWaitKey(1);
      }
      // primary_traversable_area_img_mtx.unlock();
    }
    // show_obstacle_img, traversable_area_data_img
    // if (!show_obstacle_img.empty()) {
    //   cv::imshow("show_obstacle_img", show_obstacle_img);
    // }
    // if (!traversable_area_data_img.empty()) {
    //   cv::imshow("traversable_area_data_img", traversable_area_data_img);
    // }
    cv::waitKey(5);//每5ms刷新一次
    usleep(40000);
  }
}

//<缓存先验地图头信息
void Node::PriorMapheadersWriter(){
  if (traversable_area_option_.priormap_write){
    ::ivcommon::MutexLocker lock(&mutex_priormap_write);
    std::string priormap_filename = "";
    std::string file_modle_name = "priormap";
    priormap_filename = ::ivcommon::file_directory_generate(initial_localtime, file_modle_name);
    priormap_filename += "priormapdata_table.ivrc";
    std::fstream priormapdata_file;
    priormapdata_file.open(priormap_filename, std::ios::out);
    if (traversabalmap_headers.empty()){
      LOG(WARNING) << "traversabalmap_headers.size();" << traversabalmap_headers.size();
      LOG(WARNING) << "traversabalmap_headers is empty!";
      priormapdata_file.close();
      return;
    }

    priormapdata_file << traversabalmap_headers.size() << '\t'
      << std::fixed << std::setprecision(7) << global_origin_pose.translation().x()
      << '\t' << global_origin_pose.translation().y() << '\t' <<
      global_origin_pose.translation().z() << '\t' << global_origin_pose.rotation().w()
      << '\t' << global_origin_pose.rotation().x() << '\t' << global_origin_pose.rotation().y()
      << '\t' << global_origin_pose.rotation().z() << '\n';
    ivcommon::transform::Rigid3d tem_local_pose = ivcommon::transform::Rigid3d::Identity();
    ivcommon::transform::Rigid3d tem_gpslocation_pose = ivcommon::transform::Rigid3d::Identity();
    double a = 6378137; double e2 = 0.0818192 * 0.0818192;//e的平方
    double tem_vehicle_latitude, tem_vehicle_longitude;
    // ::ivcommon::transform::Hemisphere hemi = ::ivcommon::transform::HEMI_NORTH;
    for (int i = 0; i < traversabalmap_headers.size();i++){

      tem_local_pose = global_origin_pose * traversabalmap_headers[i].pose;
      tem_gpslocation_pose = global_origin_pose * traversabalmap_headers[i].location_module_pose;
      Eigen::Vector3d local_eular_pose = ivcommon::transform::QuaterniondtoPitchRollYaw(tem_local_pose.rotation());
      Eigen::Vector3d location_eular_pose = ivcommon::transform::QuaterniondtoPitchRollYaw(traversabalmap_headers[i].location_module_pose.rotation());
      ::ivcommon::transform::grid_to_geographic(a, e2, gps_zone, hemi, tem_gpslocation_pose.translation().y(),
        tem_gpslocation_pose.translation().x() + 500000, &tem_vehicle_latitude, &tem_vehicle_longitude);
      LOG(ERROR) << "gps_zone" << gps_zone;
      LOG(ERROR) << "tem_gpslocation_pose" << tem_gpslocation_pose << "tem_vehicle_longitude:" << tem_vehicle_longitude * 180 / M_PI << "tem_vehicle_latitude:" << tem_vehicle_latitude * 180 / M_PI;

      priormapdata_file << i << '\t' << std::fixed << std::setprecision(7) << traversabalmap_headers[i].resolution << '\t' << traversabalmap_headers[i].height << '\t' << traversabalmap_headers[i].width
        << '\t' << traversabalmap_headers[i].pose_index_x << '\t' << traversabalmap_headers[i].pose_index_y << '\t' << tem_local_pose.translation().x()
        << '\t' << tem_local_pose.translation().y() << '\t' << tem_local_pose.translation().z() << '\t' << tem_local_pose.rotation().w()
        << '\t' << tem_local_pose.rotation().x() << '\t' << tem_local_pose.rotation().y() << '\t' << tem_local_pose.rotation().z() << '\t' <<
        tem_gpslocation_pose.translation().x()
        << '\t' << tem_gpslocation_pose.translation().y() << '\t' << tem_gpslocation_pose.translation().z() << '\t' << tem_gpslocation_pose.rotation().w()
        << '\t' << tem_gpslocation_pose.rotation().x() << '\t' << tem_gpslocation_pose.rotation().y() << '\t' << tem_gpslocation_pose.rotation().z()
        << '\t' << local_eular_pose.x() << '\t' << local_eular_pose.y() << '\t' << local_eular_pose.z() << '\t' << location_eular_pose.x()
        << '\t' << location_eular_pose.y() << '\t' << location_eular_pose.z() << '\t' << tem_vehicle_longitude * 180 / M_PI << '\t' << tem_vehicle_latitude * 180 / M_PI << '\n';

      LOG(INFO) << i << '\t' << traversabalmap_headers[i].resolution << '\t' << traversabalmap_headers[i].height << '\t' << traversabalmap_headers[i].width
        << '\t' << traversabalmap_headers[i].pose_index_x << '\t' << traversabalmap_headers[i].pose_index_y << '\t' << traversabalmap_headers[i].pose;
    }
    priormapdata_file.close();
    LOG(INFO) << "traversabalmap_headers is written to file:" << priormap_filename;
  }
}

void Node::LoadPriorMapProfile(){
  std::string priormap_filename = "";
  std::string file_modle_name = "priormap";
  priormap_filename = ::ivcommon::file_directory_generate(initial_localtime, file_modle_name);//生成文件夹存储priormap
  priormap_filename += "priormapdata_table.ivrc";
  std::fstream priormapdata_file;
  LOG(WARNING) << "it is going to read:" << priormap_filename;
  priormapdata_file.open(priormap_filename, std::ios::in);
  if (!priormapdata_file.is_open()){
    LOG(ERROR) << "priormapdata_file for offline twidmap invoking open failed!";
    LOG(WARNING) << priormapdata_file.goodbit;
    LOG(WARNING) << priormapdata_file.good();
  }

  int tem_priormap_index = 0;
  int size = 0;
  priormapdata_file >> size;//取值
  LOG(WARNING) << "active_submaps_.twid_map_data.size(): " << size;
  double tem_vehicle_latitude, tem_vehicle_longitude;
  double translation_x(0), translation_y(0), translation_z(0), gps_translation_x(0), gps_translation_y(0), gps_translation_z(0);
  double quarternion_w(0), quarternion_x(0), quarternion_y(0), quarternion_z(0), gps_quarternion_w(0), gps_quarternion_x(0), gps_quarternion_y(0), gps_quarternion_z(0);
  priormapdata_file >> translation_x >> translation_y >> translation_z >> quarternion_w >> quarternion_x >> quarternion_y >> quarternion_z;//平移及旋转的四元数
  global_origin_pose = ivcommon::transform::Rigid3<double>{ Eigen::Matrix<double, 3, 1>(translation_x,translation_y,translation_z),
                                                   Eigen::Quaternion<double>(quarternion_w,quarternion_x,quarternion_y,quarternion_z) };//初始位姿全局坐标系的
  double euler_x(0), euler_y(0), euler_z(0);
  for (int i = 0; i < size;i++){
    priormapdata_file >> tem_priormap_index;//第几张先验地图
    priormapdata_file >> traversabalmap_headers[tem_priormap_index].resolution;
    priormapdata_file >> traversabalmap_headers[tem_priormap_index].height;
    priormapdata_file >> traversabalmap_headers[tem_priormap_index].width;
    priormapdata_file >> traversabalmap_headers[tem_priormap_index].pose_index_x;//应该是在栅格图中车辆的位置
    priormapdata_file >> traversabalmap_headers[tem_priormap_index].pose_index_y;

    priormapdata_file >> translation_x >> translation_y >> translation_z >> quarternion_w >> quarternion_x >> quarternion_y >> quarternion_z;
    // gps的数据全是0,0,0,1,0,0,0
    priormapdata_file >> gps_translation_x >> gps_translation_y >> gps_translation_z >> gps_quarternion_w >> gps_quarternion_x >> gps_quarternion_y >> gps_quarternion_z;
    priormapdata_file >> euler_x >> euler_y >> euler_z;
    priormapdata_file >> euler_x >> euler_y >> euler_z;//这两次欧拉角不一样? 一个是里程计的 一个是定位模块
    priormapdata_file >> tem_vehicle_longitude >> tem_vehicle_latitude;//0 0 融合定位模块的 经度 经度
    std::vector<double> temp_xy{translation_x, translation_y};
    priormap_xy.push_back(temp_xy); //std::vector<std::vector<double> >
    traversabalmap_headers[tem_priormap_index].pose = global_origin_pose.inverse() * ivcommon::transform::Rigid3<double>{Eigen::Matrix<double, 3, 1>(translation_x, translation_y, translation_z),
      Eigen::Quaternion<double>(quarternion_w, quarternion_x, quarternion_y, quarternion_z)};// 这是进行什么变换 zmr?
    traversabalmap_headers[tem_priormap_index].location_module_pose = global_origin_pose.inverse() * ::ivcommon::transform::Rigid3<double>{
      Eigen::Matrix<double, 3, 1>(gps_translation_x, gps_translation_y, gps_translation_z), Eigen::Quaternion<double>(gps_quarternion_w, gps_quarternion_x, gps_quarternion_y, gps_quarternion_z)};
    LOG(INFO) << tem_priormap_index << '\t' << traversabalmap_headers[tem_priormap_index].resolution << '\t' << traversabalmap_headers[tem_priormap_index].height << '\t' << traversabalmap_headers[tem_priormap_index].width
      << '\t' << traversabalmap_headers[tem_priormap_index].pose_index_x << '\t' << traversabalmap_headers[tem_priormap_index].pose_index_y << '\t' << traversabalmap_headers[tem_priormap_index].pose;
  }
  LOG(INFO) << "priormap mode global_origin_pose:" << global_origin_pose;
  ROS_INFO_STREAM("Priori map data is loaded,map size:" << traversabalmap_headers.size());
  priormapdata_file.close();
}

//<根据前端感知信息调用先验地图信息并发布
void Node::PriorMapProcessor(int current_priormapindex, MapData& current_primarytraversablearea){
  //  return;
  if (priormapprocessor_running == true) return;
  priormapprocessor_running = true;
  // ivcommon::transform::Hemisphere hemi = ivcommon::transform::HEMI_NORTH;
  if (latest_vechicle_pose.time == ::ivcommon::Time::min()){
    LOG(WARNING) << "latest_vechicle_pose.time hasn't be inited";
    priormapprocessor_running = false;
    return;
  }

  LidarOdometryData tem_latest_vechicle_pose = latest_vechicle_pose;

  if (current_priormapindex != last_priormapindex){
    ROS_INFO("it is going to invoke priori map: %d",current_priormapindex);
    // LOG(INFO) << "it is going to invoke priori map: " << current_priormapindex;
    std::string twidmap_file_name = "";
    std::string file_modle_name = "priormap";
    twidmap_file_name = ::ivcommon::file_directory_generate(initial_localtime, file_modle_name);
    std::stringstream tem_stringstream;
    tem_stringstream.clear();
    tem_stringstream.str("");
    tem_stringstream << current_priormapindex;
    twidmap_file_name += tem_stringstream.str();
    twidmap_file_name += ".jpg";//读取的第几张图

    if (priormap_globalvalue.prior_map != nullptr){
      cvReleaseImage(&(priormap_globalvalue.prior_map));
      priormap_globalvalue.prior_map = nullptr;
    }

    (priormap_globalvalue.prior_map) = cvLoadImage(twidmap_file_name.c_str(), -1);// -1默认读取图像的原通道数

    if (priormap_globalvalue.Display_Image_ != nullptr){
      cvReleaseImage(&priormap_globalvalue.Display_Image_);
      priormap_globalvalue.Display_Image_ = nullptr;
    }
    priormap_globalvalue.Display_Image_ = cvCreateImage(cvSize(priormap_globalvalue.prior_map->width, priormap_globalvalue.prior_map->height), IPL_DEPTH_8U, 3);
    cvZero(priormap_globalvalue.Display_Image_);

    /*没用到*/
    unsigned char* pdata = (unsigned char*)priormap_globalvalue.prior_map->imageData;
    unsigned char* display_data = (unsigned char*)priormap_globalvalue.Display_Image_->imageData;

    for (int i = 0;i < priormap_globalvalue.Display_Image_->height;i++){
      pdata = (unsigned char*)priormap_globalvalue.prior_map->imageData + priormap_globalvalue.prior_map->widthStep * i;
      display_data = (unsigned char*)priormap_globalvalue.Display_Image_->imageData + priormap_globalvalue.Display_Image_->widthStep * i;
      for (int j = 0;j < priormap_globalvalue.Display_Image_->width;j++){
        if (*pdata == 124){
          display_data[0] = 0;
          display_data[1] = 255;
          display_data[2] = 0;
        }
        else if (*pdata >= 250){
          display_data[0] = 0;
          display_data[1] = 0;
          display_data[2] = 255;
        }
        pdata++;
        display_data += 3;
      }
    }
    /*没用到*/
    //在图中画出格子以及车辆位置（建立地图时的车辆位置）
    double resolution = traversabalmap_headers[current_priormapindex].resolution;
    int heightnum = priormap_globalvalue.Display_Image_->height / (10 / resolution);
    int widthnum = priormap_globalvalue.Display_Image_->width / (10 / resolution);
    for (int i = 0; i < heightnum;i++){
      cvLine(priormap_globalvalue.Display_Image_,
        cvPoint(0, priormap_globalvalue.Display_Image_->height * i / heightnum),
        cvPoint(priormap_globalvalue.Display_Image_->width - 1,
          priormap_globalvalue.Display_Image_->height * i / heightnum), cvScalar(255, 0, 0));
    }

    for (int i = 1;i < widthnum;i++){
      cvLine(priormap_globalvalue.Display_Image_, cvPoint(priormap_globalvalue.Display_Image_->width * i / widthnum, 0),
        cvPoint(priormap_globalvalue.Display_Image_->width * i / widthnum, priormap_globalvalue.Display_Image_->height - 1), cvScalar(255, 0, 0));
    }
    cvRectangle(priormap_globalvalue.Display_Image_, cvPoint(traversabalmap_headers[current_priormapindex].pose_index_x - 1 / resolution,
      priormap_globalvalue.Display_Image_->height - 1 - traversabalmap_headers[current_priormapindex].pose_index_y + 2 / resolution),
      cvPoint(traversabalmap_headers[current_priormapindex].pose_index_x + 1 / resolution,
      priormap_globalvalue.Display_Image_->height - 1 - traversabalmap_headers[current_priormapindex].pose_index_y - 2 / resolution),
      cvScalar(0, 255, 255));

    priormap_globalvalue.traversable_area_msg.header.frame_id = global_frame_str;
    // LOG(ERROR) << "!!!!!!!!!!!!!!!!!!!11";
    priormap_globalvalue.traversable_area_msg.header.stamp = ::ivcommon::ToRos(tem_latest_vechicle_pose.time);
    // LOG(ERROR) << "!!!!!!!!!!!!!!!!!!!11";
    priormap_globalvalue.traversable_area_msg.height = priormap_globalvalue.prior_map->height;
    priormap_globalvalue.traversable_area_msg.width = priormap_globalvalue.prior_map->width;
    priormap_globalvalue.traversable_area_msg.index = current_priormapindex;
    priormap_globalvalue.traversable_area_msg.resolution = resolution;
    priormap_globalvalue.traversable_area_msg.triD_submap_pose_image_index_x = traversabalmap_headers[current_priormapindex].pose_index_x;
    priormap_globalvalue.traversable_area_msg.triD_submap_pose_image_index_y = traversabalmap_headers[current_priormapindex].pose_index_y;

    double tem_submap_latitude, tem_submap_longitude, tem_location_submap_latitude, tem_location_submap_longitude;
    ::ivcommon::transform::Rigid3d tem_pendingpub_submappose = ::ivcommon::transform::Rigid3d::Identity();
    ::ivcommon::transform::Rigid3d tem_location_pendingpub_submappose = ::ivcommon::transform::Rigid3d::Identity();
    if (traversabalmap_headers[current_priormapindex].use_location_module){
      //           CHECK(twid_map_data[mapping_index].gps_global_pose!= ::ivcommon::transform::Rigid3d::Identity());
      tem_pendingpub_submappose = global_origin_pose * traversabalmap_headers[current_priormapindex].location_module_pose;
    }
    else{

      tem_pendingpub_submappose = global_origin_pose * traversabalmap_headers[current_priormapindex].pose;//里程计位姿 全局

    }
    priormap_globalvalue.current_mappose = tem_pendingpub_submappose;
    ivcommon::transform::grid_to_geographic(a, e2, gps_zone, hemi, (tem_pendingpub_submappose.translation().y()),
      (tem_pendingpub_submappose.translation().x() + 500000),
      &tem_submap_latitude, &tem_submap_longitude);//再转回到经纬度
    priormap_globalvalue.traversable_area_msg.triD_submap_pose.position.x = tem_submap_longitude * 180 / M_PI;
    priormap_globalvalue.traversable_area_msg.triD_submap_pose.position.y = tem_submap_latitude * 180 / M_PI;
    priormap_globalvalue.traversable_area_msg.triD_submap_pose.position.z = tem_pendingpub_submappose.translation().z();

    priormap_globalvalue.traversable_area_msg.triD_submap_pose.orientation.w = tem_pendingpub_submappose.rotation().w();
    priormap_globalvalue.traversable_area_msg.triD_submap_pose.orientation.x = tem_pendingpub_submappose.rotation().x();
    priormap_globalvalue.traversable_area_msg.triD_submap_pose.orientation.y = tem_pendingpub_submappose.rotation().y();
    priormap_globalvalue.traversable_area_msg.triD_submap_pose.orientation.z = tem_pendingpub_submappose.rotation().z();
    priormap_globalvalue.traversable_area_msg.submap_finished_flag = true;
    unsigned char* pixeldata = (unsigned char*)priormap_globalvalue.prior_map->imageData;

    priormap_globalvalue.traversable_area_msg.cells.clear();
    for (int i = 0; i < priormap_globalvalue.prior_map->height;i++){
      pixeldata = (unsigned char*)priormap_globalvalue.prior_map->imageData + priormap_globalvalue.prior_map->widthStep * (priormap_globalvalue.prior_map->height - 1 - i);
      for (int j = 0;j < priormap_globalvalue.prior_map->width;j++){
        if (*pixeldata >= 250){
          priormap_globalvalue.traversable_area_msg.cells.push_back(2);
        }
        else if (*pixeldata != 0){
          priormap_globalvalue.traversable_area_msg.cells.push_back(1);
        }
        else{
          priormap_globalvalue.traversable_area_msg.cells.push_back(0);
        }
        pixeldata++;
      }
    }
    last_priormapindex = current_priormapindex;
    // integrate_current_primarymapdata++;
    integrate_current_primarymapdata += 2;
  }
  //////////////////////////1210用当前的准可通行区域作为一个校正
  iv_slam_ros_msgs::TraversableArea pending_pub_traversable_area_msg = priormap_globalvalue.traversable_area_msg;
  if (integrate_current_primarymapdata > 0) {
    if (current_primarytraversablearea.header.width > 0 && current_primarytraversablearea.header.height > 0){
      ROS_INFO_THROTTLE(1, "integrate primary map into final map");
      ::ivcommon::transform::Rigid3d tem_relativepose = priormap_globalvalue.current_mappose.inverse() * current_primarytraversablearea.header.pose;//基于先验地图全局位姿的准可通行区域局部位姿
      {
        for (int i = 0;i < current_primarytraversablearea.data.size();i++){
          // if (current_primarytraversablearea.data[i] == 2){
          if (current_primarytraversablearea.data[i] > 1){
            double relative_x = (i % current_primarytraversablearea.header.width - current_primarytraversablearea.header.pose_index_x) * current_primarytraversablearea.header.resolution;
            double relative_y = (i / current_primarytraversablearea.header.width - current_primarytraversablearea.header.pose_index_y) * current_primarytraversablearea.header.resolution;
            Eigen::Vector3d relative_translation = tem_relativepose * Eigen::Vector3d(relative_x, relative_y, 0);
            int index_x = relative_translation.x() / priormap_globalvalue.traversable_area_msg.resolution;
            int index_y = relative_translation.y() / priormap_globalvalue.traversable_area_msg.resolution;
            int tem_index = index_y * priormap_globalvalue.traversable_area_msg.width + index_x;
            if (tem_index < pending_pub_traversable_area_msg.cells.size()){
              // pending_pub_traversable_area_msg.cells[tem_index] = 2;
              pending_pub_traversable_area_msg.cells[tem_index] = 99;
            }
          }
        }
      }
    }
    else{
      // LOG(WARNING) << "Current online primarytraversable area data is null !";
      ROS_WARN_THROTTLE(5, "Current online primarytraversable area data is null !");
    }
  }

  // LOG(WARNING) << " PriorMap loaded:" << last_priormapindex;
  nav_msgs::Odometry vehicle_global_pose_;
  vehicle_global_pose_.header.stamp = ::ivcommon::ToRos(tem_latest_vechicle_pose.time);
  vehicle_global_pose_.header.frame_id = global_frame_str;
  ::ivcommon::transform::Rigid3d vechicle_pose = global_origin_pose * tem_latest_vechicle_pose.pose;//这里到底是转到了哪个坐标系 local? zmr??
  double tem_vehicle_latitude, tem_vehicle_longitude;
  ::ivcommon::transform::grid_to_geographic(a, e2, gps_zone, hemi,
    (vechicle_pose.translation().y()), (vechicle_pose.translation().x() + 500000),
    &tem_vehicle_latitude, &tem_vehicle_longitude);
  vehicle_global_pose_.pose.pose.position.x = tem_vehicle_longitude * 180 / M_PI;
  vehicle_global_pose_.pose.pose.position.y = tem_vehicle_latitude * 180 / M_PI;
  vehicle_global_pose_.pose.pose.position.z = vechicle_pose.translation().z();
  vehicle_global_pose_.pose.pose.orientation.w = vechicle_pose.rotation().w();
  vehicle_global_pose_.pose.pose.orientation.x = vechicle_pose.rotation().x();
  vehicle_global_pose_.pose.pose.orientation.y = vechicle_pose.rotation().y();
  vehicle_global_pose_.pose.pose.orientation.z = vechicle_pose.rotation().z();
  TraversableAreaOptimization(pending_pub_traversable_area_msg,
    vechicle_pose, KOptimizedFinalTraversableAreaTopicName, final_dilated_traversable_area_img);
  publishers[KFinalTraversableAreaVehiclePoseTopicName].publish(vehicle_global_pose_);
  publishers[KFinalTraversableAreaTopicName].publish(pending_pub_traversable_area_msg);
  LOG(INFO) << "vehicle_global_pose_ and traversable_area_msg_ published, traversable_area_msg_ index: " << current_priormapindex;

  Eigen::Vector3d tem_submap_eular_pose =
    ivcommon::transform::QuaterniondtoPitchRollYaw(
      (global_origin_pose * traversabalmap_headers[current_priormapindex].pose).rotation());
  Eigen::Vector3d tem_vehicle_eular_pose =
    ivcommon::transform::QuaterniondtoPitchRollYaw(vechicle_pose.rotation());
  ivcommon::transform::Rigid3d tem_vehicle_pose =
    ::ivcommon::transform::Rigid3d(vechicle_pose.translation(),
      ivcommon::transform::PitchRollYaw(tem_submap_eular_pose.x(),
        tem_submap_eular_pose.y(), tem_vehicle_eular_pose.z()));
  ::ivcommon::transform::Rigid3d tem_relative_pose =
    (global_origin_pose * traversabalmap_headers[current_priormapindex].pose).inverse() *
    tem_vehicle_pose;
  double ogmresolution = traversabalmap_headers[current_priormapindex].resolution;

  cvCircle(priormap_globalvalue.Display_Image_,
    cvPoint(tem_relative_pose.translation().x() /
      ogmresolution + traversabalmap_headers[current_priormapindex].pose_index_x,
      priormap_globalvalue.Display_Image_->height - 1 -
      (tem_relative_pose.translation().y() / ogmresolution +
        traversabalmap_headers[current_priormapindex].pose_index_y)), 5,
    cvScalar(0, 255, 255), -1);
  // cvShowImage("Priormap", priormap_globalvalue.Display_Image_);
  // cvWaitKey(1);
  // LOG(INFO) << "PriorMapProcessor finished!";
  priormapprocessor_running = false;
}

//<保存先验地图
void Node::PriorMapWriter(){
  if (priormap_globalvalue.priormapmode_opened == true){ return; }
  iv_slam_ros_msgs::TraversableArea traversable_area_msg = priormap_globalvalue.last_traversable_area_msg;
  IplImage* PriormapImage = cvCreateImage(cvSize(traversable_area_msg.width, traversable_area_msg.height), IPL_DEPTH_8U, 1);
  //   cv::Mat::zeros( traversable_area_msg.height,traversable_area_msg.width,CV_8UC1);
  cvZero(PriormapImage);
  unsigned char* imagedata;
  for (int i = 0;i < traversable_area_msg.height;i++){
    imagedata = (uchar*)(PriormapImage->imageData + (traversable_area_msg.height - 1 - i) * PriormapImage->widthStep);
    unsigned char* pdata = nullptr;
    for (int j = 0; j < traversable_area_msg.width; j++){
      pdata = imagedata + j;
      if (traversable_area_msg.cells[i * traversable_area_msg.width + j] == 2){
        pdata[0] = 254;
        //       LOG(ERROR)<<"pdata[0]= 255;";
      }
      else if (traversable_area_msg.cells[i * traversable_area_msg.width + j] == 1){
        //         pdata =  imagedata + j;
        pdata[0] = 100;
      }/*else{
pdata[0]= 0;
//       LOG(ERROR)<<"pdata[0]= 0;";
    }*/

    }
  }
  std::string priormap_filename = "";
  std::string file_modle_name = "priormap";
  priormap_filename = ::ivcommon::file_directory_generate(initial_localtime, file_modle_name);
  std::stringstream tem_stringstream;
  tem_stringstream.clear();
  tem_stringstream.str("");
  tem_stringstream << priormap_globalvalue.last_traversable_area_header.index;
  priormap_filename += tem_stringstream.str();
  priormap_filename += ".jpg";
  cvSaveImage(priormap_filename.c_str(), PriormapImage);
 //std::map<int, MapHeader> traversabalmap_headers
  traversabalmap_headers[priormap_globalvalue.last_traversable_area_header.index] = priormap_globalvalue.last_traversable_area_header;
  LOG(WARNING) << "PriormapImage was written: " << priormap_globalvalue.last_traversable_area_header.index;
}

void Node::SingleTraversableAreaProcessor(DataType datatype_
  , MapData& fusemap, const MapData& mapdata)//jkj 0807
{
  //是这样的逻辑：先是正障碍检测，把可通行和正障碍标识出来，再是负障碍，只标障碍物的位置，后来的障碍物检测出来的图都是只标障碍物的位置。
  //也就是以正障碍的图为基准做的。
  //fusemap里面没有数据只有地图的尺寸和位姿，真正的数据在mapdata里面
  //这里是不是存在一个问题，就是如果
//   ::ivcommon::MutexLocker lock(&mutex);
  int passible_flag(1), obstacle_flag(0), out_obstacle_flag(SendDataType::KObstacle);

  if (datatype_ == DataType::KNegative){
    obstacle_flag = 4;//negative obstacle
  }
  if (datatype_ == DataType::KPositive || datatype_ == DataType::KBackOGM){
    obstacle_flag = 3;//positive obstacle
  }
  if (datatype_ == DataType::KPositiveSlope){
    obstacle_flag = 7;//slope obstacle
  }
  if (datatype_ == DataType::KNegativeSlope){
    obstacle_flag = 8;//slope obstacle
  }
  if (datatype_ == DataType::KStiff){
    obstacle_flag = 5;//stiff obstacle
  }
  if (datatype_ == DataType::KWater){
    obstacle_flag = 6;//water obstacle
    out_obstacle_flag = SendDataType::KWaterObstacle;
  }
  if (datatype_ == DataType::KUnevenArea){
    obstacle_flag = 9;//uneven obstacle
  }
  if (datatype_ == DataType::KRefinePositive){
    obstacle_flag = 10;
    out_obstacle_flag = SendDataType::KRefineObstacle;
  }
  int another_obstacle_flag = obstacle_flag;
  if (datatype_ == DataType::KBackOGM)
    another_obstacle_flag = 10;
  ::ivcommon::transform::Rigid3d tem_reference_pose = fusemap.header.pose.inverse()
    * mapdata.header.pose;//位姿变换
  for (int i = 0;i < mapdata.header.height;i++){
    for (int j = 0;j < mapdata.header.width; j++){
      int tem_value = mapdata.data[i * mapdata.header.width + j];
      //将mapdata栅格里面的值转换成fusemap里面的值
      if (tem_value == obstacle_flag || tem_value == another_obstacle_flag){
        double tem_x = (j - mapdata.header.pose_index_x) * mapdata.header.resolution;
        double tem_y = (i - mapdata.header.pose_index_y) * mapdata.header.resolution;
        Eigen::Vector3d tem_pose1(tem_x, tem_y, 0);
        Eigen::Vector3d tem_relative_pose1 = tem_reference_pose * tem_pose1;


        int tem_index_x = tem_relative_pose1.x() / fusemap.header.resolution
          + fusemap.header.pose_index_x;
        int tem_index_y = tem_relative_pose1.y() / fusemap.header.resolution
          + fusemap.header.pose_index_y;

        if (tem_index_x >= 0 && tem_index_x < fusemap.header.width &&
          tem_index_y >= 0 && tem_index_y < fusemap.header.height){

          fusemap.data.at(
            tem_index_y * fusemap.header.width + tem_index_x) = tem_value;

        }

        continue;
      }
      if (tem_value == passible_flag && datatype_ == DataType::KPositive)//正障碍检测的可通行区域
      {
        double tem_y = (i - mapdata.header.pose_index_y) * mapdata.header.resolution;
        double tem_x = (j - mapdata.header.pose_index_x) * mapdata.header.resolution;
        Eigen::Vector3d tem_pose1(tem_x, tem_y, 0);
        Eigen::Vector3d tem_relative_pose1 = tem_reference_pose * tem_pose1;
        int tem_index_x = tem_relative_pose1.x() / fusemap.header.resolution + fusemap.header.pose_index_x;
        int tem_index_y = tem_relative_pose1.y() / fusemap.header.resolution + fusemap.header.pose_index_y;
        if (tem_index_x >= 0 && tem_index_x < fusemap.header.width &&
          tem_index_y >= 0 && tem_index_y < fusemap.header.height
          && fusemap.data.at(tem_index_y * fusemap.header.width + tem_index_x) == 0){
          fusemap.data.at(tem_index_y * fusemap.header.width + tem_index_x) = passible_flag;
        }
        continue;
      }
    }
  }

}

void Node::PublishSingleTraversableArea()//jkj 0807
{
  std::deque<LidarOdometryData> temp_lidar_odometry_data;
  {
    ::ivcommon::MutexLocker lock(&mutex_lidarodometry);
    if (lidar_odometry_data.empty()){
      LOG(WARNING) << "lidar_odometry_data is empty";
      return;
    }
    temp_lidar_odometry_data = lidar_odometry_data;
  }
  MapData traversable_area;
  double map_height = 80;
  double map_width = 40;
  double vehicle_x = 20;
  double vehicle_y = 30;
  double resolution = 0.2;
  double tem_submap_latitude, tem_submap_longitude;
  int tem_width = map_width / resolution + 1;
  int tem_height = map_height / resolution + 1;
  ::ivcommon::transform::Rigid3d tem_map_pose = temp_lidar_odometry_data.back().pose;//当前帧数据

  traversable_area.header.resolution = resolution;
  traversable_area.header.height = tem_height;
  traversable_area.header.width = tem_width;

  traversable_area.header.pose_index_x = vehicle_x / traversable_area.header.resolution + 1;
  traversable_area.header.pose_index_y = vehicle_y / traversable_area.header.resolution + 1;
  traversable_area.header.pose = tem_map_pose;
  traversable_area.header.time = temp_lidar_odometry_data.back().time;
  traversable_area.data.assign(tem_height * tem_width, 0);


  //    LOG(INFO)<<" roadblock_data2[datatype_].size():"<<roadblock_data2[datatype_].size();
  {
    ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
    for (auto it = roadblock_data2.begin();it != roadblock_data2.end();it++){
      auto datatype_ = it->first;
      if (datatype_ == DataType::KBackOGM){
        if (temp_lidar_odometry_data.size() > 0 && roadblock_data2[datatype_].size() > 0){
          auto& tmp_map = roadblock_data2[datatype_].back();
          tmp_map.header.pose = temp_lidar_odometry_data.back().pose;
          SingleTraversableAreaProcessor(datatype_, traversable_area, tmp_map);
        }
        continue;
      }
      bool time_matched = false;
      int odom_size = temp_lidar_odometry_data.size();
      int odom_start = std::max(0, odom_size - 2);
      for (int map_i = roadblock_data2[datatype_].size() - 1;map_i >= 0;map_i--){

        time_matched = false;


        if (roadblock_data2[datatype_].at(map_i).header.time < temp_lidar_odometry_data[odom_start].time){

          roadblock_data2[datatype_].erase(roadblock_data2[datatype_].begin() + map_i);
          continue;
        }
        //	      LOG(INFO)<<"temp_lidar_odometry_data.size()="<<odom_size;
        //	      LOG(INFO)<<"roadblock_data2["<<datatype_<<"].size()="<<roadblock_data2[datatype_].size();
        for (int i = odom_start;i < odom_size;i++){
          auto& tmp_map = roadblock_data2[datatype_].at(map_i);
          //    	LOG(INFO)<<::ivcommon::ToSeconds(roadblock_data2[datatype_].front().time - temp_lidar_odometry_data[i].time);
          if (tmp_map.header.time == temp_lidar_odometry_data[i].time){
            if (temp_lidar_odometry_data[i].time > latest_vechicle_pose.time){
              latest_vechicle_pose.time = temp_lidar_odometry_data[i].time;
              latest_vechicle_pose.pose = temp_lidar_odometry_data[i].pose;
            }
            time_matched = true;

            tmp_map.header.pose = temp_lidar_odometry_data[i].pose;
            SingleTraversableAreaProcessor(datatype_, traversable_area, tmp_map);
            roadblock_data2[datatype_].erase(roadblock_data2[datatype_].begin(), roadblock_data2[datatype_].begin() + map_i);
            break;
          }
        }
        if (time_matched) break;
      }
    }
  }
  iv_slam_ros_msgs::TraversableArea traversable_area_msg;
  traversable_area_msg.header.frame_id = global_frame_str;
  traversable_area_msg.header.stamp = ::ivcommon::ToRos(traversable_area.header.time);
  traversable_area_msg.resolution = traversable_area.header.resolution;
  traversable_area_msg.height = traversable_area.header.height;
  traversable_area_msg.width = traversable_area.header.width;

  traversable_area_msg.triD_submap_pose_image_index_x = traversable_area.header.pose_index_x;
  traversable_area_msg.triD_submap_pose_image_index_y = traversable_area.header.pose_index_y;

  tem_map_pose = global_origin_pose * tem_map_pose;
  ::ivcommon::transform::grid_to_geographic(a, e2, gps_zone, hemi,
    (tem_map_pose.translation().y()),
    (tem_map_pose.translation().x() + 500000), &tem_submap_latitude, &tem_submap_longitude);

  traversable_area_msg.triD_submap_pose.position.x = tem_submap_longitude * 180 / M_PI;
  traversable_area_msg.triD_submap_pose.position.y = tem_submap_latitude * 180 / M_PI;
  traversable_area_msg.triD_submap_pose.position.z = tem_map_pose.translation().z();

  traversable_area_msg.triD_submap_pose.orientation.w = tem_map_pose.rotation().w();
  traversable_area_msg.triD_submap_pose.orientation.x = tem_map_pose.rotation().x();
  traversable_area_msg.triD_submap_pose.orientation.y = tem_map_pose.rotation().y();
  traversable_area_msg.triD_submap_pose.orientation.z = tem_map_pose.rotation().z();

  traversable_area_msg.submap_finished_flag = true;
  //    LOG(INFO)<<std::fixed<<std::setprecision(10)<<"tem_submap_longitude*180/M_PI"<<tem_submap_longitude*180/M_PI<<"tem_submap_latitude*180/M_PI"<<tem_submap_latitude*180/M_PI<<"tem_map_pose.translation().x()"<<tem_map_pose.translation().x()<<"tem_map_pose.translation().y()"<<tem_map_pose.translation().y()<<tem_map_pose;

  traversable_area_msg.cells.assign(tem_width * tem_height, 0);
  cv::Mat tmp_img = cv::Mat::zeros(traversable_area.header.height, traversable_area.header.width, CV_8UC3);
  for (int j = 0;j < traversable_area.header.height;j++){
    // uchar* pdata = tmp_img.ptr<uchar>(traversable_area.header.height - 1 - j);
    for (int i = 0;i < traversable_area.header.width;i++){
      int index = j * traversable_area.header.width + i;
      auto value = traversable_area.data[index];
      if (value > 1){
        // pdata[3 * i] = color[value][0];
        // pdata[3 * i + 1] = color[value][1];
        // pdata[3 * i + 2] = color[value][2];
        tmp_img.at<cv::Vec3b>(traversable_area.header.height - 1 - j,i)[0] = color[value][0];
        tmp_img.at<cv::Vec3b>(traversable_area.header.height - 1 - j,i)[1] = color[value][1];
        tmp_img.at<cv::Vec3b>(traversable_area.header.height - 1 - j,i)[2] = color[value][2];
        if (value == 10)
          traversable_area_msg.cells[index] = SendDataType::KRefineObstacle;
        else
          traversable_area_msg.cells[index] = SendDataType::KObstacle;

      }
      else if (value == 1){
        traversable_area_msg.cells[index] = SendDataType::KPassibility;
      }
    }
  }
  publishers[KSingleTraversableAreaTopicName].publish(traversable_area_msg);
  LOG(INFO) << KSingleTraversableAreaTopicName << " cost time:" << (ros::Time::now() - traversable_area_msg.header.stamp).toSec();
  TraversableAreaOptimization(traversable_area_msg, tem_map_pose, KOptimizedSingleTraversableAreaTopicName, single_dilated_traversable_area_img);
  int heightnum = tem_height / (10 / resolution);
  int widthnum = tem_width / (10 / resolution);
  for (int i = 0; i < heightnum;i++){
    cv::line(tmp_img, cv::Point(0, tmp_img.rows * i / heightnum),
      cv::Point(tmp_img.cols - 1, tmp_img.rows * i / heightnum), cvScalar(255, 0, 0));
  }
  for (int i = 1;i < widthnum;i++){
    cv::line(tmp_img, cv::Point(tmp_img.cols * i / widthnum, 0),
      cv::Point(tmp_img.cols * i / widthnum, tmp_img.rows - 1), cvScalar(255, 0, 0));
  }
  cv::rectangle(tmp_img, cv::Point(traversable_area.header.pose_index_x - 1 / resolution,
    tmp_img.rows - 1 - traversable_area.header.pose_index_y + 2 / resolution),
    cv::Point(traversable_area.header.pose_index_x + 1 / resolution,
      tmp_img.rows - 1 - traversable_area.header.pose_index_y - 2 / resolution),
    cv::Scalar(0, 0, 255));
  while (!single_traversable_area_img_mtx.try_lock_for(std::chrono::milliseconds(2))) {
    single_traversable_area_img = std::move(tmp_img);
    single_traversable_area_img_mtx.unlock();
  }
  // if (traversable_area_option_.display_on)         {
  //   cv::imshow(KSingleTraversableAreaTopicName, traversable_area_img);
  //   cv::waitKey(1);
  // }
}

void Node::PublishFinalTraversableArea(const ros::WallTimerEvent& event){
  // clock_t start_time = clock();
  // ::ivcommon::MutexLocker lock(&mutex);
  if (!traversable_area_inited){
    // LOG(INFO)<<"traversable_area not inited";
    traversable_area_finished = true;
    return;
  }
  PublishSingleTraversableArea();
  // if (publishers[KFinalTraversableAreaTopicName].getNumSubscribers() == 0 &&
  //         publishers[KOptimizedFinalTraversableAreaTopicName].getNumSubscribers() == 0) {
  //     return;
  // }
  if (traversable_area_finished == false){
    traversable_area_finished = true;
    return;
  }

  //   if(pending_pub_traversable_area_data.empty()){
  //     LOG(WARNING)<<"pending_pub_traversable_area_data is empty";
  //     traversable_area_finished = true;
  //     return;
  //   }
  LidarOdometryData tem_latest_vechicle_pose = latest_vechicle_pose;
  TraversableAreaData tem_pending_pub_traversable_area_data[kObstacletypes];
  {
    ::ivcommon::MutexLocker lock(&mutex_pending_pub_data);
    if (pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.time == ::ivcommon::Time::min() ||
      tem_latest_vechicle_pose.time == ::ivcommon::Time::min()){
      traversable_area_finished = true;
      return;
    }

    if (pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].data.empty()){
      LOG(WARNING) << "pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].data.empty()";
      traversable_area_finished = true;
      return;
    }
    traversable_area_finished = false;
    {
      for (int i = 0;i < kObstacletypes;i++){
        tem_pending_pub_traversable_area_data[i] = pending_pub_traversable_area_data[i];
      }
    }
  }
  for (int i = 0; i < kObstacletypes; i++){
    if (tem_pending_pub_traversable_area_data[i].header.time > ::ivcommon::Time::min() &&
      tem_pending_pub_traversable_area_data[i].header.index != tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.index){
      LOG(ERROR) << "index is not equal: " << tem_pending_pub_traversable_area_data[i].header.index
        << " vs " << tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.index;
    }
  }
  const auto& mapheader = tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header;

  tem_latest_vechicle_pose.pose = global_origin_pose * tem_latest_vechicle_pose.pose; // 相对于global的位姿

  iv_slam_ros_msgs::TraversableArea traversable_area_msg;

  traversable_area_msg.header.frame_id = global_frame_str;
  traversable_area_msg.header.stamp = ::ivcommon::ToRos(mapheader.time);

  traversable_area_msg.height = mapheader.height;
  traversable_area_msg.width = mapheader.width;
  traversable_area_msg.resolution = mapheader.resolution;
  traversable_area_msg.triD_submap_pose_image_index_x = mapheader.pose_index_x;
  traversable_area_msg.triD_submap_pose_image_index_y = mapheader.pose_index_y;

  double tem_submap_latitude, tem_submap_longitude;
  ::ivcommon::transform::Rigid3d tem_map_pose = ::ivcommon::transform::Rigid3d::Identity();
  if (/*mapheader.use_location_module*/0){
    // CHECK(tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].location_module_pose!= ::ivcommon::transform::Rigid3d::Identity());
    tem_map_pose = global_origin_pose * mapheader.location_module_pose;
  }
  else{
    tem_map_pose = global_origin_pose * mapheader.pose;//global的位姿
  }
  ::ivcommon::transform::grid_to_geographic(a, e2, gps_zone, hemi,
    (tem_map_pose.translation().y()),
    (tem_map_pose.translation().x() + 500000), &tem_submap_latitude, &tem_submap_longitude);

  traversable_area_msg.triD_submap_pose.position.x = tem_submap_longitude * 180 / M_PI;
  traversable_area_msg.triD_submap_pose.position.y = tem_submap_latitude * 180 / M_PI;
  traversable_area_msg.triD_submap_pose.position.z = tem_map_pose.translation().z();

  traversable_area_msg.triD_submap_pose.orientation.w = tem_map_pose.rotation().w();
  traversable_area_msg.triD_submap_pose.orientation.x = tem_map_pose.rotation().x();
  traversable_area_msg.triD_submap_pose.orientation.y = tem_map_pose.rotation().y();
  traversable_area_msg.triD_submap_pose.orientation.z = tem_map_pose.rotation().z();

  traversable_area_msg.submap_finished_flag = mapheader.finished;
  LOG(INFO) << std::fixed << std::setprecision(10) << "tem_submap_longitude*180/M_PI: " <<
    tem_submap_longitude * 180 / M_PI << ", tem_submap_latitude*180/M_PI: " <<
    tem_submap_latitude * 180 / M_PI << ", tem_map_pose.translation().x(): " <<
    tem_map_pose.translation().x() << ", tem_map_pose.translation().y(): " <<
    tem_map_pose.translation().y() << tem_map_pose;
  int tem_width = mapheader.width;
  int tem_height = mapheader.height;
  traversable_area_msg.cells.assign(tem_width * tem_height, 0);

  /****************************************************  ****************************************************/

  int tem_index_x = mapheader.pose_index_x;
  int tem_index_y = mapheader.pose_index_y;
  cv::Mat showimage = cv::Mat::zeros(tem_height, tem_width, CV_8UC3);

  unsigned char* showdata;

  /**************************************************** map ****************************************************/

  for (int i = 0; i < tem_height;i++){
    // showdata = showimage.ptr<uchar>(tem_height - 1 - i);
    for (int j = 0;j < tem_width;j++){
      //20181126修改lzz   应该是利用准可通行区域的障碍物检测去搞
      /*if(traversable_area_option_.integtate_mapping_traversable_area &&
              tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].data.at(i * tem_width +j).possibility>0.5) {

          traversable_area_msg.cells[i * tem_width +j] = SendDataType::KObstacle;

          unsigned char * pdata = showdata + j*3 ;
          pdata[0] = 0;
          pdata[1] = 0;
          pdata[2] = 255;

      } else if(tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].data.at(i * tem_width +j).possibility>0) {
          traversable_area_msg.cells[i * tem_width +j] =SendDataType::KPassibility;
          unsigned char * pdata = showdata + j*3 ;
          pdata[0] = 0;
          pdata[1] = 255;
          pdata[2] = 0;
      }*/
      if (mapheader.index == tem_pending_pub_traversable_area_data[DataType::KNegative].header.index){
        int index_x = j - traversable_area_msg.triD_submap_pose_image_index_x
          + tem_pending_pub_traversable_area_data[DataType::KNegative].header.pose_index_x;
        int index_y = i - traversable_area_msg.triD_submap_pose_image_index_y
          + tem_pending_pub_traversable_area_data[DataType::KNegative].header.pose_index_y;
        if (index_x >= 0 && index_x < tem_pending_pub_traversable_area_data[DataType::KNegative].header.width
          && index_y >= 0 && index_y < tem_pending_pub_traversable_area_data[DataType::KNegative].header.height){
          int tem_data_index = index_y * tem_pending_pub_traversable_area_data[DataType::KNegative].header.width + index_x;
          if (tem_pending_pub_traversable_area_data[DataType::KNegative].data.at(tem_data_index).possibility > 0.5){
            traversable_area_msg.cells[i * tem_width + j] = SendDataType::KObstacle;
            // unsigned char* pdata = showdata + j * 3;
            // pdata[0] = 0;
            // pdata[1] = 255;
            // pdata[2] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[0] = 0;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[1] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[2] = 255;
            continue;
          }
          // else if(tem_pending_pub_traversable_area_data[DataType::KNegative].data[tem_data_index].possibility>0) {
          //     traversable_area_msg.cells[i * tem_width +j] =1;
          //     pdata = (unsigned char *)(showimage->imageData+(showimage->height-1-i)*showimage->widthStep +(j)*3 ) ;
          //     pdata[0] = 125;
          //     pdata[1] = 125;
          //     pdata[2] = 125;
          // }
        }
      }
      if (mapheader.index == tem_pending_pub_traversable_area_data[DataType::KPositive].header.index){
        int index_x = j - traversable_area_msg.triD_submap_pose_image_index_x
          + tem_pending_pub_traversable_area_data[DataType::KPositive].header.pose_index_x;
        int index_y = i - traversable_area_msg.triD_submap_pose_image_index_y
          + tem_pending_pub_traversable_area_data[DataType::KPositive].header.pose_index_y;
        if (index_x >= 0 && index_x < tem_pending_pub_traversable_area_data[DataType::KPositive].header.width
          && index_y >= 0 && index_y < tem_pending_pub_traversable_area_data[DataType::KPositive].header.height){
          int tem_data_index = index_y * tem_pending_pub_traversable_area_data[DataType::KPositive].header.width + index_x;
          if (tem_pending_pub_traversable_area_data[DataType::KPositive].data.at(tem_data_index).possibility > 0.5){

            traversable_area_msg.cells[i * tem_width + j] = SendDataType::KObstacle;
            // unsigned char* pdata = showdata + j * 3;
            // pdata[0] = 255;
            // pdata[1] = 255;
            // pdata[2] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[0] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[1] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[2] = 255;
            continue;
          }
          // else if(tem_pending_pub_traversable_area_data[DataType::KPositive].data[tem_data_index].possibility>0) {
          //     traversable_area_msg.cells[i * tem_width +j] =1;
          //     pdata = (unsigned char *)(showimage->imageData+(showimage->height-1-i)*showimage->widthStep +(j)*3 ) ;
          //     pdata[0] = 125;
          //     pdata[1] = 125;
          //     pdata[2] = 125;
          // }
        }
      }
      if (mapheader.index == tem_pending_pub_traversable_area_data[DataType::KRefinePositive].header.index){//jkj
        int index_x = j - traversable_area_msg.triD_submap_pose_image_index_x
          + tem_pending_pub_traversable_area_data[DataType::KRefinePositive].header.pose_index_x;
        int index_y = i - traversable_area_msg.triD_submap_pose_image_index_y
          + tem_pending_pub_traversable_area_data[DataType::KRefinePositive].header.pose_index_y;
        if (index_x >= 0 && index_x < tem_pending_pub_traversable_area_data[DataType::KRefinePositive].header.width
          && index_y >= 0 && index_y < tem_pending_pub_traversable_area_data[DataType::KRefinePositive].header.height){
          int tem_data_index = index_y * tem_pending_pub_traversable_area_data[DataType::KRefinePositive].header.width + index_x;
          if (tem_pending_pub_traversable_area_data[DataType::KRefinePositive].data.at(tem_data_index).possibility > 0.5){
            traversable_area_msg.cells[i * tem_width + j] = SendDataType::KRefineObstacle;
            // continue;
            // unsigned char* pdata = showdata + j * 3;
            // pdata[0] = 60;
            // pdata[1] = 60;
            // pdata[2] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[0] = 60;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[1] = 60;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[2] = 255;
            continue;
          }
          // else if(tem_pending_pub_traversable_area_data[DataType::KPositive].data[tem_data_index].possibility>0) {
          //     traversable_area_msg.cells[i * tem_width +j] =1;
          //     pdata = (unsigned char *)(showimage->imageData+(showimage->height-1-i)*showimage->widthStep +(j)*3 ) ;
          //     pdata[0] = 125;
          //     pdata[1] = 125;
          //     pdata[2] = 125;
          // }
        }
      }
      if (mapheader.index == tem_pending_pub_traversable_area_data[DataType::KPositiveSlope].header.index){
        int index_x = j - traversable_area_msg.triD_submap_pose_image_index_x
          + tem_pending_pub_traversable_area_data[DataType::KPositiveSlope].header.pose_index_x;
        int index_y = i - traversable_area_msg.triD_submap_pose_image_index_y
          + tem_pending_pub_traversable_area_data[DataType::KPositiveSlope].header.pose_index_y;
        if (index_x >= 0 && index_x < tem_pending_pub_traversable_area_data[DataType::KPositiveSlope].header.width
          && index_y >= 0 && index_y < tem_pending_pub_traversable_area_data[DataType::KPositiveSlope].header.height){
          int tem_data_index = index_y * tem_pending_pub_traversable_area_data[DataType::KPositiveSlope].header.width + index_x;

          if (tem_pending_pub_traversable_area_data[DataType::KPositiveSlope].data.at(tem_data_index).possibility > 0.5){

            traversable_area_msg.cells[i * tem_width + j] = SendDataType::KObstacle;
            // unsigned char* pdata = showdata + j * 3;
            // pdata[0] = 255;
            // // 	  pdata[1] = 0;
            // pdata[1] = 0;
            // pdata[2] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[0] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[1] = 0;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[2] = 255;
            continue;
          }
          // else if(tem_pending_pub_traversable_area_data[DataType::KPositiveSlope].data[tem_data_index].possibility>0) {
          //     traversable_area_msg.cells[i * tem_width +j] =1;
          //     pdata = (unsigned char *)(showimage->imageData+(showimage->height-1-i)*showimage->widthStep +(j)*3 ) ;
          //     pdata[0] = 125;
          //     pdata[1] = 125;
          //     pdata[2] = 125;
          // }
        }
      }
      if (mapheader.index == tem_pending_pub_traversable_area_data[DataType::KNegativeSlope].header.index){
        int index_x = j - traversable_area_msg.triD_submap_pose_image_index_x
          + tem_pending_pub_traversable_area_data[DataType::KNegativeSlope].header.pose_index_x;
        int index_y = i - traversable_area_msg.triD_submap_pose_image_index_y
          + tem_pending_pub_traversable_area_data[DataType::KNegativeSlope].header.pose_index_y;
        if (index_x >= 0 && index_x < tem_pending_pub_traversable_area_data[DataType::KNegativeSlope].header.width
          && index_y >= 0 && index_y < tem_pending_pub_traversable_area_data[DataType::KNegativeSlope].header.height){
          int tem_data_index = index_y * tem_pending_pub_traversable_area_data[DataType::KNegativeSlope].header.width + index_x;

          if (tem_pending_pub_traversable_area_data[DataType::KNegativeSlope].data.at(tem_data_index).possibility > 0.5){
            traversable_area_msg.cells[i * tem_width + j] = SendDataType::KObstacle;
            // unsigned char* pdata = showdata + j * 3;
            // pdata[0] = 125;
            // // 	  pdata[1] = 0;
            // pdata[1] = 0;
            // pdata[2] = 125;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[0] = 125;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[1] = 0;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[2] = 125;
            continue;
          }
          // else if(tem_pending_pub_traversable_area_data[DataType::KNegativeSlope].data[tem_data_index].possibility>0) {
          //     traversable_area_msg.cells[i * tem_width +j] =1;
          //     pdata = (unsigned char *)(showimage->imageData+(showimage->height-1-i)*showimage->widthStep +(j)*3 ) ;
          //     pdata[0] = 125;
          //     pdata[1] = 125;
          //     pdata[2] = 125;
          // }
        }
      }
      if (mapheader.index == tem_pending_pub_traversable_area_data[DataType::KUnevenArea].header.index){
        int index_x = j - traversable_area_msg.triD_submap_pose_image_index_x
          + tem_pending_pub_traversable_area_data[DataType::KUnevenArea].header.pose_index_x;
        int index_y = i - traversable_area_msg.triD_submap_pose_image_index_y
          + tem_pending_pub_traversable_area_data[DataType::KUnevenArea].header.pose_index_y;
        if (index_x >= 0 && index_x < tem_pending_pub_traversable_area_data[DataType::KUnevenArea].header.width
          && index_y >= 0 && index_y < tem_pending_pub_traversable_area_data[DataType::KUnevenArea].header.height){
          int tem_data_index = index_y * tem_pending_pub_traversable_area_data[DataType::KUnevenArea].header.width + index_x;

          if (tem_pending_pub_traversable_area_data[DataType::KUnevenArea].data.at(tem_data_index).possibility > 0.5){
            traversable_area_msg.cells[i * tem_width + j] = SendDataType::KObstacle;
            // unsigned char* pdata = showdata + j * 3;
            // pdata[0] = 255;
            // pdata[1] = 255;
            // pdata[2] = 0;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[0] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[1] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[2] = 0;
            continue;
          }
          // else if(tem_pending_pub_traversable_area_data[DataType::KUnevenArea].data[tem_data_index].possibility>0) {
          //     traversable_area_msg.cells[i * tem_width +j] =1;
          //     pdata = (unsigned char *)(showimage->imageData+(showimage->height-1-i)*showimage->widthStep +(j)*3 ) ;
          //     pdata[0] = 125;
          //     pdata[1] = 125;
          //     pdata[2] = 125;
          // }
        }
      }
      if (mapheader.index == tem_pending_pub_traversable_area_data[DataType::KStiff].header.index){
        int index_x = j - traversable_area_msg.triD_submap_pose_image_index_x
          + tem_pending_pub_traversable_area_data[DataType::KStiff].header.pose_index_x;
        int index_y = i - traversable_area_msg.triD_submap_pose_image_index_y
          + tem_pending_pub_traversable_area_data[DataType::KStiff].header.pose_index_y;
        if (index_x >= 0 && index_x < tem_pending_pub_traversable_area_data[DataType::KStiff].header.width
          && index_y >= 0 && index_y < tem_pending_pub_traversable_area_data[DataType::KStiff].header.height){
          int tem_data_index = index_y * tem_pending_pub_traversable_area_data[DataType::KStiff].header.width + index_x;

          if (tem_pending_pub_traversable_area_data[DataType::KStiff].data.at(tem_data_index).possibility > 0.5){
            traversable_area_msg.cells[i * tem_width + j] = SendDataType::KObstacle;
            // unsigned char* pdata = showdata + j * 3;
            // pdata[0] = 255;
            // pdata[1] = 0;
            // pdata[2] = 0;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[0] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[1] = 0;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[2] = 0;
            continue;
          }
          // else if(tem_pending_pub_traversable_area_data[DataType::KStiff].data[tem_data_index].possibility>0) {
          //     traversable_area_msg.cells[i * tem_width +j] =1;
          //     pdata = (unsigned char *)(showimage->imageData+(showimage->height-1-i)*showimage->widthStep +(j)*3 ) ;
          //     pdata[0] = 125;
          //     pdata[1] = 125;
          //     pdata[2] = 125;
          // }
        }
      }
      if (mapheader.index == tem_pending_pub_traversable_area_data[DataType::KWater].header.index){
        int index_x = j - traversable_area_msg.triD_submap_pose_image_index_x
          + tem_pending_pub_traversable_area_data[DataType::KWater].header.pose_index_x;
        int index_y = i - traversable_area_msg.triD_submap_pose_image_index_y
          + tem_pending_pub_traversable_area_data[DataType::KWater].header.pose_index_y;
        if (index_x >= 0 && index_x < tem_pending_pub_traversable_area_data[DataType::KWater].header.width
          && index_y >= 0 && index_y < tem_pending_pub_traversable_area_data[DataType::KWater].header.height){
          int tem_data_index = index_y * tem_pending_pub_traversable_area_data[DataType::KWater].header.width + index_x;

          if (tem_pending_pub_traversable_area_data[DataType::KWater].data.at(tem_data_index).possibility > 0.5){
            traversable_area_msg.cells[i * tem_width + j] = SendDataType::KWaterObstacle;//jkj
            // unsigned char* pdata = showdata + j * 3;
            // pdata[0] = 0;
            // pdata[1] = 255;
            // pdata[2] = 0;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[0] = 0;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[1] = 255;
            showimage.at<cv::Vec3b>(tem_height - 1 - i, j)[2] = 0;
            continue;
          }
          // else if(tem_pending_pub_traversable_area_data[DataType::KWater].data[tem_data_index].possibility>0) {
          //     traversable_area_msg.cells[i * tem_width +j] =1;
          //     pdata = (unsigned char *)(showimage->imageData+(showimage->height-1-i)*showimage->widthStep +(j)*3 ) ;
          //     pdata[0] = 125;
          //     pdata[1] = 125;
          //     pdata[2] = 125;
          // }
        }
      }
    }
  }

  /**************************************************** publish ****************************************************/

  if (traversable_area_option_.priormap_write){
    ::ivcommon::MutexLocker lock(&mutex_priormap_write);
    if (priormap_globalvalue.last_traversable_area_header.index < mapheader.index){
      PriorMapWriter();
      LOG(ERROR) << "traversable area saved onlinemap index: " << priormap_globalvalue.last_traversable_area_header.index;
    }
    priormap_globalvalue.last_traversable_area_msg = traversable_area_msg;
    priormap_globalvalue.last_traversable_area_header = tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header;
  }
  nav_msgs::Odometry vehicle_global_pose_;

  vehicle_global_pose_.header.stamp = ::ivcommon::ToRos(tem_latest_vechicle_pose.time);;
  vehicle_global_pose_.header.frame_id = global_frame_str;
  double tem_vehicle_latitude, tem_vehicle_longitude;
  ::ivcommon::transform::grid_to_geographic(a, e2, gps_zone, hemi, (tem_latest_vechicle_pose.pose.translation().y()), (tem_latest_vechicle_pose.pose.translation().x() + 500000),
    &tem_vehicle_latitude, &tem_vehicle_longitude);
  LOG(INFO) << "tem_latest_vechicle_pose" << tem_latest_vechicle_pose.pose << "tem_map_pose" << tem_map_pose;
  vehicle_global_pose_.pose.pose.position.x = tem_vehicle_longitude * 180 / M_PI;
  vehicle_global_pose_.pose.pose.position.y = tem_vehicle_latitude * 180 / M_PI;
  vehicle_global_pose_.pose.pose.position.z = tem_latest_vechicle_pose.pose.translation().z();
  vehicle_global_pose_.pose.pose.orientation.w = tem_latest_vechicle_pose.pose.rotation().w();
  vehicle_global_pose_.pose.pose.orientation.x = tem_latest_vechicle_pose.pose.rotation().x();
  vehicle_global_pose_.pose.pose.orientation.y = tem_latest_vechicle_pose.pose.rotation().y();
  vehicle_global_pose_.pose.pose.orientation.z = tem_latest_vechicle_pose.pose.rotation().z();
  publishers[KFinalTraversableAreaVehiclePoseTopicName].publish(vehicle_global_pose_);
  publishers[KFinalTraversableAreaTopicName].publish(traversable_area_msg);
  LOG(INFO) << "tem_vehicle_longitude*180/M_PI: " << tem_vehicle_longitude * 180 / M_PI <<
    "tem_vehicle_latitude*180/M_PI: " << tem_vehicle_latitude * 180 / M_PI;
  auto time_final = ros::Time::now();
  LOG(INFO) << KFinalTraversableAreaTopicName << " cost time: " << (time_final - traversable_area_msg.header.stamp).toSec();
  string filename = ::ivcommon::expand_user("~/traversable_area_extraction_time.txt");
  static std::ofstream file(filename);
  file << std::fixed << std::setprecision(4) << traversable_area_msg.header.stamp.toSec()
    << " " << (time_final - traversable_area_msg.header.stamp).toSec() << std::endl;

  TraversableAreaOptimization(traversable_area_msg, tem_latest_vechicle_pose.pose, KOptimizedFinalTraversableAreaTopicName, final_dilated_traversable_area_img);

  /**************************************************** visualizaiton ****************************************************/
  if (traversable_area_option_.display_on){
    double& resolution = traversable_area_msg.resolution;//0.2;
    int heightnum = tem_height / (10 / resolution);
    int widthnum = tem_width / (10 / resolution);
    for (int i = 0; i < heightnum;i++){
      cv::line(showimage, cv::Point(0, showimage.rows * i / heightnum),
        cv::Point(showimage.cols - 1, showimage.rows * i / heightnum), cvScalar(255, 0, 0));
    }
    for (int i = 1;i < widthnum;i++){
      cv::line(showimage, cv::Point(showimage.cols * i / widthnum, 0),
        cv::Point(showimage.cols * i / widthnum, showimage.rows - 1), cvScalar(255, 0, 0));
    }
    // cv::rectangle(showimage,cv::Point(tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.pose_index_x-1/resolution,
    //     showimage.rows-1-tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.pose_index_y+2/resolution),
    // cv::Point(tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.pose_index_x+1/resolution,
    //     showimage.rows-1-tem_pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.pose_index_y-2/resolution),
    //     cvScalar(0,0,255));
    ::ivcommon::transform::Rigid3d tem_relative_pose = tem_map_pose.inverse() * tem_latest_vechicle_pose.pose;
    cv::circle(showimage, cv::Point(tem_relative_pose.translation().x() / resolution
      + traversable_area_msg.triD_submap_pose_image_index_x,
      showimage.rows - 1 - (tem_relative_pose.translation().y() / resolution
        + traversable_area_msg.triD_submap_pose_image_index_y)), 5,
      cv::Scalar(0, 255, 255), -1);
    while (!final_traversable_area_img_mtx.try_lock_for(std::chrono::milliseconds(2))) {
      final_traversable_area_img = std::move(showimage);
      final_traversable_area_img_mtx.unlock();
    }
    // cv::imshow(KFinalTraversableAreaTopicName, showimage);
    // cv::waitKey(1);
  }
  traversable_area_finished = true;
  // if (!traversable_area_data_img.empty()) {
  //     cv::namedWindow(DataTypeString[DataType::KPositive], 0);
  //     cv::imshow(DataTypeString[DataType::KPositive], show_obstacle_img);
  //     cv::namedWindow("traversable_area_data_img", 0);
  //     cv::imshow("traversable_area_data_img", traversable_area_data_img);
  //     cv::waitKey(5);
  // }
  // LOG(WARNING)<<"vehicle_global_pose_ and traversable_area_msg_ published";
  // clock_t end_time = clock();
  // ROS_INFO("PublishFinalTraversableArea time cost: %f ms", (double)(end_time - start_time) / CLOCKS_PER_SEC * 1000.0);
}

void Node::traversable_area_optimization(cv::Mat& map, float resolution,
  float vehicle_width_, SendDataType senddatatype){
  struct MaxVal{
    MaxVal() :
      distancesquare(-1){
    }
    int distancesquare;
    cv::Point point;
  };
  //膨胀系数是根据车宽一半膨胀的
  float vehicle_width_half_cell = vehicle_width_ / resolution / 2;
  int radius = std::ceil(vehicle_width_half_cell);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(radius + 1, radius + 1));
  cv::Mat dilated_map;
  cv::dilate(map, dilated_map, element);
  std::vector<MaxVal> indexs;
  int square[radius * 2 + 1];
  int* square_signed = &square[radius];
  for (int i = -radius; i <= radius; i++){
    square_signed[i] = i * i;
  }
  int radiussquare = std::ceil(vehicle_width_half_cell * vehicle_width_half_cell);

  for (int window_j = 0; window_j <= radius; window_j++){
    for (int window_i = 0; window_i <= radius; window_i++){
      double dis = square_signed[window_i] + square_signed[window_j];
      if (dis <= radiussquare){
        MaxVal val;
        val.point = cv::Point(window_i, window_j);
        val.distancesquare = dis;
        indexs.push_back(val);
      }
    }
  }

  std::sort(indexs.begin(), indexs.end(), [](MaxVal a, MaxVal b){
    return a.distancesquare > b.distancesquare;
    });

  cv::Mat tempmap = map.clone();
  for (int j = radius; j < map.rows - radius; j += 1){
    for (int i = radius; i < map.cols - radius; i += 1){
      if (map.at<unsigned char>(j, i) == senddatatype
        || dilated_map.at<unsigned char>(j, i) != senddatatype)
        continue;

      std::vector<cv::Point> points;

      for (auto index : indexs){
        int map_j = j - index.point.y;
        int map_i = i - index.point.x;
        if (tempmap.at<unsigned char>(map_j, map_i) == senddatatype){
          points.emplace_back(map_i, map_j);
          break;
        }
      }

      for (auto index : indexs){
        if (index.point.x == 0)
          continue;
        int map_j = j + index.point.y;
        int map_i = i - index.point.x;
        if (tempmap.at<unsigned char>(map_j, map_i) == senddatatype){
          points.emplace_back(map_i, map_j);
          break;
        }
      }

      for (auto index : indexs){
        if (index.point.x == 0 || index.point.y == 0)
          continue;
        int map_j = j + index.point.y;
        int map_i = i + index.point.x;
        if (tempmap.at<unsigned char>(map_j, map_i) == senddatatype){
          points.emplace_back(map_i, map_j);
          break;
        }
      }

      for (auto index : indexs){
        if (index.point.y == 0)
          continue;
        int map_j = j - index.point.y;
        int map_i = i + index.point.x;
        if (tempmap.at<unsigned char>(map_j, map_i) == senddatatype){
          points.emplace_back(map_i, map_j);
          break;
        }
      }

      int num = points.size();
      if (num > 1){
        //				LOG(INFO)<<"num:"<<num;
        const cv::Point* ppt[1] = { points.data() };
        int npt[] = { num };
        cv::fillPoly(map, ppt, npt, 1, cv::Scalar(senddatatype));
      }
    }
  }
}

void Node::known_area_extraction(cv::Mat& srcimage, cv::Point center, int radius){
  const int kgap_num_threshold = 5;

  cv::Mat knownimg = cv::Mat::zeros(cv::Size(radius * 2 + 1, radius * 2 + 1), srcimage.type());
  cv::Point knownimg_center(radius, radius);
  std::vector<cv::Point> circle_points;
  cv::ellipse2Poly(knownimg_center, cv::Size(radius, radius), 0, 0, 360, 1, circle_points);
  for (auto it = circle_points.begin();it != circle_points.end();it++){
    cv::LineIterator lit(srcimage, center, *it + center - knownimg_center);
    cv::LineIterator litknown(knownimg, knownimg_center, *it);
    bool last_known = true;
    int gap_num = 0;
    for (int i = 0;i < lit.count;i++, litknown++, lit++){
      const auto& srcdata = *(*lit);
      auto& data = *(*litknown);
      if (last_known){
        if (srcdata == SendDataType::KObstacle){
          last_known = false;
          gap_num = 0;
        }
        else
          data = SendDataType::KPassibility;
      }
      else{
        if (gap_num >= kgap_num_threshold && srcdata == SendDataType::KPassibility){
          last_known = true;
          gap_num = 0;
        }
        else if (srcdata == SendDataType::KObstacle)
          gap_num = 0;
        else{
          gap_num++;
          data = SendDataType::KNearUnknown;
        }
      }

    }
  }
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(knownimg, knownimg, element);
  for (int j = 0;j < knownimg.rows;j++){
    int src_j = j - knownimg_center.y + center.y;
    if (src_j < 0 || src_j >= srcimage.rows)
      continue;
    unsigned char* pdata = knownimg.ptr<unsigned char>(j);
    unsigned char* srcdata = srcimage.ptr<unsigned char>(src_j);
    for (int i = 0;i < knownimg.cols;i++){
      int src_i = i - knownimg_center.x + center.x;
      if (src_i < 0 || src_i >= srcimage.cols)
        continue;

      srcdata[src_i] = std::max(srcdata[src_i], pdata[i]);
    }
  }

}


void  Node::TraversableAreaOptimization(const iv_slam_ros_msgs::TraversableArea& received_traversable_area
  , ivcommon::transform::Rigid3d global_vehicle_pose
  , const std::string traversable_area_topic_name
  , cv::Mat& traversable_area_img)//jkj
{
  double pose_x = 0, pose_y = 0;
  ::ivcommon::transform::geographic_to_grid(a, e2,
    (received_traversable_area.triD_submap_pose.position.y) * M_PI / 180,
    (received_traversable_area.triD_submap_pose.position.x) * M_PI / 180,
    &gps_zone, &hemi, &(pose_y), &(pose_x));

  ::ivcommon::transform::Rigid3d TwiDTraversableArea_pose = ::ivcommon::transform::Rigid3d(
    Eigen::Vector3d(pose_x - 500000,
      pose_y,
      received_traversable_area.triD_submap_pose.position.z),
    Eigen::Quaternion<double>(
      received_traversable_area.triD_submap_pose.orientation.w,
      received_traversable_area.triD_submap_pose.orientation.x,
      received_traversable_area.triD_submap_pose.orientation.y,
      received_traversable_area.triD_submap_pose.orientation.z));

  Eigen::Vector3d submap_eular_pose = ::ivcommon::transform::toRollPitchYaw(
    TwiDTraversableArea_pose.rotation());
  Eigen::Vector3d global_vehicle_eular_pose = ::ivcommon::transform::toRollPitchYaw(
    global_vehicle_pose.rotation());
  global_vehicle_pose = ::ivcommon::transform::Rigid3d(
    global_vehicle_pose.translation(),
    ::ivcommon::transform::RollPitchYaw(submap_eular_pose.x(),
    submap_eular_pose.y(), global_vehicle_eular_pose.z()));
  cv::Mat src_image = cv::Mat::zeros(received_traversable_area.height,
    received_traversable_area.width, CV_8U);
  cv::Mat traversable_area_optimized_image = src_image.clone();
  cv::Mat traversable_area_optimized_image_refine = src_image.clone();
  cv::Mat traversable_area_optimized_image_water = src_image.clone();
  int data_index = 0;

  int x_index2, y_index2;

  // double start_time = ros::Time::now().toSec();
  // 栅格图都是左下角为起始点的，而图像默认都是左上角为起始点
  for (int j = 0; j < traversable_area_optimized_image.rows; j++){
    int index_j = traversable_area_optimized_image.rows - 1 - j;//从下往上
    uchar* srcdata = src_image.ptr<uchar>(index_j);
    uchar* testdata = traversable_area_optimized_image.ptr<uchar>(index_j);
    uchar* refinedata = traversable_area_optimized_image_refine.ptr<uchar>(index_j);
    uchar* waterdata = traversable_area_optimized_image_water.ptr<uchar>(index_j);
    for (int i = 0; i < traversable_area_optimized_image.cols; i++){
      const auto& val = received_traversable_area.cells.at(data_index);
      srcdata[i] = val;
      if (val == SendDataType::KObstacle){
        testdata[i] = val;
      }
      else if (val == SendDataType::KRefineObstacle)
        refinedata[i] = val;
      else if (val == SendDataType::KWaterObstacle)
        waterdata[i] = val;
      // if ((received_traversable_area.cells.at(data_index) == 2)) {
      // 	testdata[i] = 255;
      // } else if (received_traversable_area.cells.at(data_index) == 1) {
      // 	testdata[i] = 50;
      // }
      data_index++;
    }
  }

  auto vehicle_pose = TwiDTraversableArea_pose.inverse()
    * global_vehicle_pose.translation();//将车辆位姿转换到子地图位姿上

  cv::Point vehicle_position_in_image(
    received_traversable_area.triD_submap_pose_image_index_x
    + vehicle_pose.x()
    / received_traversable_area.resolution,
    traversable_area_optimized_image.rows - 1
    - (received_traversable_area.triD_submap_pose_image_index_y
      + vehicle_pose.y()
      / received_traversable_area.resolution));


  traversable_area_optimization(traversable_area_optimized_image_refine,
    received_traversable_area.resolution, traversable_area_option_.vehicle_width, SendDataType::KRefineObstacle);//没看懂这是啥意思zhubc
  traversable_area_optimization(traversable_area_optimized_image,
    received_traversable_area.resolution, traversable_area_option_.vehicle_width, SendDataType::KObstacle);
  traversable_area_optimization(traversable_area_optimized_image_water,
    received_traversable_area.resolution, traversable_area_option_.vehicle_width, SendDataType::KWaterObstacle);
  // known_area_extraction(traversable_area_optimized_image,vehicle_position_in_image
  //         ,traversable_area_option_.known_radius/received_traversable_area.resolution);

  /*******************************************publish*************************************************/
  iv_slam_ros_msgs::TraversableArea traversable_area_optimized = received_traversable_area;
  data_index = 0;
  for (int j = 0; j < traversable_area_optimized_image.rows; j++){
    int index_j = traversable_area_optimized_image.rows - 1 - j;
    uchar* testdata = traversable_area_optimized_image.ptr<uchar>(index_j);
    uchar* refinedata = traversable_area_optimized_image_refine.ptr<uchar>(index_j);
    uchar* waterdata = traversable_area_optimized_image_water.ptr<uchar>(index_j);
    for (int i = 0; i < traversable_area_optimized_image.cols; i++){
      if (testdata[i] == SendDataType::KObstacle)
        traversable_area_optimized.cells.at(data_index) = SendDataType::KObstacle;
      else if (refinedata[i] == SendDataType::KRefineObstacle)
        traversable_area_optimized.cells.at(data_index) = SendDataType::KRefineObstacle;
      else if (waterdata[i] == SendDataType::KWaterObstacle)
        traversable_area_optimized.cells.at(data_index) = SendDataType::KWaterObstacle;
      data_index++;
    }
  }
  ros::Time time_final = ros::Time::now();
  publishers[traversable_area_topic_name].publish(traversable_area_optimized);
  LOG(INFO) << traversable_area_topic_name << " cost time-total:" << (time_final - traversable_area_optimized.header.stamp).toSec();
  string filename = ::ivcommon::expand_user("~/traversable_area_optimization_time.txt");
  static std::ofstream file(filename);
  file << std::fixed << std::setprecision(4) << traversable_area_optimized.header.stamp.toSec()
    << " " << (time_final - traversable_area_optimized.header.stamp).toSec() << std::endl;
  /*******************************************display*************************************************/
  if (traversable_area_option_.display_on) {

    cv::Mat tmp_img = cv::Mat::zeros(
      cv::Size(traversable_area_optimized_image.cols, traversable_area_optimized_image.rows), CV_8UC3);

    for (int j = 0; j < tmp_img.rows; j++){
      unsigned char* pdata = traversable_area_optimized_image.ptr<unsigned char>(j);
      unsigned char* refinedata = traversable_area_optimized_image_refine.ptr<unsigned char>(j);
      unsigned char* waterdata = traversable_area_optimized_image_water.ptr<unsigned char>(j);
      unsigned char* srcdata = src_image.ptr<unsigned char>(j);
      // unsigned char* display_data = (unsigned char*)tmp_img.ptr<uchar>(j);
      for (int i = 0; i < tmp_img.cols; i++){
        uchar pixel;
        //			if (srcdata[i] == SendDataType::KUnknown)
        //			{
        //
        //				display_data[3 * i] = 0;
        //				display_data[3 * i + 1] = 0;
        //				display_data[3 * i + 2] = 0;
        //			}
        if (srcdata[i] == 99){
          // display_data[3 * i] = 0;
          // display_data[3 * i + 1] = 0;
          // display_data[3 * i + 2] = 255;
          tmp_img.at<cv::Vec3b>(j,i)[0] = 0;
          tmp_img.at<cv::Vec3b>(j,i)[1] = 0;
          tmp_img.at<cv::Vec3b>(j,i)[2] = 255;
        }
        else if (pdata[i] == SendDataType::KObstacle){
          // display_data[3 * i] = 255;
          // display_data[3 * i + 1] = 255;
          // display_data[3 * i + 2] = 255;
          tmp_img.at<cv::Vec3b>(j,i)[0] = 255;
          tmp_img.at<cv::Vec3b>(j,i)[1] = 255;
          tmp_img.at<cv::Vec3b>(j,i)[2] = 255;
        }
        else if (refinedata[i] == SendDataType::KRefineObstacle){
          // display_data[3 * i] = 60;
          // display_data[3 * i + 1] = 60;
          // display_data[3 * i + 2] = 255;
          tmp_img.at<cv::Vec3b>(j,i)[0] = 60;
          tmp_img.at<cv::Vec3b>(j,i)[1] = 60;
          tmp_img.at<cv::Vec3b>(j,i)[2] = 255;
        }
        else if (waterdata[i] == SendDataType::KWaterObstacle){
          // display_data[3 * i] = 255;
          // display_data[3 * i + 1] = 60;
          // display_data[3 * i + 2] = 60;
          tmp_img.at<cv::Vec3b>(j,i)[0] = 255;
          tmp_img.at<cv::Vec3b>(j,i)[1] = 60;
          tmp_img.at<cv::Vec3b>(j,i)[2] = 60;
        }
        else if (srcdata[i] == SendDataType::KNearUnknown){
          // display_data[3 * i] = 200;
          // display_data[3 * i + 1] = 200;
          // display_data[3 * i + 2] = 200;
          tmp_img.at<cv::Vec3b>(j,i)[0] = 200;
          tmp_img.at<cv::Vec3b>(j,i)[1] = 200;
          tmp_img.at<cv::Vec3b>(j,i)[2] = 200;
        }
        else if (srcdata[i] == SendDataType::KPassibility){
          // display_data[3 * i] = 100;
          // display_data[3 * i + 1] = 100;
          // display_data[3 * i + 2] = 100;
          tmp_img.at<cv::Vec3b>(j,i)[0] = 100;
          tmp_img.at<cv::Vec3b>(j,i)[1] = 100;
          tmp_img.at<cv::Vec3b>(j,i)[2] = 100;
        }
        //			else
        //			{
        //
        //				display_data[3 * i] = 0;
        //				display_data[3 * i + 1] = 255;
        //				display_data[3 * i + 2] = 0;
        //			}
        // if(*pdata == 254)

      }
    }

    int linestep = 10 / received_traversable_area.resolution;
    int heightnum = tmp_img.rows / (linestep);
    int widthnum = tmp_img.cols / (linestep);
    for (int i = 0; i < heightnum; i++){
      cv::line(tmp_img, cv::Point(0, linestep * i),
        cv::Point(tmp_img.cols - 1, linestep * i),
        cv::Scalar(255, 0, 0));
    }

    for (int i = 1; i < widthnum; i++){
      cv::line(tmp_img, cv::Point(linestep * i, 0),
        cv::Point(linestep * i, tmp_img.rows - 1),
        cv::Scalar(255, 0, 0));
    }
    cv::circle(tmp_img, vehicle_position_in_image,
      5, cv::Scalar(255, 0, 0), -1);
    if (traversable_area_topic_name == KOptimizedFinalTraversableAreaTopicName) {
      while (!final_dilated_traversable_area_img_mtx.try_lock_for(std::chrono::milliseconds(2))) {
        traversable_area_img = std::move(tmp_img);
        final_dilated_traversable_area_img_mtx.unlock();
      }
    }
    else {
      while (!single_dilated_traversable_area_img_mtx.try_lock_for(std::chrono::milliseconds(2))) {
        traversable_area_img = std::move(tmp_img);
        single_dilated_traversable_area_img_mtx.unlock();
      }
    }
    // if (traversable_area_option_.display_on) {
    //   if (traversable_area_img.data != nullptr)
    //     cv::imshow(traversable_area_topic_name, traversable_area_img);
    //   cv::waitKey(1);
    // }
  }
}

void Node::ConvertToNowCoordinationMat(cv::Mat& area_img, const DataType datatype_, const MapHeader& mapheader){
  LidarOdometryData tem_latest_vechicle_pose = latest_vechicle_pose;
  if (traversable_area_data[datatype_].header.time == ::ivcommon::Time::min())
    return;
  if (traversable_area_data[DataType::KPrimaryTraversableArea].header.time != ::ivcommon::Time::min())
    //上一帧准可通行区域转到当前帧
  {
    const auto& traversable_area_last =
      traversable_area_data[DataType::KPrimaryTraversableArea];//上一帧准可通行区域
    ::ivcommon::transform::Rigid3d tem_reference_pose = mapheader.pose.inverse()
      * traversable_area_last.header.pose;//上一帧转当前帧
    ::ivcommon::transform::Rigid3d tem_vehicle_reference_pose = tem_latest_vechicle_pose.pose.inverse()
      * traversable_area_last.header.pose;//上一帧里程计相对于车体的位姿变化
    for (int i = 0; i < traversable_area_last.header.height; i++){
      for (int j = 0; j < traversable_area_last.header.width; j++){
        double tem_value = traversable_area_last.data[i
          * traversable_area_last.header.width + j].possibility;
        if (tem_value > 0){
          double tem_y = (i - traversable_area_last.header.pose_index_y)
            * traversable_area_last.header.resolution;//单位应该是m
          double tem_x = (j - traversable_area_last.header.pose_index_x)
            * traversable_area_last.header.resolution;
          Eigen::Vector3d tem_pose(tem_x, tem_y, 0);
          Eigen::Vector3d tem_vehicle_relative_pose = tem_vehicle_reference_pose
            * tem_pose;//车体坐标系
          if ((tem_vehicle_relative_pose.x() < -2 || tem_vehicle_relative_pose.x() > 2
            || tem_vehicle_relative_pose.y() > 12) && tem_vehicle_relative_pose.head(2).norm() > 10)
            continue;
          Eigen::Vector3d tem_relative_pose = tem_reference_pose
            * tem_pose;//将上一帧的位置转换到当前帧准可通行区域的位置
          int tem_index_x = (tem_relative_pose.x()
            / mapheader.resolution + mapheader.pose_index_x);
          int tem_index_y = (tem_relative_pose.y()
            / mapheader.resolution + mapheader.pose_index_y);
          if (tem_index_x >= 0 && tem_index_x < mapheader.width
            && tem_index_y >= 0
            && tem_index_y < mapheader.height){
            if (tem_value < 0.4){
              area_img.at<uchar>(tem_index_y, tem_index_x) = 1;
            }
          }
        }
      }
    }
  }

  if (traversable_area_data[datatype_].header.time
    != ::ivcommon::Time::min()) //上一帧障碍物地图转到当前帧准可通行区域坐标系中
  {
    const auto& traversable_area_last =
      traversable_area_data[datatype_];
    ::ivcommon::transform::Rigid3d tem_reference_pose = mapheader.pose.inverse()
      * traversable_area_last.header.pose;
    ::ivcommon::transform::Rigid3d tem_vehicle_reference_pose = tem_latest_vechicle_pose.pose.inverse()
      * traversable_area_last.header.pose;
    for (int i = 0; i < traversable_area_last.header.height; i++){
      for (int j = 0; j < traversable_area_last.header.width; j++){
        double tem_value = traversable_area_last.data[i
          * traversable_area_last.header.width + j].possibility;
        if (tem_value > 0){
          double tem_y = (i
            - traversable_area_last.header.pose_index_y)
            * traversable_area_last.header.resolution;
          double tem_x = (j
            - traversable_area_last.header.pose_index_x)
            * traversable_area_last.header.resolution;
          Eigen::Vector3d tem_pose(tem_x, tem_y, 0);
          Eigen::Vector3d tem_vehicle_relative_pose = tem_vehicle_reference_pose
            * tem_pose;
          if ((tem_vehicle_relative_pose.x() < -2 || tem_vehicle_relative_pose.x() > 2
            || tem_vehicle_relative_pose.y() > 12) && tem_vehicle_relative_pose.head(2).norm() > 10)
            continue;
          Eigen::Vector3d tem_relative_pose = tem_reference_pose
            * tem_pose;
          int tem_index_x = (tem_relative_pose.x()
            / mapheader.resolution + mapheader.pose_index_x);
          int tem_index_y = (tem_relative_pose.y()
            / mapheader.resolution + mapheader.pose_index_y);
          if (tem_index_x >= 0 && tem_index_x < mapheader.width
            && tem_index_y >= 0
            && tem_index_y < mapheader.height){
            if (tem_value > 0.5){
              area_img.at<uchar>(tem_index_y, tem_index_x) = 2;
            }
            else if (tem_value < 0.4){
              area_img.at<uchar>(tem_index_y, tem_index_x) = 1;
            }
            else{
              area_img.at<uchar>(tem_index_y, tem_index_x) = 0;
            }

          }
        }
      }
    }
  }
}

void Node::MapDataCovertToTraversableArea(const MapData& mapdata, TraversableAreaData& tem_TraversableAreaData)//jkj 0728
{
  //这个mapdata是当前帧准可通行区域的内容
  const auto& mapheader = mapdata.header;
  LidarOdometryData tem_latest_vechicle_pose = latest_vechicle_pose;//车辆位姿
  cv::Mat area_img = cv::Mat::zeros(mapheader.height, mapheader.width, CV_8U);//初始化的值是0

  if (traversable_area_data[DataType::KPrimaryTraversableArea].header.time
    != ::ivcommon::Time::min())//如果不是第一帧数据
  {
    const auto& traversable_area_last =
      traversable_area_data[DataType::KPrimaryTraversableArea];//上一帧准可通行区域
    ::ivcommon::transform::Rigid3d tem_reference_pose = mapheader.pose.inverse()
      * traversable_area_last.header.pose;//将上一帧准可通行区域的位姿转成当前帧坐标系下
    ::ivcommon::transform::Rigid3d tem_vehicle_reference_pose = tem_latest_vechicle_pose.pose.inverse()
      * traversable_area_last.header.pose;//将上一帧准可通行区域的local pose转成车体坐标系下
// 把之前的准可通行区域数据变换到当前坐标系，然后存储在 area_img 中
    for (int i = 0; i < traversable_area_last.header.height; i++){
      for (int j = 0; j < traversable_area_last.header.width; j++){
        double tem_value = traversable_area_last.data[i * traversable_area_last.header.width + j].possibility;
        if (tem_value > 0){
          double tem_y = (i - traversable_area_last.header.pose_index_y) * traversable_area_last.header.resolution;
          double tem_x = (j - traversable_area_last.header.pose_index_x) * traversable_area_last.header.resolution;
          Eigen::Vector3d tem_pose(tem_x, tem_y, 0);//转成m单位
          Eigen::Vector3d tem_vehicle_relative_pose = tem_vehicle_reference_pose
            * tem_pose;//转成准可通行区域车体坐标系下
// ? 为什么对于10米距离外且 abs(x)>2、y>10 的栅格不处理
          if ((tem_vehicle_relative_pose.x() < -2 || tem_vehicle_relative_pose.x() > 2
            || tem_vehicle_relative_pose.y() > 10) && tem_vehicle_relative_pose.head(2).norm() > 10)
            continue;//这里是10？而非12？zhubc
          Eigen::Vector3d tem_relative_pose = tem_reference_pose * tem_pose;//转成当前帧子图坐标系下，单位是m
          int tem_index_x = (tem_relative_pose.x()
            / mapheader.resolution + mapheader.pose_index_x);//转成栅格
          int tem_index_y = (tem_relative_pose.y()
            / mapheader.resolution + mapheader.pose_index_y);
          if (tem_index_x >= 0 && tem_index_x < mapheader.width
            && tem_index_y >= 0
            && tem_index_y < mapheader.height){
            if (tem_value > 0.5){
              area_img.at<uchar>(tem_index_y, tem_index_x) = 2;
            }
            else if (tem_value < 0.4){
              area_img.at<uchar>(tem_index_y, tem_index_x) = 1;
            }
            else{
              area_img.at<uchar>(tem_index_y, tem_index_x) = 0;
            }

          }
        }
      }
    }
  }
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat area_img_dilated;
  cv::dilate(area_img, area_img_dilated, element);

  tem_TraversableAreaData.header = mapheader;

  tem_TraversableAreaData.data.assign(mapheader.width * mapheader.height, CellData());
  //当前帧和上一帧的区别就在于值是2或1还是0。理论上，如果是可通行则为0.4，如果是不可通行或未知则为0.6。
  //因此，如果当前帧状态为可通行，如果是12米外的话，更新概率为0.4,{如果是12米内，则不更新概率}?。
  //感觉还是有问题1217zhubc
  //只是针对于准可通行区域的结果进行的处理
  for (int i = 0; i < mapdata.header.height; i++){
    for (int j = 0; j < mapdata.header.width; j++){
      int tem_data = mapdata.data.at(i * mapdata.header.width + j);
      //如果是可通行，则概率给0.4,观测值为当前值，观测值和概率是一一对应的
      if (tem_data == 1){
        tem_TraversableAreaData.data[i * mapheader.width + j].possibility = KNullProbability;

        tem_TraversableAreaData.data[i * mapheader.width + j].observation_num -= 0;
      }
      //如果当前为障碍物，且上一帧膨胀之后的值不是可通行，则概率为0.6，观测值为1
      // else if (tem_data == 2 && area_img_dilated.at<uchar>(i, j) != 1){
      else if (tem_data > 1 && area_img_dilated.at<uchar>(i, j) != 1){
        tem_TraversableAreaData.data[i * mapheader.width + j].possibility = KHitProbability;
        tem_TraversableAreaData.data[i * mapheader.width + j].observation_num = 1;
      }
      else if (area_img.at<uchar>(i, j) == 1)
        //如果上一帧为可通行，则不管当前帧状态是啥，概率都给0.4，观测值为0，不管当前帧的状态不就是概率不更新的意思嘛
      {
        tem_TraversableAreaData.data[i * mapheader.width + j].possibility = KNullProbability;
        tem_TraversableAreaData.data[i * mapheader.width + j].observation_num = 0;
      }

    }
  }
}


void Node::TraversableAreaProcessor(const MapData& mapdata){
  if (TraversableAreaProcessor_running) return;
  bool index_changed = false; // 准可通行区域所在的子地图编号是否发生切换
  ros::Time time1 = ros::Time::now();
  // mapdata是当前帧的准可通行区域的数据
  {
    TraversableAreaProcessor_running = true;
    ::ivcommon::MutexLocker lock_eachtype(&mutexs_eachtype[DataType::KPrimaryTraversableArea]);
    // traversable_area_data[DataType::KPrimaryTraversableArea].header.index 初始值为 -1
    CHECK_GE(mapdata.header.index, traversable_area_data[DataType::KPrimaryTraversableArea].header.index);
    if (traversable_area_inited == false){
      // traversable_area_inited = false;
      TraversableAreaData tem_TraversableAreaData;
      // 融合 traversable_area_data[DataType::KPrimaryTraversableArea] 和 mapdata 来生成 tem_TraversableAreaData
      MapDataCovertToTraversableArea(mapdata, tem_TraversableAreaData);
      // 用 tem_TraversableAreaData 更新 traversable_area_data[DataType::KPrimaryTraversableArea]
      traversable_area_data[DataType::KPrimaryTraversableArea].header = tem_TraversableAreaData.header;
      priormap_globalvalue.last_traversable_area_header = tem_TraversableAreaData.header;//lzz
      traversable_area_data[DataType::KPrimaryTraversableArea].data.swap(tem_TraversableAreaData.data);
      // global_origin_pose = mapdata.header.pose;
      // traversable_area_data[DataType::KPrimaryTraversableArea].header.pose
      //         = /*global_origin_pose.inverse()**/ mapdata.header.pose;
      // traversable_area_data[DataType::KPrimaryTraversableArea].header.location_module_pose
      //         = /*global_origin_pose.inverse()**/ mapdata.header.location_module_pose;

      if (map_init_time == ::ivcommon::Time::min()){
        map_init_time = mapdata.header.time;
      }
      LOG(WARNING) << "traversable_area_data inited!" << " traversable_area_data[DataType::KPrimaryTraversableArea].index:" << traversable_area_data[DataType::KPrimaryTraversableArea].header.index << '\t' <<
        "traversable_area_data[DataType::KPrimaryTraversableArea].time:" << traversable_area_data[DataType::KPrimaryTraversableArea].header.time << '\t' << "height:" << traversable_area_data[DataType::KPrimaryTraversableArea].header.height << '\t' <<
        "width:" << traversable_area_data[DataType::KPrimaryTraversableArea].header.width;
      LOG(INFO) << "global_origin_pose" << global_origin_pose;
      {
        ::ivcommon::MutexLocker lock(&mutex_pending_pub_data);
        for (int i = 0;i < kObstacletypes;i++){
          pending_pub_traversable_area_data[i] = traversable_area_data[i];
        }

      }
      traversable_area_inited = true;
      TraversableAreaProcessor_running = false;
      return;
    }
    if (mapdata.header.index != traversable_area_data[DataType::KPrimaryTraversableArea].header.index){
      index_changed = true;
    }
    TraversableAreaData tem_TraversableAreaData;
    MapDataCovertToTraversableArea(mapdata, tem_TraversableAreaData);
    traversable_area_data[DataType::KPrimaryTraversableArea].header = tem_TraversableAreaData.header;
    traversable_area_data[DataType::KPrimaryTraversableArea].data.swap(tem_TraversableAreaData.data);

    // CHECK(traversable_area_data[DataType::KPrimaryTraversableArea].header.index == mapdata.header.index);

     // 子地图没有切换，直接用 traversable_area_data 覆盖 pending_pub_traversable_area_data
    if (!index_changed){
      ::ivcommon::MutexLocker lock_pending_pub_data(&mutex_pending_pub_data);
      pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea]
        = traversable_area_data[DataType::KPrimaryTraversableArea];
    }
  }
  // CHECK(traversable_area_data[DataType::KPrimaryTraversableArea].header.index == mapdata.header.index);

  if (index_changed){
    // auto time1 = ros::Time::now();
    TraversableAreaData temp_pending_pub_traversable_area_data[kObstacletypes];
    for (int i = 0;i < kObstacletypes;i++){
      ::ivcommon::MutexLocker lock_eachtype(&mutexs_eachtype[i]);
      if (traversable_area_data[i].header.time > ::ivcommon::Time::min()){
        if (i != DataType::KPrimaryTraversableArea){
          CommonObjectSwitcher(DataType(i), mapdata.header);
        }
        temp_pending_pub_traversable_area_data[i] = traversable_area_data[i];
      }
    }

    ::ivcommon::MutexLocker lock(&mutex_pending_pub_data);
    for (int i = 0;i < kObstacletypes;i++){
      if (temp_pending_pub_traversable_area_data[i].header.time > ::ivcommon::Time::min()){
        pending_pub_traversable_area_data[i].header = temp_pending_pub_traversable_area_data[i].header;
        pending_pub_traversable_area_data[i].data.swap(temp_pending_pub_traversable_area_data[i].data);
        CHECK_GE(pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.index
          , pending_pub_traversable_area_data[i].header.index);
      }
    }

    LOG(INFO) << "CommonObjectSwitcher cost time:" << (ros::Time::now() - time1).toSec();
  }


  LOG(INFO) << "TraversableAreaProcessor cost time:" << (ros::Time::now() - time1).toSec();
  TraversableAreaProcessor_running = false;
  // PublishFinalTraversableArea();
}

void Node::CommonObjectSwitcher(DataType datatype_, const MapHeader& mapheader){
  // mapheader 是当前帧准可通行区域的 header
  thread_num++;
  const float negative_dead_zone_radius = 8.;
  CHECK(!map_size_switching[datatype_]);
  LidarOdometryData tem_latest_vechicle_pose = latest_vechicle_pose;
  ::ivcommon::transform::Rigid3d vehicle_pose = mapheader.pose.inverse() * tem_latest_vechicle_pose.pose;//基于当前帧准可通行区域坐标系的车辆位置
  ros::Time time1 = ros::Time::now();
  map_size_switching[datatype_] = true;
  TraversableAreaData tem_common_traversable_area_data;//当前帧的某种障碍物地图
  tem_common_traversable_area_data.header = traversable_area_data[datatype_].header;
  tem_common_traversable_area_data.data.swap(traversable_area_data[datatype_].data);
  traversable_area_data[datatype_].header = mapheader;//这才是更新的地方，将当前帧的位姿更新

  traversable_area_data[datatype_].data.assign(mapheader.width * mapheader.height, CellData());//对值进行初始化

  CHECK(!tem_common_traversable_area_data.data.empty());
  // 障碍物地图基于当前帧子图坐标系的相对位姿
  ::ivcommon::transform::Rigid3d tem_reference_pose = mapheader.pose.inverse() * tem_common_traversable_area_data.header.pose;
  // 对上一帧 traversable_area_data[datatype_] 地图数据进行遍历
  for (int i = 0; i < tem_common_traversable_area_data.header.height;i++){
    for (int j = 0; j < tem_common_traversable_area_data.header.width; j++){
      // 上一帧 traversable_area_data[datatype_] 地图栅格中的占据概率值
      double tem_value = tem_common_traversable_area_data.data[i * tem_common_traversable_area_data.header.width + j].possibility;
      if (tem_value <= 0) continue;
      double tem_y = (i - tem_common_traversable_area_data.header.pose_index_y) * tem_common_traversable_area_data.header.resolution;
      double tem_x = (j - tem_common_traversable_area_data.header.pose_index_x) * tem_common_traversable_area_data.header.resolution;
      Eigen::Vector3d tem_pose(tem_x, tem_y, 0);
      Eigen::Vector3d tem_relative_pose = tem_reference_pose * tem_pose;//转成当前帧子地图的坐标系下(单位是米)
      int tem_index_x = (tem_relative_pose.x() / mapheader.resolution + mapheader.pose_index_x);//转成当前帧子地图坐标系下(单位是栅格)
      int tem_index_y = (tem_relative_pose.y() / mapheader.resolution + mapheader.pose_index_y);
      if (tem_index_x >= 0 && tem_index_x < mapheader.width && tem_index_y >= 0 && tem_index_y < mapheader.height){
        // TODO 暂时将这里修改成直接拷贝原来的概率值和观测计数 - hsh 20201214
        traversable_area_data[datatype_].data[tem_index_y * mapheader.width + tem_index_x]
          = tem_common_traversable_area_data.data[i * tem_common_traversable_area_data.header.width + j];
        continue;
        // ! 为什么将上一帧栅格地图转换到当前坐标系的时候也需要更新栅格占据概率???
        // 若上一帧栅格被占据
        if (tem_value > 0.5){
          // 负障碍距离车体坐标系原点小于8米的时候不做概率更新
          if (datatype_ == DataType::KNegative
            && (vehicle_pose.inverse() * tem_relative_pose).head(2).norm() < negative_dead_zone_radius)
            continue;
          traversable_area_data[datatype_].data[tem_index_y * mapheader.width + tem_index_x].possibility
            = BayesUpdator(tem_value, KNullProbability);
          traversable_area_data[datatype_].data[tem_index_y * mapheader.width + tem_index_x].observation_num
            = tem_common_traversable_area_data.data[i * tem_common_traversable_area_data.header.width + j].observation_num;
        }
        // 若上一帧栅格未被占据
        else if (tem_value < 0.4){
          traversable_area_data[datatype_].data[tem_index_y * mapheader.width + tem_index_x].possibility
            = BayesUpdator(tem_value, KHitProbability);
          traversable_area_data[datatype_].data[tem_index_y * mapheader.width + tem_index_x].observation_num
            = tem_common_traversable_area_data.data[i * tem_common_traversable_area_data.header.width + j].observation_num;
        }
        else // 这个不是涵盖了下面的那个了吗？zhubc
        {
          // traversable_area_data[datatype_].data[tem_index_y * mapheader.width + tem_index_x].possibility = 0.5;
          traversable_area_data[datatype_].data[tem_index_y * mapheader.width + tem_index_x].possibility = -1;
          traversable_area_data[datatype_].data[tem_index_y * mapheader.width + tem_index_x].observation_num = 0;
        }

        // 下面这段已经被上面涵盖了，将其注释 - 20201213 hsh
        // if((vehicle_pose.inverse() * tem_relative_pose).head(2).norm()>5 //非盲区
        //    &&traversable_area_data[datatype_].data[tem_index_y*mapheader.width + tem_index_x].possibility>=0.4
        //    &&traversable_area_data[datatype_].data[tem_index_y*mapheader.width + tem_index_x].possibility<=0.5)
        // {
        //     traversable_area_data[datatype_].data[tem_index_y*mapheader.width + tem_index_x].possibility = -1;
        //     traversable_area_data[datatype_].data[tem_index_y*mapheader.width + tem_index_x].observation_num = 0;
        // }
      }
    }
  }
  map_size_switching[datatype_] = false;
  LOG(INFO) << "CommonObjectSwitcher cost time " << datatype_ << " :" << (ros::Time::now() - time1).toSec();
  thread_num--;
}

void Node::MapSizeUpdater(DataType datatype_, const MapHeader& mapheader){
  if (traversable_area_data[datatype_].header.height == mapheader.height &&
    traversable_area_data[datatype_].header.width == mapheader.width &&
    traversable_area_data[datatype_].header.pose_index_x == mapheader.pose_index_x &&
    traversable_area_data[datatype_].header.pose_index_y == mapheader.pose_index_y)
    return;
  CHECK(!map_size_updating[datatype_]);
  ros::Time time1 = ros::Time::now();
  // if(map_size_updating[datatype_] == true){return;}
  map_size_updating[datatype_] = true;
  // MapHeader 里面只有地图尺寸的一系列信息，而没有地图里面栅格的值
  TraversableAreaData tem_traversable_area_data;
  tem_traversable_area_data.header = traversable_area_data[datatype_].header;//上一次的值
  tem_traversable_area_data.data.swap(traversable_area_data[datatype_].data);
  // if (datatype_ == DataType::KPositive) {
  //     ROS_INFO("traversable_area_data[DataType::KPositive] size change from (%d,%d) to (%d,%d)", 
  //         traversable_area_data[datatype_].header.height, traversable_area_data[datatype_].header.width, 
  //         mapheader.height, mapheader.width);
  // }
  traversable_area_data[datatype_].header = mapheader;//更新
  traversable_area_data[datatype_].data.assign(mapheader.width * mapheader.height, CellData());

  // LOG(INFO)<<"assign cost time "<<datatype_<<" :"<<(ros::Time::now()-time1).toSec();

  // 这里涉及到坐标转换的问题，就是如果当前发过来的地图和之前的车辆位置不同，需要转移一下。之前判断了栅格总数是一致的。
  for (int i = 0; i < tem_traversable_area_data.header.height;i++){
    for (int j = 0; j < tem_traversable_area_data.header.width;j++){
      if (tem_traversable_area_data.data[i * tem_traversable_area_data.header.width + j].possibility <= 0)
        continue;
      int index_x = j - tem_traversable_area_data.header.pose_index_x + traversable_area_data[datatype_].header.pose_index_x;
      int index_y = i - tem_traversable_area_data.header.pose_index_y + traversable_area_data[datatype_].header.pose_index_y;
      if (index_x >= 0 && index_x < traversable_area_data[datatype_].header.width
        && index_y >= 0 && index_y < traversable_area_data[datatype_].header.height){
        traversable_area_data[datatype_].data[index_y * traversable_area_data[datatype_].header.width + index_x] =
          tem_traversable_area_data.data[i * tem_traversable_area_data.header.width + j];
      }

    }
  }

  map_size_updating[datatype_] = false;
  LOG(INFO) << "MapSizeUpdater cost time " << datatype_ << " :" << (ros::Time::now() - time1).toSec();
}

void Node::CommonObjectProcessor(DataType datatype_, const MapData mapdata, const MapHeader& mapheader){
  // 这里的 MapHeader 是 PrimaryTraversalbe, 准可通行区域的位姿
  if (mapheader.width <= 0 || mapheader.height <= 0){
    LOG(WARNING) << "common objectprocessor mapheader size zero!";
    return;
  }
  // LOG(INFO)<<"CommonObjectProcessor runing";
  thread_num++;
  ::ivcommon::MutexLocker lock_eachtype(&mutexs_eachtype[datatype_]);
  bool filter_on = false;
  // 第一帧
  if (traversable_area_data[datatype_].header.time == ::ivcommon::Time::min()){
    TraversableAreaData tem_TraversableAreaData;
    tem_TraversableAreaData.header = mapheader;
    tem_TraversableAreaData.data.assign(mapheader.width * mapheader.height, CellData());
    traversable_area_data[datatype_] = std::move(tem_TraversableAreaData);
  }
  else if (traversable_area_data[datatype_].header.index == mapheader.index) // 确认栅格总数相同
  {
    // 更新地图中车体位姿以及对应的坐标系里的所有值，将上一帧的值都转换到了当前帧里，并且现在位姿也是当前帧
    MapSizeUpdater(datatype_, mapheader);
  }

  // ::ivcommon::MutexLocker lock(&mutex);
  int passable_flag(1), obstacle_flag(0);
  if (datatype_ == DataType::KNegative){
    obstacle_flag = 4;	  //negative obstacle
    filter_on = true;
  }
  else if (datatype_ == DataType::KPositive){
    obstacle_flag = 3;	  //positive obstacle
    filter_on = true;
  }
  else if (datatype_ == DataType::KPositiveSlope){
    obstacle_flag = 7;	  //slope obstacle
  }
  else if (datatype_ == DataType::KNegativeSlope){
    obstacle_flag = 8;	  //slope obstacle
  }
  else if (datatype_ == DataType::KStiff){
    obstacle_flag = 5;	  //stiff obstacle
  }
  else if (datatype_ == DataType::KWater){
    obstacle_flag = 6;	  //water obstacle
  }
  else if (datatype_ == DataType::KUnevenArea){
    obstacle_flag = 9;	  //uneven obstacle
  }
  else if (datatype_ == DataType::KRefinePositive){
    obstacle_flag = 10;
    filter_on = true;
  }
  cv::Mat area_img = cv::Mat::zeros(mapheader.height, mapheader.width, CV_8U);
  cv::Mat area_img_dilated;
  if (filter_on) // 看来是只对正负障碍物检测才有
  {
    ::ivcommon::MutexLocker lock_eachtype(&mutexs_eachtype[DataType::KPrimaryTraversableArea]);
    ConvertToNowCoordinationMat(area_img, datatype_, mapheader);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 + 1, 2 + 1));//生成形态学操作用到的核
    cv::dilate(area_img, area_img_dilated, element, cv::Point(1, 1));
  }
  // 将当前帧障碍物图转到当前帧准可通行区域里去
  ::ivcommon::transform::Rigid3d tem_reference_pose = 
    traversable_area_data[datatype_].header.pose.inverse() * mapdata.header.pose;

  // if (datatype_ == DataType::KPositive){
  //   show_obstacle_img = cv::Mat::zeros(mapdata.header.height, mapdata.header.width, CV_8UC3);
  //   traversable_area_data_img = cv::Mat::zeros(
  //     traversable_area_data[datatype_].header.height, traversable_area_data[datatype_].header.width, CV_8UC3);
  // }
  auto getPoint = [&mapdata, &tem_reference_pose, this, &datatype_](const double x, const double y) -> cv::Point2d{
    double tem_y = (y - mapdata.header.pose_index_y) * mapdata.header.resolution;
    double tem_x = (x - mapdata.header.pose_index_x) * mapdata.header.resolution;
    Eigen::Vector3d tem_pose1(tem_x, tem_y, 0);
    Eigen::Vector3d tem_relative_pose1 = tem_reference_pose * tem_pose1;
    int tem_index_x = tem_relative_pose1.x() / this->traversable_area_data[datatype_].header.resolution + this->traversable_area_data[datatype_].header.pose_index_x;
    int tem_index_y = tem_relative_pose1.y() / this->traversable_area_data[datatype_].header.resolution + this->traversable_area_data[datatype_].header.pose_index_y;
    return cv::Point2d(tem_index_x, tem_index_y);
  };

  for (int i = 0; i < mapdata.header.height; i++){
    for (int j = 0; j < mapdata.header.width; j++){
      int tem_value = mapdata.data[i * mapdata.header.width + j];
      // if (datatype_ == DataType::KPositive){
      //   if (tem_value == obstacle_flag){
      //     show_obstacle_img.at<cv::Vec3b>(i, j)[0] = 0;
      //     show_obstacle_img.at<cv::Vec3b>(i, j)[1] = 0;
      //     show_obstacle_img.at<cv::Vec3b>(i, j)[2] = 255;
      //   }
      //   else if (tem_value == passable_flag){
      //     show_obstacle_img.at<cv::Vec3b>(i, j)[0] = 0;
      //     show_obstacle_img.at<cv::Vec3b>(i, j)[1] = 255;
      //     show_obstacle_img.at<cv::Vec3b>(i, j)[2] = 0;
      //   }
      // }
      if (tem_value == obstacle_flag){
        // if(obstacle_flag==3&&i<28/mapdata.header.resolution
        // 		&&j>15/mapdata.header.resolution&&j<25/mapdata.header.resolution)
        // 	continue;
        cv::Point2d tem_point = getPoint(j, i);
        // double tem_x = (j - mapdata.header.pose_index_x) * mapdata.header.resolution;
        // double tem_y = (i - mapdata.header.pose_index_y) * mapdata.header.resolution;
        // Eigen::Vector3d tem_pose1(tem_x, tem_y, 0);
        // Eigen::Vector3d tem_relative_pose1 = tem_reference_pose * tem_pose1;
        // // 将障碍物栅格图中每一个栅格转移到可通行区域的栅格图中
        // int tem_index_x = tem_relative_pose1.x()/traversable_area_data[datatype_].header.resolution
        //                   + traversable_area_data[datatype_].header.pose_index_x;
        // int tem_index_y = tem_relative_pose1.y()/traversable_area_data[datatype_].header.resolution
        //                   + traversable_area_data[datatype_].header.pose_index_y;

        if (tem_point.x >= 0 && tem_point.x < traversable_area_data[datatype_].header.width &&
          tem_point.y >= 0 && tem_point.y < traversable_area_data[datatype_].header.height)// &&
          // (!filter_on || area_img_dilated.at<uchar>(tem_point.y, tem_point.x) != 1)) // 栅格的 possibility 小于0.4的时候才是1
        {
          // 贝叶斯概率更新
          // HSH
          // traversable_area_data[datatype_].data.at(
          //   tem_point.y * traversable_area_data[datatype_].header.width + tem_point.x).possibility = 0.6;
          traversable_area_data[datatype_].data.at(
            tem_point.y * traversable_area_data[datatype_].header.width + tem_point.x).possibility =
            BayesUpdator(traversable_area_data[datatype_].data.at(
              tem_point.y * traversable_area_data[datatype_].header.width + tem_point.x).possibility, KHitProbability);
          traversable_area_data[datatype_].data.at(
            tem_point.y * traversable_area_data[datatype_].header.width + tem_point.x).observation_num += 1;
        }
      }
      // 可通行区域的概率更新
      else if (tem_value == passable_flag){
        cv::Point2d tem_point = getPoint(j, i);
        // Eigen::Vector3d tem_pose1((j - mapdata.header.pose_index_x) * mapdata.header.resolution,
        //   (i - mapdata.header.pose_index_y) * mapdata.header.resolution, 0);
        // if (tem_pose1.y() < 0 || (tem_pose1.head(2).norm() < 6 && tem_pose1.y()<5)) continue;
        // double tem_y = (i-mapdata.header.pose_index_y)*mapdata.header.resolution;
        // double tem_x = (j-mapdata.header.pose_index_x)*mapdata.header.resolution;
        // Eigen::Vector3d tem_pose1(tem_x,tem_y,0);
        // Eigen::Vector3d tem_relative_pose1 = tem_reference_pose * tem_pose1;
        // int tem_index_x = tem_relative_pose1.x()/traversable_area_data[datatype_].header.resolution + traversable_area_data[datatype_].header.pose_index_x;
        // int tem_index_y = tem_relative_pose1.y()/traversable_area_data[datatype_].header.resolution + traversable_area_data[datatype_].header.pose_index_y;
        if (tem_point.x >= 0 && tem_point.x < traversable_area_data[datatype_].header.width &&
          tem_point.y >= 0 && tem_point.y < traversable_area_data[datatype_].header.height){
          traversable_area_data[datatype_].data.at(tem_point.y * traversable_area_data[datatype_].header.width + tem_point.x).possibility =
            BayesUpdator(traversable_area_data[datatype_].data.at(tem_point.y * traversable_area_data[datatype_].header.width + tem_point.x).possibility, KNullProbability);
          if (traversable_area_data[datatype_].data.at(tem_point.y * traversable_area_data[datatype_].header.width + tem_point.x).observation_num > 0)
            traversable_area_data[datatype_].data.at(tem_point.y * traversable_area_data[datatype_].header.width + tem_point.x).observation_num -= 1;
        }
      }
    }
  }

  // if (datatype_ == DataType::KPositive){
  //   for (int i = 0; i < traversable_area_data[datatype_].header.height; i++){
  //     for (int j = 0; j < traversable_area_data[datatype_].header.width; j++){
  //       double possibility = traversable_area_data[datatype_].data[i * traversable_area_data[datatype_].header.width + j].possibility;
  //       if (possibility > 0.5){
  //         traversable_area_data_img.at<cv::Vec3b>(i, j)[0] = 0;
  //         traversable_area_data_img.at<cv::Vec3b>(i, j)[1] = 0;
  //         traversable_area_data_img.at<cv::Vec3b>(i, j)[2] = 230;
  //       }
  //       else if (possibility > 0.4){
  //         traversable_area_data_img.at<cv::Vec3b>(i, j)[0] = 100;
  //         traversable_area_data_img.at<cv::Vec3b>(i, j)[1] = 100;
  //         traversable_area_data_img.at<cv::Vec3b>(i, j)[2] = 100;
  //       }
  //       else{
  //         traversable_area_data_img.at<cv::Vec3b>(i, j)[0] = 0;
  //         traversable_area_data_img.at<cv::Vec3b>(i, j)[1] = 220;
  //         traversable_area_data_img.at<cv::Vec3b>(i, j)[2] = 0;
  //       }
  //     }
  //   }
  //   cv::Point2d left_top = getPoint(0, 0);
  //   cv::Point2d right_top = getPoint(mapdata.header.width - 1, 0);
  //   cv::Point2d right_bottom = getPoint(mapdata.header.width - 1, mapdata.header.height - 1);
  //   cv::Point2d left_bottom = getPoint(0, mapdata.header.height - 1);
  //   cv::Point2d vehicle_pos = getPoint(mapdata.header.pose_index_x, mapdata.header.pose_index_y);
  //   cv::line(traversable_area_data_img, left_top, right_top, cv::Scalar(0, 214, 255), 2);
  //   cv::line(traversable_area_data_img, left_top, left_bottom, cv::Scalar(0, 214, 255), 2);
  //   cv::line(traversable_area_data_img, right_top, right_bottom, cv::Scalar(0, 214, 255), 2);
  //   cv::line(traversable_area_data_img, left_bottom, right_bottom, cv::Scalar(0, 214, 255), 2);
  //   cv::circle(traversable_area_data_img, vehicle_pos, 3, cv::Scalar(0, 214, 255), 2);

  //   cv::flip(show_obstacle_img, show_obstacle_img, 0);
  //   cv::flip(traversable_area_data_img, traversable_area_data_img, 0);
  // }
  // lock.Await([this]() { return !updating_flag&&!switching_falg; });
  ::ivcommon::MutexLocker lock_pending_pub_data(&mutex_pending_pub_data);
  if (pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.index
    >= traversable_area_data[datatype_].header.index)
    pending_pub_traversable_area_data[datatype_] = traversable_area_data[datatype_];
  else
    LOG(WARNING) << datatype_ << ": "
    << pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.index << " < "
    << traversable_area_data[datatype_].header.index
    << ". pending_pub_traversable_area_data index < traversable_area_data index !!!";
  CHECK_GE(pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.index
    , pending_pub_traversable_area_data[datatype_].header.index);
  LOG(INFO) << "cost time of " << datatype_ << " : " << (ros::Time::now() - ::ivcommon::ToRos(mapdata.header.time)).toSec();
  thread_num--;
}

//用于调度各障碍物信息，并舍弃过时数据或无效数据
void Node::CommonObjectDispatch(DataType datatype_){
  // 其实里程计位姿就是里程计坐标系下的车辆位姿，因为所有的障碍物检测都是基于车体坐标系进行的，
  // 因此只要对准时间戳，就直接把里程计的位姿赋给障碍物检测的栅格图就行。
  CHECK(datatype_ != DataType::KPrimaryTraversableArea);

  LOG(INFO) << "it is going to dispatch: " << datatype_;
  std::deque<LidarOdometryData> temp_lidar_odometry_data;
  {
    ::ivcommon::MutexLocker lock(&mutex_lidarodometry);
    if (lidar_odometry_data.empty()){
      LOG(WARNING) << "lidar_odometry_data is empty";
      return;
    }
    temp_lidar_odometry_data = lidar_odometry_data;
  }
  bool time_matched = false;

  ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
  while (roadblock_data[datatype_].size() > 0){
    time_matched = false;

    int odom_size = temp_lidar_odometry_data.size();
    int odom_start = std::max(0, odom_size - 2); // odom_start = 6 or 7 or 8
    // 舍弃过时数据
    if (roadblock_data[datatype_].front().header.time < temp_lidar_odometry_data[odom_start].time){
      LOG(INFO) << "CommonObeject: " << datatype_ << " CommonObeject time:" << roadblock_data[datatype_].front().header.time << " didn`t find corresponding pose, popped!";
      roadblock_data[datatype_].pop_front();
      continue;
    }
    // temp_lidar_odometry_data 里有当前帧及前一帧共2帧数据
    // 只能是当 roadblock_data 里面有2帧数据的时候才能继续往下走，或者就是roadblock_data里面有一帧数据，时间戳是大于等于里程计的
    for (int i = odom_start;i < odom_size;i++){
      if (roadblock_data[datatype_].front().header.time == temp_lidar_odometry_data[i].time){
        if (temp_lidar_odometry_data[i].time > latest_vechicle_pose.time){
          ::ivcommon::MutexLocker lock(&mutex_latest_vechicle_pose); //互斥锁是怎么跟数据对上的
          latest_vechicle_pose.time = temp_lidar_odometry_data[i].time;//更新位姿和时间
          latest_vechicle_pose.pose = temp_lidar_odometry_data[i].pose;//里程计位姿，也就是相对位姿
        }
        time_matched = true;
        // 在这里给出来的位姿，其实就是当前帧里程计的位姿！！！！！也就是MapHeader里面的pose
        roadblock_data[datatype_].front().header.pose = temp_lidar_odometry_data[i].pose;

        MapData tem_map = std::move(roadblock_data[datatype_].front());

        roadblock_data[datatype_].pop_front();
        // LOG(ERROR)<<"[NotError] roadblock time = lidar odom time: " << tem_map.header.time << " pose: " << tem_map.header.pose;
        ::ivcommon::MutexLocker lock_eachtype(&mutexs_eachtype[DataType::KPrimaryTraversableArea]);
        boost::thread common_object_thread(boost::bind(&Node::CommonObjectProcessor, this, datatype_, tem_map
          , traversable_area_data[DataType::KPrimaryTraversableArea].header));
        common_object_thread.detach();
        break;
      }
    }
    if (time_matched) continue;
    if (roadblock_data[datatype_].front().header.time < temp_lidar_odometry_data.back().time){
      LOG(INFO) << "CommonObeject: " << datatype_ << " CommonObeject time:" << roadblock_data[datatype_].front().header.time << "didn`t find corresponding pose, poped!";
      roadblock_data[datatype_].pop_front();
      continue;
    }
    break;

  }
  // LOG(INFO)<<"thread_num="<<thread_num;
  return;
}

void Node::ProcessDynamicObject(DataType datatype_, traversable_area_extraction::DynamicObject dynamic_objects){
  LOG(INFO) << "ProcessDynamicObject runing";
  ::ivcommon::MutexLocker lock_eachtype(&mutexs_eachtype[datatype_]);
  if (traversable_area_data[datatype_].header.time == ::ivcommon::Time::min())
    return;
  ::ivcommon::transform::Rigid3d map_pose = global_origin_pose * traversable_area_data[datatype_].header.pose;
  ::ivcommon::transform::Rigid3d map_pose_inverse = map_pose.inverse();
  LOG(INFO) << "targetnum:" << dynamic_objects.target_num << " " << map_pose;
  double resolution = traversable_area_data[datatype_].header.resolution;
  int map_index_x = traversable_area_data[datatype_].header.pose_index_x;
  int map_index_y = traversable_area_data[datatype_].header.pose_index_y;
  int width = traversable_area_data[datatype_].header.width;
  int height = traversable_area_data[datatype_].header.height;
  for (int i = 0;i < dynamic_objects.target_num;i++){
    auto& tem_movingtarget = dynamic_objects.moving_target.at(i);//todo
    LOG(INFO) << ::ivcommon::ToRos(traversable_area_data[datatype_].header.time) << " " << map_pose;
    LOG(INFO) << "history_num:" << tem_movingtarget.history_num;
    if (tem_movingtarget.history_num > 0){
      for (int j = 0; j < tem_movingtarget.history_num;j++){
        auto& one_history_traj = tem_movingtarget.history_traj.at(j);
        std::vector<Eigen::Array2i> object_rectangle;

        for (int k = 0; k < 4;k++){

          Eigen::Vector3d tem_local_point = map_pose_inverse *
            Eigen::Vector3d(one_history_traj.points[k].x(), one_history_traj.points[k].y()
              , (map_pose.translation()).z());
          LOG(INFO) << "x:" << tem_local_point.x() << " y:" << tem_local_point.y();
          int index_x = tem_local_point.x() / resolution + map_index_x;
          int index_y = tem_local_point.y() / resolution + map_index_y;

          object_rectangle.push_back(Eigen::Array2i(index_x, index_y));
        }

        int min_x = std::min<int>(object_rectangle[0].x(),
          std::min<int>(object_rectangle[1].x(), std::min<int>(object_rectangle[2].x(), object_rectangle[3].x())));
        int min_y = std::min<int>(object_rectangle[0].y(),
          std::min<int>(object_rectangle[1].y(), std::min<int>(object_rectangle[2].y(), object_rectangle[3].y())));
        int max_x = std::max<int>(object_rectangle[0].x(),
          std::max<int>(object_rectangle[1].x(), std::max<int>(object_rectangle[2].x(), object_rectangle[3].x())));
        int max_y = std::max<int>(object_rectangle[0].y(),
          std::max<int>(object_rectangle[1].y(), std::max<int>(object_rectangle[2].y(), object_rectangle[3].y())));

        int expand_num = 1;
        for (int ind_i = min_x - expand_num; ind_i <= max_x + expand_num;ind_i++){
          for (int ind_j = min_y - expand_num; ind_j <= max_y + expand_num;ind_j++){
            if (ind_i >= 0 && ind_j >= 0 && ind_i < width && ind_j < height){
              traversable_area_data[datatype_].data[ind_j * width + ind_i].observation_num = 0;
              traversable_area_data[datatype_].data[ind_j * width + ind_i].possibility = -1;
            }
          }
        }
      }
    }
  }
  ::ivcommon::MutexLocker lock_pending_pub_data(&mutex_pending_pub_data);
  if (pending_pub_traversable_area_data[DataType::KPrimaryTraversableArea].header.index
    >= traversable_area_data[datatype_].header.index)
    pending_pub_traversable_area_data[datatype_] = traversable_area_data[datatype_];
}

void Node::HandleDynamicObjectMessage(const iv_dynamicobject_msgs::moving_target_send::ConstPtr& msg){
  if (global_origin_pose.translation().x() == 0 || global_origin_pose.translation().y() == 0){
    // LOG(WARNING) << "global_origin_pose msg Zero";
    return;
  }

  traversable_area_extraction::DynamicObject dynamic_object;

  dynamic_object.time = ::ivcommon::FromRos(ros::Time(msg->time_stamp));
  LOG(WARNING) << "dynanic_num:" << msg->target_num;
  dynamic_object.target_num = msg->target_num;

  for (int i = 0; i < dynamic_object.target_num; i++){
    traversable_area_extraction::DynamicObject::MovingTarget tem_moving_target;

    tem_moving_target.is_updated = msg->target.at(i).is_updated;
    tem_moving_target.history_num = msg->target.at(i).history_num;

    for (int j = 0; j < tem_moving_target.history_num; j++){
      const auto& one_history = msg->target.at(i).history_traj.at(j);
      traversable_area_extraction::DynamicObject::MovingTarget::HistoryTraj tem_history_traj;
      tem_history_traj.center_point = Eigen::Vector3d(one_history.center_point.x,
        one_history.center_point.y, one_history.center_point.z);
      for (int k = 0; k < 4;k++){
        tem_history_traj.points.push_back(Eigen::Vector3d(one_history.line_point.at(k).x,
          one_history.line_point.at(k).y, one_history.line_point.at(k).z));
      }
      tem_moving_target.history_traj.push_back(tem_history_traj);
    }
    dynamic_object.moving_target.push_back(tem_moving_target);
  }
  ProcessDynamicObject(DataType::KPositive, dynamic_object);
}

void Node::HandleNegativeObjectMessage(const negative_msgs::NegativeOGM::ConstPtr& msg){  //jkj 0807
  // clock_t start_time = clock();
  if (!traversable_area_inited){
    // LOG(WARNING) << "traversable_area_inited didn`t finished!";
    // ROS_WARN_THROTTLE(5, "ObjectMsg: traversable_area_inited didn`t finished");
    return;
  }
  if (global_origin_pose.translation().x() == 0 || global_origin_pose.translation().y() == 0){
    // LOG(WARNING) << "global_origin_pose msg Zero";
    ROS_WARN_THROTTLE(5, "global_origin_pose msg Zero");
    return;
  }

  if (msg->ogmwidth <= 0 || msg->ogmheight <= 0 || msg->vehicle_x < 0 || msg->vehicle_y < 0 || msg->header.stamp.toSec() <= 0){
    LOG(ERROR) << "HandleNegativeObjectMessage msg  is invalid!";
    return;
  }

  MapData negative_obstacle_map;
  negative_obstacle_map.header.time = ::ivcommon::FromRos(msg->header.stamp);
  negative_obstacle_map.header.width = msg->ogmwidth;
  negative_obstacle_map.header.height = msg->ogmheight;
  negative_obstacle_map.header.resolution = msg->ogmresolution;
  negative_obstacle_map.header.pose_index_x = msg->vehicle_x;
  negative_obstacle_map.header.pose_index_y = msg->vehicle_y;

  negative_obstacle_map.data.assign(negative_obstacle_map.header.width * negative_obstacle_map.header.height, 0);//初始化

  MapData obstacle_map = negative_obstacle_map;
  MapData tem_map_forsingle = negative_obstacle_map;
  MapData low_obstacle_map = negative_obstacle_map;
  // 3是更高的高度差阈值出来的障碍物，4是负障碍，10是较小的高度差阈值出来的障碍物，1是可通行区域
  for (int i = 0;i < msg->data.size();i++){
    if (msg->data[i] == 10){
      low_obstacle_map.data[i] = msg->data[i];//较小的高度差阈值出来的正障碍物
    }
    else if (msg->data[i] == 4){
      negative_obstacle_map.data[i] = msg->data[i];//负障碍
    }
    else if (msg->data[i] == 3){
      obstacle_map.data[i] = msg->data[i];//更高的高度差阈值出来的正障碍物
      tem_map_forsingle.data[i] = msg->data[i];
    }
    else if (msg->data[i] == 1){
      tem_map_forsingle.data[i] = msg->data[i];
      if (traversable_area_option_.wipe_history_obstacle){
        obstacle_map.data[i] = msg->data[i];
        low_obstacle_map.data[i] = msg->data[i];
      }
    }
  }

  // 将当前帧负障碍图放入 roadblock_data 和 roadblock_data2
  if (!negative_obstacle_map.data.empty()){
    ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
    roadblock_data[DataType::KNegative].push_back(negative_obstacle_map);
    roadblock_data2[DataType::KNegative].push_back(std::move(negative_obstacle_map));
    while (roadblock_data[DataType::KNegative].size() > kBufferSize){
      LOG(INFO) << " KNegative Obeject isn't processed timely,dropped stamp: " << roadblock_data[DataType::KNegative].front().header.time;
      roadblock_data[DataType::KNegative].pop_front();
      roadblock_data[DataType::KNegative].pop_front();
      // 最后 roadblcok_data 里面有2帧或者1帧数据，而 roadblcok_data2 里面有所有帧数据
    }
  }
  // 将高度差更高的障碍物地图放入 roadblock_data 里面去
  if (!obstacle_map.data.empty()){
    ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
    roadblock_data[DataType::KPositive].push_back(std::move(obstacle_map));
    roadblock_data2[DataType::KPositive].push_back(std::move(tem_map_forsingle));//tem_map_forsingle里面集成了更高正障碍和可通行区域
    while (roadblock_data[DataType::KPositive].size() > kBufferSize){
      LOG(INFO) << " KPositive Obeject isn't processed timely,dropped stamp: " << roadblock_data[DataType::KPositive].front().header.time;
      roadblock_data[DataType::KPositive].pop_front();
      roadblock_data[DataType::KPositive].pop_front();
    }
  }
  // 将高度差更低的障碍物地图放入 roadblock_data 里面去
  if (!low_obstacle_map.data.empty()){
    ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
    roadblock_data[DataType::KRefinePositive].push_back(low_obstacle_map);
    roadblock_data2[DataType::KRefinePositive].push_back(std::move(low_obstacle_map));
    while (roadblock_data[DataType::KRefinePositive].size() > kBufferSize){
      LOG(INFO) << " KRefinePositive Obeject isn't processed timely,dropped stamp: " << roadblock_data[DataType::KRefinePositive].front().header.time;
      roadblock_data[DataType::KRefinePositive].pop_front();
      roadblock_data[DataType::KRefinePositive].pop_front();
    }
  }
  // LOG(INFO)<<"it is going to dispatch: DataType::KNegative";
  CommonObjectDispatch(DataType::KNegative);
  // LOG(INFO)<<"it is going to dispatch: DataType::KPositive";
  CommonObjectDispatch(DataType::KPositive);
  CommonObjectDispatch(DataType::KRefinePositive);
  // clock_t end_time = clock();
  // ROS_INFO("HandleNegativeObjectMessage time cost: %f ms", (double)(end_time - start_time) / CLOCKS_PER_SEC * 1000.0);
}

void Node::HandleBackOgmMessage(const obstacle_msgs::ObstacleOGM::ConstPtr& msg){//jkj 0807
  if (!traversable_area_inited){
    // LOG(INFO) << "traversable_area_inited didn`t finished!";
    return;
  }
  if (global_origin_pose.translation().x() == 0 || global_origin_pose.translation().y() == 0){
    // LOG(WARNING) << "global_origin_pose msg Zero";
    return;
  }

  if (msg->ogmwidth <= 0 || msg->ogmheight <= 0 || msg->vehicle_x < 0 || msg->vehicle_y < 0 || msg->header.stamp.toSec() <= 0){
    LOG(ERROR) << "HandleBackOgmMessage msg  is invalid!";
    return;
  }
  LOG(WARNING) << "HandleBackOgmMessage";
  MapData tem_map;
  tem_map.header.time = ::ivcommon::FromRos(msg->header.stamp);
  tem_map.header.width = msg->ogmwidth;
  tem_map.header.height = msg->ogmheight;
  tem_map.header.resolution = msg->ogmresolution;
  tem_map.header.pose_index_x = msg->vehicle_x;
  tem_map.header.pose_index_y = msg->vehicle_y;

  tem_map.data.assign(tem_map.header.width * tem_map.header.height, 0);
  for (int i = 0;i < msg->data.size();i++){
    if (msg->data[i] == 3 || msg->data[i] == 10){
      tem_map.data[i] = msg->data[i];//正障碍
    }
    else if (msg->data[i] == 1){
      tem_map.data[i] = msg->data[i];//可通行区域
      //    	  tem_map.data[i] = msg->data[i];
      //    	  tem_map2.data[i] = msg->data[i];
    }
  }
  //后置雷达正障碍物地图只存到了roadblock_data2
  if (!tem_map.data.empty()){
    ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
    //	roadblock_data[DataType::KPositive].push_back(tem_map2);
    roadblock_data2[DataType::KBackOGM].push_back(tem_map);
    while (roadblock_data2[DataType::KBackOGM].size() > 1)
      roadblock_data2[DataType::KBackOGM].pop_front();//roadblcok_data2里面只1帧后置雷达的障碍物数据
  }
}

void Node::HandleSlopeObjectMessage(const slopeogm_msgs::SlopeOGM::ConstPtr& msg){
  if (!traversable_area_inited){
    // LOG(INFO) << "traversable_area_inited didn`t finished!";
    return;
  }
  //       ::ivcommon::MutexLocker lock(&mutex);
  if (global_origin_pose.translation().x() == 0 || global_origin_pose.translation().y() == 0){
    // LOG(WARNING) << "global_origin_pose msg Zero";
    return;
  }

  if (msg->ogmwidth <= 0 || msg->ogmheight <= 0 || msg->vehicle_x < 0 || msg->vehicle_y < 0 || msg->header.stamp.toSec() <= 0){
    LOG(ERROR) << "HandleSlopeObjectMessage msg  is invalid!";
    return;
  }
  //         LOG(WARNING)<<"HandleSlopeObjectMessage runing!"; 
  MapData tem_map;
  tem_map.header.time = ::ivcommon::FromRos(msg->header.stamp);
  tem_map.header.width = msg->ogmwidth;
  tem_map.header.height = msg->ogmheight;
  tem_map.header.resolution = msg->ogmresolution;
  tem_map.header.pose_index_x = msg->vehicle_x;
  tem_map.header.pose_index_y = msg->vehicle_y;
  tem_map.data.assign(tem_map.header.width * tem_map.header.height, 0);
  MapData tem_map2 = tem_map;
  for (int i = 0;i < msg->slopeval.size();i++){
    if (msg->slopeval[i] == 7)tem_map.data[i] = msg->slopeval[i];//正斜坡
    if (msg->slopeval[i] == 8)tem_map2.data[i] = msg->slopeval[i];//负斜坡
    if (msg->slopeval[i] == 1){
      tem_map.data[i] = msg->slopeval[i];
      tem_map2.data[i] = msg->slopeval[i];
    }
  }
  // 将正斜坡的障碍物地图放入 roadblock_data 和 roadblock_data2 里面去
  if (!tem_map.data.empty()){
    ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
    roadblock_data[DataType::KPositiveSlope].push_back(tem_map);//这里面始终会有1~2帧数据
    roadblock_data2[DataType::KPositiveSlope].push_back(tem_map);
    while (roadblock_data[DataType::KPositiveSlope].size() > kBufferSize){
      LOG(WARNING) << " KPositiveSlope Obeject isn't processed timely,dropped stamp:" << roadblock_data[DataType::KPositiveSlope].front().header.time;
      roadblock_data[DataType::KPositiveSlope].pop_front();
      roadblock_data[DataType::KPositiveSlope].pop_front();
    }
  }
  // 将负斜坡的障碍物地图放入 roadblock_data 和 roadblock_data2 里面去
  if (!tem_map2.data.empty()){

    ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
    roadblock_data[DataType::KNegativeSlope].push_back(tem_map2);
    roadblock_data2[DataType::KNegativeSlope].push_back(tem_map2);
    while (roadblock_data[DataType::KNegativeSlope].size() > kBufferSize){
      LOG(WARNING) << " KNegativeSlope Obeject isn't processed timely,dropped stamp:" << roadblock_data[DataType::KNegativeSlope].front().header.time;
      roadblock_data[DataType::KNegativeSlope].pop_front();
      roadblock_data[DataType::KNegativeSlope].pop_front();
    }
  }
  //      LOG(INFO)<<"it is going to dispatch: DataType::KNegativeSlope";
  CommonObjectDispatch(DataType::KPositiveSlope);
  //     LOG(INFO)<<"it is going to dispatch: DataType::KWater";
  CommonObjectDispatch(DataType::KNegativeSlope);


}
void Node::HandleStiffObjectMessage(const stiff_msgs::stiffwater::ConstPtr& msg){//jkj 0818
  if (!traversable_area_inited){
    // LOG(INFO) << "traversable_area_inited didn`t finished!";
    return;
  }
  //       ::ivcommon::MutexLocker lock(&mutex);
  if (global_origin_pose.translation().x() == 0 || global_origin_pose.translation().y() == 0){
    // LOG(WARNING) << "global_origin_pose msg Zero";
    return;
  }
  if (msg->ogmwidth <= 0 || msg->ogmheight <= 0 || msg->vehicle_x < 0 || msg->vehicle_y < 0 || msg->header.stamp.toSec() <= 0){
    LOG(ERROR) << "HandleStiffObjectMessage msg  is invalid!";
    return;
  }
  MapData tem_map;
  tem_map.header.time = ::ivcommon::FromRos(msg->header.stamp);
  tem_map.header.width = msg->ogmwidth;
  tem_map.header.height = msg->ogmheight;
  tem_map.header.resolution = msg->resolution;
  tem_map.header.pose_index_x = msg->vehicle_x;
  tem_map.header.pose_index_y = msg->vehicle_y;
  tem_map.data.assign(tem_map.header.width * tem_map.header.height, 0);
  MapData tem_map2 = tem_map;

  for (int i = 0;i < msg->data.size();i++){
    if (msg->data[i] == 1){
      tem_map.data[i] = msg->data[i];
      tem_map2.data[i] = msg->data[i];
    }
    else if (msg->data[i] == 5){//悬崖
      tem_map.data[i] = msg->data[i];
    }
    else if (msg->data[i] == 6){//水
      tem_map2.data[i] = msg->data[i];
    }
  }
  // 将悬崖的障碍物地图放入 roadblock_data 和 roadblock_data2 里面去
  {
    ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
    roadblock_data[DataType::KStiff].push_back(tem_map);//只存1~2帧数据
    roadblock_data2[DataType::KStiff].push_back(tem_map);
    //		roadblock_data2[DataType::KWater].push_back(tem_map);
    while (roadblock_data[DataType::KStiff].size() > kBufferSize){
      LOG(WARNING) << " KStiff Obeject isn't processed timely,dropped stamp:" << roadblock_data[DataType::KStiff].front().header.time;
      roadblock_data[DataType::KStiff].pop_front();
      roadblock_data[DataType::KStiff].pop_front();
    }
  }
  CommonObjectDispatch(DataType::KStiff);
  // 将水的障碍物地图放入 roadblock_data 和 roadblock_data2 里面去
  {
    ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
    roadblock_data[DataType::KWater].push_back(tem_map2);
    //		roadblock_data2[DataType::KWater].push_back(tem_map);
    while (roadblock_data[DataType::KWater].size() > kBufferSize){
      LOG(WARNING) << " KStiff Obeject isn't processed timely,dropped stamp:" << roadblock_data[DataType::KWater].front().header.time;
      roadblock_data[DataType::KWater].pop_front();
      roadblock_data[DataType::KWater].pop_front();
    }
  }
  //     LOG(INFO)<<"it is going to dispatch: DataType::KStiff";

  CommonObjectDispatch(DataType::KWater);
}

void Node::HandleUnevenAreaObjectMessage(const uneven_area_msgs::HeightMap::ConstPtr& msg){
  if (!traversable_area_inited){
    // LOG(INFO) << "traversable_area_inited didn`t finished!";
    return;
  }
  //       ::ivcommon::MutexLocker lock(&mutex);
  if (global_origin_pose.translation().x() == 0 || global_origin_pose.translation().y() == 0){
    // LOG(WARNING) << "global_origin_pose msg Zero";
    return;
  }
  if (msg->ogmwidth <= 0 || msg->ogmheight <= 0 || msg->vehicle_x < 0 || msg->vehicle_y < 0 || msg->header.stamp.toSec() <= 0){
    LOG(ERROR) << "HandleUnevenAreaObjectMessage msg  is invalid!";
    return;
  }


  MapData tem_map;
  tem_map.header.time = ::ivcommon::FromRos(msg->header.stamp);
  tem_map.header.width = msg->ogmwidth;
  tem_map.header.height = msg->ogmheight;
  tem_map.header.resolution = msg->ogmresolution;
  tem_map.header.pose_index_x = msg->vehicle_x;
  tem_map.header.pose_index_y = msg->vehicle_y;
  tem_map.data.assign(tem_map.header.width * tem_map.header.height, 0);

  for (int i = 0;i < msg->data.size();i++){
    if (i / msg->ogmwidth - msg->vehicle_y >= 0 && i / msg->ogmwidth - msg->vehicle_y <= (int)(7 / msg->ogmresolution) &&
      i % msg->ogmwidth - msg->vehicle_x >= (int)(-2 / msg->ogmresolution) && i % msg->ogmwidth - msg->vehicle_x <= (int)(2 / msg->ogmresolution)
      ){
      continue;
    }
    if (i / msg->ogmwidth - msg->vehicle_y >= (int)(15 / msg->ogmresolution)){
      continue;
    }
    if (msg->data[i] > 0 && msg->data[i] <= 0.15) tem_map.data[i] = 9;
    if (msg->data[i] > 0.15 && msg->data[i] <= 1) tem_map.data[i] = 1;
  }
  // 将非平坦区域的障碍物地图放入 roadblock_data 和 roadblock_data2 里面去
  {
    ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
    roadblock_data[DataType::KUnevenArea].push_back(tem_map);
    roadblock_data2[DataType::KUnevenArea].push_back(tem_map);
    while (roadblock_data[DataType::KUnevenArea].size() > kBufferSize){
      LOG(WARNING) << " KUnevenArea Obeject isn't processed timely,dropped stamp:" << roadblock_data[DataType::KUnevenArea].front().header.time;
      roadblock_data[DataType::KUnevenArea].pop_front();
      roadblock_data[DataType::KUnevenArea].pop_front();
    }
  }
  //      LOG(INFO)<<"it is going to dispatch: DataType::KUnevenArea";
  CommonObjectDispatch(DataType::KUnevenArea);

}
void Node::HandleLidarOdometryMessage(const covgrid_slam_msgs::LidarOdometryForMapping::ConstPtr& msg){
  //   if(traversable_area_inited==false){
  //     return;
  //   }

  if (global_origin_pose.translation().x() == 0 || global_origin_pose.translation().y() == 0){
    // LOG(WARNING) << "global_origin_pose msg Zero";
    return;
  }
  if (msg->odometry.header.stamp.toSec() <= 0 || msg->odometry.pose.pose.position.x == 0){
    LOG(ERROR) << "HandleLidarOdometryMessage msg  is invalid!";
    return;
  }
  LOG(INFO) << "LidarOdometryForMapping time:" << (ros::Time::now() - msg->gps.header.stamp).toSec();
  LidarOdometryData temp_odom;
  temp_odom.time = ::ivcommon::FromRos(msg->odometry.header.stamp);
  if (temp_odom.time < map_init_time) return;
  temp_odom.mode = msg->mode;
  temp_odom.pose = global_origin_pose.inverse() * ::ivcommon::transform::Rigid3d(Eigen::Vector3d(msg->odometry.pose.pose.position.x, msg->odometry.pose.pose.position.y,
    msg->odometry.pose.pose.position.z), Eigen::Quaternion<double>(msg->odometry.pose.pose.orientation.w, msg->odometry.pose.pose.orientation.x,
      msg->odometry.pose.pose.orientation.y, msg->odometry.pose.pose.orientation.z));//将位姿转换成针对于第一帧坐标系的位姿local pose //odometry.pose.pose.都是车辆相对于地球坐标系的
  temp_odom.GPS = Eigen::Vector3d(msg->gps.longitude, msg->gps.latitude, msg->gps.altitude);
  for (int i = 0; i < msg->indexs.size();i++){
    temp_odom.indexs.push_back(msg->indexs[i]);
  }
  {
    ::ivcommon::MutexLocker lock(&mutex_lidarodometry);
    lidar_odometry_data.push_back(temp_odom);//只存8~10帧数据
    while (lidar_odometry_data.size() > 10){
      lidar_odometry_data.erase(lidar_odometry_data.begin(), lidar_odometry_data.begin() + 2);//删除从first到last之间的字符
    }
  }
  //[=]这是什么意思zhubc
  boost::thread common_thread(
    [=](){
      ::ivcommon::MutexLocker lock(&mutex_roadblock_data);
      for (auto it = roadblock_data.begin();it != roadblock_data.end();it++){
        if (it->second.size() > 0){
          boost::thread common_dispatch_thread(boost::bind(&Node::CommonObjectDispatch, this, it->first));
          common_dispatch_thread.detach();//detach是使主线程不用等待子线程可以继续往下执行，但即使主线程终止了，子线程也不一定终止。
        }
      }
    }
  );
  common_thread.detach();
}

void Node::HandleGPSMsg(const sensor_driver_msgs::GpswithHeading::ConstPtr& gps_msg) {
    std::vector<double> tmp_xy{gps_msg->gps.longitude, gps_msg->gps.latitude};
    gps_deque.push_back(tmp_xy);
    while (gps_deque.size() > 2) {
        gps_deque.pop_front();
    }
}

int Node::PriormapMatchBasedOnLocation(int last_map_idx) {
    if (gps_deque.empty()) {
        LOG(WARNING) << "[PriormapMatchBasedOnLocation] no gps received";
        return -1;
    }
    if (priormap_xy.empty()) {
        LOG(WARNING) << "[PriormapMatchBasedOnLocation] no priormap headers";
        return -1;
    }
    std::vector<double> gps_xy = gps_deque.back();
    // LidarOdometryData tem_msg = new LidarOdometryData();
    double x,y;
    ::ivcommon::transform::geographic_to_grid(
      a, e2, gps_xy[1]*M_PI/180,  gps_xy[0]*M_PI/180, &gps_zone, &hemi, &y, &x);
    x -= 500000;
    // std::cout << "[PriormapMatchBasedOnLocation] x: " << x << ", y: " << y << std::endl;

    float min_distance = FLT_MAX;
    int match_index = -1;
    // use pose to calculate distance
    if (last_map_idx != -1) {
      for (int i=std::max(0, last_map_idx-5); i<std::min(last_map_idx+5, (int)(priormap_xy.size())); i++) {
          float temp_distance = poseSquareDistance(x,y, priormap_xy[i]);
          if ((temp_distance > 0) && (temp_distance < min_distance)) {
              min_distance = temp_distance;
              match_index = i;
          }
      }
    }
    if (match_index == -1 || min_distance > 600.0) {
      for (int i=0; i<priormap_xy.size(); i++) {
          // tem_msg.GPS = Eigen::Vector3d(msg->gps.longitude,msg->gps.latitude,msg->gps.altitude);
          // tem_msg.pose = global_origin_pose.inverse()*transform::Rigid3d(
          //     Eigen::Vector3d(msg->odometry.pose.pose.position.x,msg->odometry.pose.pose.position.y,
          //     msg->odometry.pose.pose.position.z),Eigen::Quaternion<double>(
          //     msg->odometry.pose.pose.orientation.w,msg->odometry.pose.pose.orientation.x,
          float temp_distance = poseSquareDistance(x,y, priormap_xy[i]);
          if ((temp_distance > 0) && (temp_distance < min_distance)) {
              min_distance = temp_distance;
              match_index = i;
          }
      }
    }
    if (match_index >= 0) 
      printf("GPS-based map match: the %dth map. min_distance: %f\n", match_index, std::sqrt(min_distance));
    return match_index;
}

float Node::poseSquareDistance(double x, double y, std::vector<double> &b) {
    return std::pow(x - b[0], 2) + std::pow(y-b[1], 2);
}

void Node::HandlePrimaryTraversableAreaMessage(const iv_slam_ros_msgs::PrimarytraversableArea::ConstPtr& msg){
  // clock_t start_time = clock();
  //准可通行区域消息处理，主要提供的是地图的位姿以及先验地图模式的index
  static auto last_time = ::ivcommon::FromRos(msg->header.stamp);//只是第一次进来的时候会初始化，之后再进这个回调都不会执行这个了。
  static int last_map_idx = -1;
  auto now_time = ::ivcommon::FromRos(msg->header.stamp);
  if (now_time <= last_time){
    LOG(WARNING) << "HandlePrimaryTraversableAreaMessage now_time <= last_time: "
      << now_time << " vs " << last_time;
    return;
  }
  last_time = now_time;
  MapData tem_map;
  ConvertFromMsg2Mapdata(msg, tem_map);//主要涉及到的是坐标系转换，将triD的坐标系进行的转换，然后是赋值  基于第一帧定位为坐标系的位姿，即为local pose
  // primary_traversable_area_img = cv::Mat::zeros(cv::Size(tem_map.header.width, tem_map.header.height), CV_8UC3);
  // int data_idx = 0;
  // for (int i=0; i<tem_map.header.height; i++) {
  //   for (int j=0; j<tem_map.header.width; j++) {
  //     if (tem_map.data[data_idx] == 1) {
  //       primary_traversable_area_img.at<cv::Vec3b>(i,j)[0] = 0;
  //       primary_traversable_area_img.at<cv::Vec3b>(i,j)[1] = 255;
  //       primary_traversable_area_img.at<cv::Vec3b>(i,j)[2] = 0;
  //     }
  //     else if (tem_map.data[data_idx] == 2) {
  //       primary_traversable_area_img.at<cv::Vec3b>(i,j)[0] = 0;
  //       primary_traversable_area_img.at<cv::Vec3b>(i,j)[1] = 0;
  //       primary_traversable_area_img.at<cv::Vec3b>(i,j)[2] = 255;
  //     }
  //     data_idx ++;
  //   }
  // }
  // cv::flip(primary_traversable_area_img, primary_traversable_area_img, 0);
  if (msg->prirormapmode) // 0是正在建图，1是正在切图，2是先验模式
  {
    ::ivcommon::MutexLocker lock(&mutex_priormap_write);
    traversable_area_inited = false;
    if (traversable_area_option_.priormap_write){
      PriorMapWriter();//生成并保存先验地图
    }
    // LOG(WARNING) << "going to run PriorMapProcessor";
    ROS_INFO_THROTTLE(3, "going to run PriorMapProcessor");
    priormap_globalvalue.priormapmode_opened = true;
    int tem_index = msg->index;
    PriorMapProcessor(tem_index, tem_map);
    return;
  }
  else if(traversable_area_option_.load_priormap && 
      use_gps_to_match_when_lidarodometry_invalid) {
    priormap_globalvalue.priormapmode_opened = true;
    int tem_index = PriormapMatchBasedOnLocation(last_map_idx);
    last_map_idx = tem_index;
    // printf("GPS based map match - the %dth map\n", tem_index);
    if (tem_index >= 0) {
      PriorMapProcessor(tem_index, tem_map);
      return;
    }
  }
  if (msg->width <= 0 || msg->height <= 0 || msg->header.stamp.toSec() < 0){
    LOG(ERROR) << "PrimatyTraversableAreaMessage msg is invalid!";
    return;
  }
  if (priormap_globalvalue.priormapmode_opened){
    ::ivcommon::MutexLocker lock_pending_pub_data(&mutex_pending_pub_data);
    traversable_area_inited = false;
    roadblock_data.clear();
    for (int i = 0;i < kObstacletypes;i++){
      ::ivcommon::MutexLocker lock_eachtype(&mutexs_eachtype[i]);
      traversable_area_data[DataType(i)] = TraversableAreaData();
      pending_pub_traversable_area_data[DataType(i)] = TraversableAreaData();
    }

    priormap_globalvalue.priormapmode_opened = false;
    cv::destroyWindow("Priormap");
    last_priormapindex = -1;
    integrate_current_primarymapdata = -1;
  }
  // LOG(WARNING)<<"HandlePrimaryTraversableAreaMessage runing! "<<traversable_area_inited;
  static int zone_init_index = 0;
  if (zone_init_index == 0){
    double N, E;
    ::ivcommon::transform::geographic_to_grid(a, e2, msg->triD_submap_pose.position.y * M_PI / 180, msg->triD_submap_pose.position.x * M_PI / 180, &gps_zone, &hemi, &N, &E);
    LOG(INFO) << "traversable area zone:" << gps_zone;
    zone_init_index++;
  }

  TraversableAreaProcessor(tem_map);
  //准可通行区域提供了最终可通行区域里面的一些基本信息，比如说位姿及尺寸。
  // clock_t end_time = clock();
  // ROS_INFO("HandlePrimaryTraversableAreaMessage time cost: %f ms", (double)(end_time - start_time) / CLOCKS_PER_SEC * 1000.0);
}

//<将接收到的PrimarytraversableArea地图消息转换为通用地图格式
void Node::ConvertFromMsg2Mapdata(const iv_slam_ros_msgs::PrimarytraversableArea::ConstPtr& msg, MapData& tem_map){
  tem_map.header.time = ::ivcommon::FromRos(msg->header.stamp);
  tem_map.header.resolution = msg->resolution;
  tem_map.header.index = msg->index;

  tem_map.header.finished = msg->submap_finished_flag;//该可通行区域子地图是否创建完成
  LOG(INFO) << " msg->index: " << msg->index <<
    " tem_map.header.index: " << tem_map.header.index <<
    " tem_map.header.finished: " << tem_map.header.finished;
  // triD_submap_pose的是子地图中里程计的位姿
  tem_map.header.pose_index_x = msg->triD_submap_pose_image_index_x;//该可通行区域对应的子地图位姿在子地图可通行区域数据中对应的ｘ向索引指数(应该是车辆位置)
  tem_map.header.pose_index_y = msg->triD_submap_pose_image_index_y;//该可通行区域对应的子地图位姿在子地图可通行区域数据中对应的y向索引指数
  tem_map.header.width = msg->width;
  tem_map.header.height = msg->height;
  //use_location_module对应的是zzh那边综合定位的位姿
  tem_map.header.use_location_module = msg->use_location_module;//子地图位姿，经纬高分别对应xyz
  if (tem_map.header.use_location_module && (msg->location_module_triD_submap_pose.position.x < 0 || msg->location_module_triD_submap_pose.position.y < 0)){
    LOG(ERROR) << "PrimatyTraversableAreaMessage location module msg  is invalid!";
  }

  double tem_x(0), tem_y(0), tem_location_x(0), tem_location_y(0);
  // ::ivcommon::transform::GridZone  tem_zone = ::ivcommon::transform::UTM_ZONE_AUTO;
  //将经纬度转换成距离,rad转换成m  zmr??
  ::ivcommon::transform::geographic_to_grid(a, e2, msg->triD_submap_pose.position.y * M_PI / 180, msg->triD_submap_pose.position.x * M_PI / 180, &gps_zone, &hemi, &tem_y, &tem_x);
  //tem_map.pose是基于第一帧定位为坐标系的位姿，即为local pose
  tem_map.header.pose = global_origin_pose.inverse() * ::ivcommon::transform::Rigid3d(Eigen::Vector3d(tem_x - 500000, tem_y, msg->triD_submap_pose.position.z),
    Eigen::Quaternion<double>(msg->triD_submap_pose.orientation.w, msg->triD_submap_pose.orientation.x, msg->triD_submap_pose.orientation.y, msg->triD_submap_pose.orientation.z));
  //对global_origin_pose进行初始化
  if ((!traversable_area_option_.load_priormap) && global_origin_pose.translation().x() == 0 && global_origin_pose.translation().y() == 0){
    global_origin_pose = tem_map.header.pose;
    tem_map.header.pose = global_origin_pose.inverse() * tem_map.header.pose;//变成单位阵了.
    LOG(INFO) << "global_origin_pose initialized to " << global_origin_pose;
  }

  LOG(INFO) << "msg->location_module_triD_submap_pose.position.y" << msg->location_module_triD_submap_pose.position.y;
  ::ivcommon::transform::geographic_to_grid(a, e2, 
    msg->location_module_triD_submap_pose.position.y * M_PI / 180, 
    msg->location_module_triD_submap_pose.position.x * M_PI / 180, 
    &gps_zone, &hemi, &tem_location_y, &tem_location_x);
  tem_map.header.location_module_pose = global_origin_pose.inverse() * ::ivcommon::transform::Rigid3d(Eigen::Vector3d(tem_location_x - 500000, tem_location_y, msg->location_module_triD_submap_pose.position.z),
    Eigen::Quaternion<double>(msg->location_module_triD_submap_pose.orientation.w, msg->location_module_triD_submap_pose.orientation.x, msg->location_module_triD_submap_pose.orientation.y, msg->location_module_triD_submap_pose.orientation.z));

  tem_map.data.assign(tem_map.header.width * tem_map.header.height, 0);
  //将准可通行区域的值赋给tem_map.data    0为未知或可通行，２为不可通行
  for (int i = 0;i < msg->cells.size();i++){
    if (msg->cells[i] > 0) tem_map.data[i] = msg->cells[i];
  }
}

void Node::HandleTraversableAreaVehiclePose(const iv_slam_ros_msgs::Traversablevehiclepose::ConstPtr& msg){
  // 	if (traversable_area_inited == false) {
  // 		return;
  // 	}

  if (msg->header.stamp.toSec() < 0){
    LOG(ERROR) << "HandleTraversableAreaVehiclePose msg error";
    return;
  }
  if (global_origin_pose.translation().x() == 0 || global_origin_pose.translation().y() == 0){
    // LOG(WARNING) << "global_origin_pose msg Zero";
    return;
  }

  LidarOdometryData tem_latest_vechicle_pose = latest_vechicle_pose;//如果是第一次进来的话，主要给的是init()里面初始化的时间
  ::ivcommon::Time tem_time = ::ivcommon::FromRos(msg->header.stamp);
  if (tem_time > tem_latest_vechicle_pose.time){
    tem_latest_vechicle_pose.time = tem_time;
    double tem_x(0), tem_y(0);
    LOG(INFO) << "msg->primary_submap_vehicle_pose.pose.pose.position.x:" << msg->primary_submap_vehicle_pose.pose.pose.position.x <<
      "msg->primary_submap_vehicle_pose.pose.pose.position.y" << msg->primary_submap_vehicle_pose.pose.pose.position.y;
    // ::ivcommon::transform::GridZone tem_zone = ::ivcommon::transform::UTM_ZONE_AUTO;
    ::ivcommon::transform::geographic_to_grid(a, e2,
      msg->primary_submap_vehicle_pose.pose.pose.position.y * M_PI / 180,
      msg->primary_submap_vehicle_pose.pose.pose.position.x * M_PI / 180,
      &gps_zone, &hemi, &tem_y, &tem_x);
    tem_latest_vechicle_pose.pose = global_origin_pose.inverse() *
      ::ivcommon::transform::Rigid3d(Eigen::Vector3d(tem_x - 500000, tem_y, msg->primary_submap_vehicle_pose.pose.pose.position.z),
        Eigen::Quaternion<double>(
          msg->primary_submap_vehicle_pose.pose.pose.orientation.w,
          msg->primary_submap_vehicle_pose.pose.pose.orientation.x,
          msg->primary_submap_vehicle_pose.pose.pose.orientation.y,
          msg->primary_submap_vehicle_pose.pose.pose.orientation.z));//车辆位姿变换，都是相对于第一帧里程计位姿
  }
  {
    ::ivcommon::MutexLocker lock(&mutex_latest_vechicle_pose);
    latest_vechicle_pose = tem_latest_vechicle_pose; //jkj 0728   和lidar_odometry_data都在更新latest_vechicle_pose zmr??
  }
}

}//namespace traversable_area_extraction
