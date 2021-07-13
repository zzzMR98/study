#ifndef TRAVERSaBLE_AREA_NODE_OPTION_H
#define TRAVERSaBLE_AREA_NODE_OPTION_H
#include <string>
#include "ivcommon/common/configuration_file_resolver.h"
#include "ivcommon/common/lua_parameter_dictionary.h"
#include "ivcommon/common/make_unique.h"
namespace traversable_area_extraction{
  
struct TraversableAreaOption{
  bool integrate_negativeobeject;///<是否融合正负障碍
  bool integtate_stiffobeject;///<是否融合悬崖
  bool integtate_slopeobeject;///<是否融合斜坡
  bool integtate_unevenareaobeject; ///<是否融合非平坦区域
  bool integtate_mapping_traversable_area;///<是否融合准通行区域正障碍
  bool wiping_dynamicobejectflag;///<是否提出动态障碍物

  std::string map_frame;
  std::string tracking_frame;
  double traversable_area_publish_period_sec;
  double vehicle_width;
  double known_radius;
  bool display_on;
  bool load_priormap ;///<是否加载先验地图，此选项用于使用先验地图时
  bool wipe_history_obstacle; // 当前检测到某处无障碍物时是否抹去该位置的历史障碍物信息
  bool priormap_write;///<是否创建先验地图
  std::string priormap_file_name_time;///<加载先验地图时，先验地图所在文件夹名称
};
 TraversableAreaOption CreateTraversableAreaOption(  ::ivcommon::LuaParameterDictionary* const lua_parameter_dictionary);
 
TraversableAreaOption LoadOptions( const string& configuration_directory,const string& configuration_basename);///<加载配置文件




}//namespace traversable_area_extraction
#endif
