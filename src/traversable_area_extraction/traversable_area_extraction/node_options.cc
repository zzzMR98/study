#include "node_options.h"

namespace traversable_area_extraction {

    TraversableAreaOption CreateTraversableAreaOption(
            ::ivcommon::LuaParameterDictionary* const
            lua_parameter_dictionary) {
      TraversableAreaOption options;
      options.integrate_negativeobeject = lua_parameter_dictionary->GetBool("integrate_negativeobeject");
      options.integtate_slopeobeject = lua_parameter_dictionary->GetBool("integtate_slopeobeject");
      options.integtate_stiffobeject = lua_parameter_dictionary->GetBool("integtate_stiffobeject");
      options.integtate_unevenareaobeject = lua_parameter_dictionary->GetBool("integtate_unevenareaobeject");
      options.integtate_mapping_traversable_area = lua_parameter_dictionary->GetBool("integtate_mapping_traversable_area");
      options.wiping_dynamicobejectflag = lua_parameter_dictionary->GetBool("wiping_dynamicobejectflag");
      options.map_frame = lua_parameter_dictionary->GetString("map_frame");
      options.tracking_frame = lua_parameter_dictionary->GetString("tracking_frame");
      options.traversable_area_publish_period_sec = lua_parameter_dictionary->GetDouble("traversable_area_publish_period_sec");
      options.vehicle_width = lua_parameter_dictionary->GetDouble("vehicle_width");
      options.known_radius = lua_parameter_dictionary->GetDouble("known_radius");
      options.display_on = lua_parameter_dictionary->GetBool("display_on");
      options.wipe_history_obstacle= lua_parameter_dictionary->GetBool("wipe_history_obstacle");
      options.load_priormap= lua_parameter_dictionary->GetBool("load_priormap");
      options.priormap_write = lua_parameter_dictionary->GetBool("priormap_write");
      options.priormap_file_name_time =lua_parameter_dictionary->GetString("priormap_file_name_time");

      return options;
    }


    TraversableAreaOption LoadOptions(
            const string& configuration_directory,
            const string& configuration_basename)
    {
      auto file_resolver = ::ivcommon::make_unique<  //make_unique智能指针
              ::ivcommon::ConfigurationFileResolver>(
              std::vector<string>{configuration_directory});//设置目录
      const string code =
              file_resolver->GetFileContentOrDie(configuration_basename);//获取文件完整内容
      ::ivcommon::LuaParameterDictionary lua_parameter_dictionary(
              code, std::move(file_resolver));//std::move将一个左值强制转化为右值引用

      return CreateTraversableAreaOption(&lua_parameter_dictionary);
    }


}//namespace traversable_area_extraction
