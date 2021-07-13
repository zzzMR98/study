#ifndef FILE_DIRECTORY_GENETATION
#define FILE_DIRECTORY_GENETATION
//#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
//#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <unistd.h>
#include <time.h>

namespace ivcommon
{
  /***********lzz171029**********
     * output file_name_ ,which is the final name you want to generate,
     * which would be "~/SLAM_DATA/system_time/module_name_/node_name_"
     * input module_name_;
     * input  node_name_;
     * time_t t = time(0); 
    char timename[32]="";
    strftime(timename, sizeof(timename), "%Y%m%d_%H%M%S",localtime(&t));
     */
  // std::string expand_user(std::string path);
  std::string file_directory_generate(std::string time_, std::string module_name_);
} // namespace ivcommon

#endif
