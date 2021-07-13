#include "common/file_directory_generation.h"

namespace ivcommon
{
    ///
    ///解析＂~＂
    ///
  // std::string expand_user(std::string path)
  // {

  //   if (!path.empty() && path[0] == '~')
  //   {
  //     assert(path.size() == 1 or path[1] == '/'); // or other error handling
  //     char const *home = getenv("HOME");
  //     if (home || ((home = getenv("USERPROFILE"))))
  //     {
  //       path.replace(0, 1, home);
  //     }
  //     else
  //     {
  //       char const *hdrive = getenv("HOMEDRIVE"),
  //                  *hpath = getenv("HOMEPATH");
  //       assert(hdrive); // or other error handling
  //       assert(hpath);
  //       path.replace(0, 1, std::string(hdrive) + hpath);
  //     }
  //   }
  //   return path;
  // }

///
///根据module_name_，自动在home生成该文件夹
///
  std::string file_directory_generate(std::string time_, std::string module_name_)
  {

    std::string file_directory_;
    char const *hpath = getenv("HOME");
    if (hpath == NULL)
      hpath = "";
    file_directory_.clear();
    file_directory_ = std::string(hpath);
    file_directory_ += "/SLAM_DATA";
    boost::filesystem::path dir_base(file_directory_);
    if (!boost::filesystem::exists(dir_base))
    {
      boost::filesystem::create_directory(dir_base);
    }
    file_directory_ += "/";
    file_directory_ += time_;
    boost::filesystem::path dir_base1(file_directory_);
    if (!boost::filesystem::exists(dir_base1))
    {
      boost::filesystem::create_directory(dir_base1);
    }
    file_directory_ += "/";
    file_directory_ += module_name_;
    file_directory_ += "/";
    boost::filesystem::path dir_base2(file_directory_);
    if (!boost::filesystem::exists(dir_base2))
    {
      boost::filesystem::create_directory(dir_base2);
    }
    return file_directory_;
  }
} // namespace ivcommon
