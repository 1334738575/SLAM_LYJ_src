#ifndef SLAM_LYJ_SRC_DEFINES_H
#define SLAM_LYJ_SRC_DEFINES_H


#ifdef WIN32
#ifdef _MSC_VER
#define SLAM_LYJ_SRC_API __declspec(dllexport)
#else
#define SLAM_LYJ_SRC_API
#endif
#else
#define SLAM_LYJ_SRC_API
#endif



namespace SLAM_LYJ_src
{



}

#endif//SLAM_LYJ_src_DEFINES_H