#ifndef SLAM_LYJ_SRC_INCLUDE_H
#define SLAM_LYJ_SRC_INCLUDE_H


#include "SLAM_LYJ_src_Defines.h"
#include <stdio.h>



namespace SLAM_LYJ_src
{


SLAM_LYJ_SRC_API void print_SLAM_LYJ_src_Test();

SLAM_LYJ_SRC_API void reconstructVisualPoint(ProcessOption _opt = ProcessOption());
SLAM_LYJ_SRC_API void reconstructVisualWithMesh(ProcessOption _opt=ProcessOption());

SLAM_LYJ_SRC_API void reconstructVisualWithMeshCom(COMMON_LYJ::BaseTriMesh& _btm, std::vector<COMMON_LYJ::CompressedImage*>& _imgs, std::vector<COMMON_LYJ::Pose3D>& _Tcws, std::vector<COMMON_LYJ::PinholeCamera>& _cams, ProcessComOption _opt);



}

#endif//SLAM_LYJ_src_INCLUDE_H