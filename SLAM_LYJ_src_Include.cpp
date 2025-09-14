#include "SLAM_LYJ_src_Include.h"
#include <processor/processorVP.h>



namespace SLAM_LYJ_src
{


SLAM_LYJ_SRC_API void print_SLAM_LYJ_src_Test()
{
    printf("Hello SLAM_LYJ_src!");
}

SLAM_LYJ_SRC_API void reconstructVisualPoint(const ProVPOpt& _opt)
{
    ProcessorVP pro;
    pro.readData("D:/tmp/processDebug");
    //pro.writeData("D:/tmp/processDebug");
    return;
    pro.setData(_opt);
    pro.extractFeature();
    pro.matchFeature();
    pro.generateCorrespondGraph();
    pro.writeData("D:/tmp/processDebug");
    pro.readData("D:/tmp/processDebug");
    return;
}



}