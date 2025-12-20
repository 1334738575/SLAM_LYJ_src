#include "SLAM_LYJ_src_Include.h"
#include <processor/processorVP.h>
#include <processor/processorWithMesh.h>



namespace SLAM_LYJ_src
{


SLAM_LYJ_SRC_API void print_SLAM_LYJ_src_Test()
{
    printf("Hello SLAM_LYJ_src!");
}

SLAM_LYJ_SRC_API void reconstructVisualPoint(ProcessOption _opt)
{
    _opt.imageExtractOpt.pointExtractMode = 0;
    _opt.imageExtractOpt.usePointFeature = true;
    _opt.imageExtractOpt.useLineFeature = true;
    _opt.imageExtractOpt.useEdgeFeature = true;
    _opt.imageMatchOpt.pointMatchMode = 0;
    //_opt.imageMatchOpt.pointMatchCheck = true;
    _opt.imageMatchOpt.usePointMatch = true;
    _opt.imageMatchOpt.useLineMatch = false;
    _opt.imageMatchOpt.useEdgeMatch = false;
    _opt.imageMatchOpt.usePatchMatch = false;
    ProcessorVP pro;
    pro.setData(_opt);
    if (_opt.readCache) {
        pro.readData("D:/tmp/processDebug");
    }
    else {
        pro.extractFeature();
        pro.matchFeature();
        pro.generateCorrespondGraph(false);
    }
    //pro.writeData("D:/tmp/processDebug");
    //pro.incrementalMap();
    //pro.generateMap();
    //pro.optimize();
    return;
}

SLAM_LYJ_SRC_API void reconstructVisualWithMesh(ProcessOption _opt)
{
    _opt.imageExtractOpt.pointExtractMode = 0;
    _opt.imageExtractOpt.usePointFeature = true;
    _opt.imageExtractOpt.useLineFeature = true;
    _opt.imageExtractOpt.useEdgeFeature = true;
    _opt.imageMatchOpt.pointMatchMode = 0;
    _opt.imageMatchOpt.pointMatchCheck = true;
    _opt.imageMatchOpt.squareDThInMesh= 0.04f;
    _opt.imageMatchOpt.usePointMatch = true;
    _opt.imageMatchOpt.useLineMatch = false;
    _opt.imageMatchOpt.useEdgeMatch = false;
    _opt.imageMatchOpt.usePatchMatch = false;
    ProcessorWithMesh pro;
    pro.setData(_opt);
    if (_opt.readCache) {
        pro.readData("D:/tmp/processDebug");
    }
    else {
        pro.extractFeature();
        pro.matchFeature();
        pro.generateCorrespondGraph();
    }
    pro.writeData("D:/tmp/processDebug");
    pro.generateMap();
    pro.optimize();
    return;
}



}