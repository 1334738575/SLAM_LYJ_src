#include "SLAM_LYJ_src_Include.h"
#include <processor/processorVP.h>
#include <processor/processorWithMesh.h>
#include <processor/processorWithMeshCom.h>
#include <IO/BaseIO.h>


namespace SLAM_LYJ_src
{


SLAM_LYJ_SRC_API void print_SLAM_LYJ_src_Test()
{
    printf("Hello SLAM_LYJ_src!");
}

SLAM_LYJ_SRC_API void reconstructVisualPoint(ProcessOption _opt)
{
    //_opt.imageExtractOpt.pointExtractMode = 0;
    //_opt.imageExtractOpt.usePointFeature = true;
    //_opt.imageExtractOpt.useLineFeature = true;
    //_opt.imageExtractOpt.useEdgeFeature = true;
    //_opt.imageMatchOpt.pointMatchMode = 0;
    ////_opt.imageMatchOpt.pointMatchCheck = true;
    //_opt.imageMatchOpt.usePointMatch = true;
    //_opt.imageMatchOpt.useLineMatch = false;
    //_opt.imageMatchOpt.useEdgeMatch = false;
    //_opt.imageMatchOpt.usePatchMatch = false;
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
    if(_opt.dumpPath != "")
        pro.writeData(_opt.dumpPath);
    //pro.incrementalMap();
    //pro.generateMap();
    //pro.optimize();
    return;
}

SLAM_LYJ_SRC_API void reconstructVisualWithMesh(ProcessOption _opt)
{
    //_opt.imageExtractOpt.pointExtractMode = 0;
    //_opt.imageExtractOpt.usePointFeature = true;
    //_opt.imageExtractOpt.useLineFeature = false;
    //_opt.imageExtractOpt.useEdgeFeature = false;
    //_opt.imageMatchOpt.pointMatchMode = 0;
    //_opt.imageMatchOpt.pointMatchCheck = true;
    //_opt.imageMatchOpt.squareDThInMesh = 900;// 0.04f;
    //_opt.imageMatchOpt.usePointMatch = true;
    //_opt.imageMatchOpt.useLineMatch = false;
    //_opt.imageMatchOpt.useEdgeMatch = false;
    //_opt.imageMatchOpt.usePatchMatch = false;
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
    if (_opt.dumpPath != "")
        pro.writeData(_opt.dumpPath);
    //pro.generateMap();
    //pro.optimize();
    return;
}

SLAM_LYJ_SRC_API void reconstructVisualWithMeshCom(COMMON_LYJ::BaseTriMesh& _btm, std::vector<COMMON_LYJ::CompressedImage*>& _imgs, std::vector<COMMON_LYJ::Pose3D>& _Tcws, std::vector<COMMON_LYJ::PinholeCamera>& _cams, ProcessComOption _opt)
{
    using namespace COMMON_LYJ;
    //COMMON_LYJ::writeBinFile<BaseTriMesh&, std::vector<CompressedImage*>&, std::vector<Pose3D>&, std::vector<PinholeCamera>&, ProcessComOption>("D:/tmp/reconstructVisualWithMeshCom.bin", _btm, _imgs, _Tcws, _cams, _opt);
    //COMMON_LYJ::writeBinFile<BaseTriMesh&, std::vector<CompressedImage*>&, std::vector<Pose3D>&, std::vector<PinholeCamera>&>("D:/tmp/reconstructVisualWithMeshComView.bin", _btm, _imgs, _Tcws, _cams);
    ProcessorWithMeshCom processor;
    processor.process(_btm, _imgs, _Tcws, _cams, _opt);
    return;
}



}