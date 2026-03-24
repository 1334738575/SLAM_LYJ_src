#ifndef SLAM_LYJ_PROCESSORWITHMESHCOMAFT_H
#define SLAM_LYJ_PROCESSORWITHMESHCOMAFT_H


#include "processorWithMeshCom.h"


NSP_SLAM_LYJ_SRC_BEGIN


class ProcessorWithMeshComAft : public ProcessorWithMeshCom
{
public:

	ProcessorWithMeshComAft();
	~ProcessorWithMeshComAft();

	void process(COMMON_LYJ::BaseTriMesh& _btm, std::vector<COMMON_LYJ::CompressedImage*>& _imgs, std::vector<COMMON_LYJ::Pose3D>& _Tcws, std::vector<COMMON_LYJ::PinholeCamera>& _cams, ProcessComOption _opt) override;

protected:
	bool generatePairs() override;
	bool generate3DInfos() override;
	bool optimize(int _i) override;
	bool optimizeCeres() override;

protected:
	std::vector<std::shared_ptr<ObserveDataCom>> observeDatas_;
};



NSP_SLAM_LYJ_SRC_END


#endif //SLAM_LYJ_PROCESSORWITHMESHCOMAFT_H