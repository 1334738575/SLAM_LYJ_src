#ifndef SLAM_LYJ_PROCESSORWITHMESH_H
#define SLAM_LYJ_PROCESSORWITHMESH_H

#include "processorVP.h"
#include <CUDAInclude.h>


NSP_SLAM_LYJ_SRC_BEGIN


class ProcessorWithMesh : public ProcessorVP
{
public:
	ProcessorWithMesh();
	~ProcessorWithMesh();

	// ͨ�� processorVP �̳�
	virtual void setData(const ProcessOption& _opt);

	// ͨ�� processorAbr �̳�
	bool extractFeature() override;
	bool generateMap() override;
	bool optimize() override;

private:
	SLAM_LYJ::BaseTriMesh btm_;
	CUDA_LYJ::ProHandle proHandle_ = nullptr;
	CUDA_LYJ::ProjectorCache proCache_;
};



NSP_SLAM_LYJ_SRC_END


#endif // !SLAM_LYJ_PROCESSORWITHMESH_H
