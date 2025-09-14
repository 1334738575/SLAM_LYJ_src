#include "processorVP.h"
#include "STLPlus/include/file_system.h"
#include <ImageProcess_LYJ_Include.h>

NSP_SLAM_LYJ_SRC_BEGIN


ProcessorVP::ProcessorVP(/* args */)
{}
ProcessorVP::~ProcessorVP()
{}

void ProcessorVP::setData(const ProVPOpt & _opt)
{
    opt_ = _opt;
    if (!stlplus::folder_exists(opt_.imgDir))
    {
        std::cout << "image directory is not exist!" << std::endl;
        return;
    }
    if (opt_.imgNames.empty())
    {
        opt_.imgNames = stlplus::folder_files(opt_.imgDir);
        if (opt_.imgNames.empty()) {
            std::cout << "image directory has no image!" << std::endl;
            return;
        }
    }
    if (!stlplus::file_exists(opt_.camFile))
    {
        std::cout << "camera file is not exist!" << std::endl;
        return;
    }
    if (!stlplus::folder_exists(opt_.priTcwDir))
    {
        std::cout << "prior Tcw directory is not exist!" << std::endl;
        return;
    }
    if (opt_.priTcwNames.empty())
    {
        opt_.priTcwNames = stlplus::folder_files(opt_.priTcwDir);
        if (opt_.priTcwNames.empty()) {
            std::cout << "prior Tcw directory has no image!" << std::endl;
            return;
        }
    }
}
bool ProcessorVP::extractFeature()
{
    if (opt_.imgNames.empty())
        return false;
    int imgSize = opt_.imgNames.size();
    COMMON_LYJ::readPinCamera(opt_.camFile, cam_);
    for (int i = 0; i < imgSize; ++i)
    {
        std::string imgFile = stlplus::create_filespec(opt_.imgDir, opt_.imgNames[i]);
        imageExtractDatas_[i].reset(new ImageProcess_LYJ::ImageExtractData());
        imageExtractDatas_[i]->id = i;
        imageExtractDatas_[i]->usePointFeature = true;
        imageExtractDatas_[i]->useLineFeature = false;
        imageExtractDatas_[i]->useEdgeFeature = false;
        imageExtractDatas_[i]->path = imgFile;
        imageExtractDatas_[i]->img = cv::imread(imgFile, 0);
		if (imageExtractDatas_[i]->img.empty())
		{
			std::cout << "read image failed: " << imgFile << std::endl;
			continue;
		}
		std::cout << "read image success: " << imgFile << std::endl;
        ImageProcess_LYJ::extractFeature(imageExtractDatas_[i].get());
        imageExtractDatas_[i]->cam = &cam_;
        std::string priTcwFile = stlplus::create_filespec(opt_.priTcwDir, opt_.priTcwNames[i]);
        COMMON_LYJ::readT34(priTcwFile, imageExtractDatas_[i]->Tcw);
    }
    return true;
}
bool ProcessorVP::matchFeature()
{
    int imgSize = imageExtractDatas_.size();
    if (imgSize < 2)
        return false;
    uint64_t key;
    ImageProcess_LYJ::ImageMatchOption opt;
    opt.pointMatchMode = 7;
    opt.pointMatchCheck = true;
    for (int i = 0; i < imgSize - 1; ++i)
    {
        for (int j = i + 1; j < i+4; ++j)
        {
            if (j >= imgSize)
                continue;
            ImageProcess_LYJ::site2Key(i, j, key);
			//uint64_t key = ((uint64_t)i << 32) | (uint64_t)j;
			if (imageMatchDatas_.find(key) != imageMatchDatas_.end())
				continue;
            std::cout << "match image " << i << " and image " << j << std::endl;
			imageMatchDatas_[key].reset(new ImageProcess_LYJ::ImageMatchData());
			imageMatchDatas_[key]->usePointMatch = true;
			imageMatchDatas_[key]->useLineMatch = false;
			imageMatchDatas_[key]->useEdgeMatch = false;
            imageMatchDatas_[key]->id1 = i;
            imageMatchDatas_[key]->id2 = j;
			imageMatchDatas_[key]->debugPath = "D:/tmp/imageProcess/match/";
			imageMatchDatas_[key]->pointMatchSize = ImageProcess_LYJ::matchFeature(imageExtractDatas_[i].get(), imageExtractDatas_[j].get(), imageMatchDatas_[key].get(), opt);
        }
    }
    return true;
}
bool ProcessorVP::generateCorrespondGraph()
{
    if (imageExtractDatas_.empty() && imageMatchDatas_.empty())
        return false;
    correspondGraph_.reset(new ImageProcess_LYJ::CorrespondGraph());
    correspondGraph_->generate(imageExtractDatas_, imageMatchDatas_);
    return true;
}
bool ProcessorVP::writeData(const std::string& _path)
{
    if (!correspondGraph_->writeData(_path, false))
        return false;
    return true;
}
bool ProcessorVP::readData(const std::string& _path)
{
    correspondGraph_.reset(new ImageProcess_LYJ::CorrespondGraph());
    if (!correspondGraph_->readData(_path))
        return false;
    correspondGraph_->getData(imageExtractDatas_, imageMatchDatas_, cam_);
    return true;
}



NSP_SLAM_LYJ_SRC_END