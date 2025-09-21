#include "processorVP.h"
#include <ImageProcess_LYJ_Include.h>
#include <debugger/debugger.h>

NSP_SLAM_LYJ_SRC_BEGIN


ProcessorVP::ProcessorVP(/* args */)
{}
ProcessorVP::~ProcessorVP()
{}

void ProcessorVP::setData(const ProcessOption & _opt)
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
    int imgSize = opt_.imgNames.size();
    imageExtractDatasPtr_.resize(imgSize, nullptr);
    for (int i = 0; i < imgSize; ++i)
    {
        imageExtractDatasPtr_[i].reset(new ImageProcess_LYJ::ImageExtractData());
        imageExtractDatas_[i] = imageExtractDatasPtr_[i];
        imageExtractDatasPtr_[i]->id = i;
        std::string imgFile = stlplus::create_filespec(opt_.imgDir, opt_.imgNames[i]);
        imageExtractDatasPtr_[i]->path = imgFile;
    }

    if (!stlplus::file_exists(opt_.camFile))
    {
        std::cout << "camera file is not exist!" << std::endl;
        return;
    }
    COMMON_LYJ::readPinCamera(opt_.camFile, cam_);
    for (int i = 0; i < imgSize; ++i)
    {
        imageExtractDatasPtr_[i]->cam = &cam_;
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
    for (int i = 0; i < imgSize; ++i)
    {
        std::string priTcwFile = stlplus::create_filespec(opt_.priTcwDir, opt_.priTcwNames[i]);
        COMMON_LYJ::readT34(priTcwFile, imageExtractDatasPtr_[i]->Tcw);
    }
}
bool ProcessorVP::extractFeature()
{
    if (opt_.imgNames.empty())
        return false;
    int imgSize = opt_.imgNames.size();
    std::vector<cv::Mat> descs(imgSize);
    for (int i = 0; i < imgSize; ++i)
    {
        imageExtractDatasPtr_[i]->usePointFeature = opt_.imageExtractOpt.usePointFeature;
        imageExtractDatasPtr_[i]->useLineFeature = opt_.imageExtractOpt.useLineFeature;
        imageExtractDatasPtr_[i]->useEdgeFeature = opt_.imageExtractOpt.useEdgeFeature;
        imageExtractDatasPtr_[i]->img = cv::imread(imageExtractDatasPtr_[i]->path, 0);
		if (imageExtractDatasPtr_[i]->img.empty())
		{
			std::cout << "read image failed: " << imageExtractDatasPtr_[i]->path << std::endl;
			continue;
		}
		std::cout << "read image success: " << imageExtractDatasPtr_[i]->path << std::endl;
        ImageProcess_LYJ::extractFeature(imageExtractDatasPtr_[i].get(), opt_.imageExtractOpt);
        descs[i] = imageExtractDatasPtr_[i]->descriptors_;
    }
    voc_.reset(new DBoW3::Vocabulary(9, 3));
    if(!opt_.writeVoc)
        voc_->load(opt_.vocPath);
    else {
        voc_->create(descs);
        if (opt_.vocPath != "")
            voc_->save(opt_.vocPath);
    }
    std::vector<cv::Mat> features;
    features.reserve(2000);
    for (int i = 0; i < imgSize; ++i)
    {
        features.clear();
        for (int j = 0; j < imageExtractDatas_[i]->descriptors_.rows; ++j)
            features.push_back(imageExtractDatas_[i]->descriptors_.row(j));
        voc_->transform(features, imageExtractDatas_[i]->bowVec, imageExtractDatas_[i]->featureVec, 0);
    }
    vocDB_.reset(new DBoW3::Database(*voc_));
    for (int i = 0; i < imgSize; ++i)
        vocDB_->add(imageExtractDatas_[i]->bowVec);
    return true;
}
bool ProcessorVP::matchFeature()
{
    int imgSize = imageExtractDatas_.size();
    if (imgSize < 2)
        return false;
    auto funcMatch = [&](std::shared_ptr<ImageProcess_LYJ::ImageMatchData> _imageMatchData) {
        const uint32_t& id1 = _imageMatchData->id1;
        const uint32_t& id2 = _imageMatchData->id2;
        _imageMatchData->usePointMatch = opt_.imageMatchOpt.usePointMatch;
        _imageMatchData->useLineMatch = opt_.imageMatchOpt.useLineMatch;
        _imageMatchData->useEdgeMatch = opt_.imageMatchOpt.useEdgeMatch;
        _imageMatchData->usePatchMatch = opt_.imageMatchOpt.usePatchMatch;
        _imageMatchData->debugPath = "D:/tmp/imageProcess/match/";
        ImageProcess_LYJ::matchFeature(imageExtractDatas_[id1].get(), imageExtractDatas_[id2].get(), _imageMatchData.get(), opt_.imageMatchOpt);

        std::cout << "match image " << id1 << " and image " << id2 << " cnt " << _imageMatchData->pointMatchSize \
            << " triangle " << _imageMatchData->triDatas.triSuccess << " " << _imageMatchData->triDatas.triSize << std::endl;
        if (_imageMatchData->triDatas.triSuccess)
        {
            std::vector<Eigen::Vector3f> Ps;
            const auto& triData = _imageMatchData->triDatas;
            for (int i = 0; i < triData.bTris.size(); ++i)
            {
                if (triData.bTris[i])
                    Ps.push_back(triData.Ps[i].cast<float>());
            }
            SLAM_LYJ::BaseTriMesh btm;
            btm.setVertexs(Ps);
            SLAM_LYJ::writePLYMesh("D:/tmp/tri.ply", btm);
        }
        cv::Mat imgMatch;
        SLAM_LYJ::SLAM_LYJ_DEBUGGER::drawPointMatches(imageExtractDatas_[id1]->img, imageExtractDatas_[id1]->kps_, imageExtractDatas_[id2]->img, imageExtractDatas_[id2]->kps_, _imageMatchData->match2to1P, imgMatch, cv::Scalar(255,255,255), 3);
        cv::pyrDown(imgMatch, imgMatch);
        cv::pyrDown(imgMatch, imgMatch);
        cv::imshow("match", imgMatch);
        cv::waitKey();
    };
    std::vector<SLAM_LYJ::BitFlagVec> bfvs(imgSize);
    for (int i = 0; i < imgSize; ++i)
        bfvs[i].assign(imgSize, false);
    for (int i = 0; i < imgSize - 1; ++i)
        for (int j = i + 1; j < i+3; ++j)
            if (j < imgSize)
                bfvs[i].setFlag(j, true);
    DBoW3::QueryResults qRets;
    for (int i = 0; i < imgSize; ++i)
    {
        vocDB_->query(imageExtractDatas_[i]->bowVec, qRets, 3);
        for (int j = 0; j < qRets.size(); ++j)
        {
            if (qRets[j].Score > 0.5 && qRets[j].Id != i)
            {
                bfvs[i].setFlag(qRets[j].Id, true);
                bfvs[qRets[j].Id].setFlag(i, true);
            }
        }
    }
    std::vector<std::shared_ptr<ImageProcess_LYJ::ImageMatchData>> matchDatasPtr;
    matchDatasPtr.reserve(10 * imgSize);
    uint64_t key;
    for (int i = 0; i < imgSize - 1; ++i)
    {
        for (int j = i + 1; j < imgSize; ++j)
        {
            if (!bfvs[i][j])
                continue;
            matchDatasPtr.push_back(std::shared_ptr<ImageProcess_LYJ::ImageMatchData>(new ImageProcess_LYJ::ImageMatchData()));
            matchDatasPtr.back()->id1 = i;
            matchDatasPtr.back()->id2 = j;
            ImageProcess_LYJ::site2Key(i, j, key);
            imageMatchDatas_[key] = matchDatasPtr.back();
        }
    }
    for (int i = 0; i < matchDatasPtr.size(); ++i)
    {
        funcMatch(matchDatasPtr[i]);
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


namespace L3DPP
{
    // element of cluster universe
    typedef struct {
        int rank_;
        int clusterID_;
        int size_;
    } CLUnivElement;
    // class that holds a clustering result
    class CLUniverse
    {
    public:
        CLUniverse(int numNodes) : num_(numNodes)
        {
            // init
            elements_ = new CLUnivElement[num_];
            for (int i = 0; i < num_; ++i)
            {
                elements_[i].rank_ = 0;
                elements_[i].size_ = 1;
                elements_[i].clusterID_ = i;
            }
        }

        ~CLUniverse()
        {
            delete[] elements_;
        }

        // find clusterID for given node
        int find(int nodeID)
        {
            int y = nodeID;
            while (y != elements_[y].clusterID_)
                y = elements_[y].clusterID_;

            elements_[nodeID].clusterID_ = y;
            return y;
        }

        // joins two nodes into one class/segment
        void join(int x, int y)
        {
            if (elements_[x].rank_ > elements_[y].rank_)
            {
                elements_[y].clusterID_ = x;
                elements_[x].size_ += elements_[y].size_;
            }
            else
            {
                elements_[x].clusterID_ = y;
                elements_[y].size_ += elements_[x].size_;
                if (elements_[x].rank_ == elements_[y].rank_)
                    elements_[y].rank_++;
            }
            --num_;
        }

        float size(int x) const { return elements_[x].size_; }
        int numSets() const { return num_; }

    private:
        CLUnivElement* elements_;
        int num_;
    };

    // edge in affinity matrix
    typedef struct {
        int i_;
        int j_;
        float w_;
    } CLEdge;

    // sorting function for edges
    static bool sortCLEdgesByWeight(const CLEdge& a, const CLEdge& b)
    {
        return a.w_ < b.w_;
    }
    // sort entries for sparse affinity matrix (CLEdges)
    static bool sortCLEdgesByCol(const CLEdge& a1, const CLEdge& a2)
    {
        return ((a1.j_ < a2.j_) || (a1.j_ == a2.j_ && a1.i_ < a2.i_));
    }
    static bool sortCLEdgesByRow(const CLEdge& a1, const CLEdge& a2)
    {
        return ((a1.i_ < a2.i_) || (a1.i_ == a2.i_ && a1.j_ < a2.j_));
    }

    // perform graph clustering
    // NOTE: edges are sorted during the process!
    CLUniverse* performClustering(std::list<CLEdge>& edges, int numNodes,
        float c)
    {
        if (edges.size() == 0)
            return NULL;

        // sort edges by weight (increasing)
        edges.sort(L3DPP::sortCLEdgesByWeight);

        // init universe
        CLUniverse* u = new CLUniverse(numNodes);

        // init thresholds
        float* threshold = new float[numNodes];
        for (int i = 0; i < numNodes; ++i)
            threshold[i] = c;

        // perform clustering
        std::list<CLEdge>::const_iterator it = edges.begin();
        for (; it != edges.end(); ++it)
        {
            CLEdge e = *it;

            // components conected by this edge
            int a = u->find(e.i_);
            int b = u->find(e.j_);
            if (a != b)
            {
                // not in the same segment yet
                if ((e.w_ <= threshold[a]) && (e.w_ <= threshold[b]))
                {
                    // join nodes
                    u->join(a, b);
                    a = u->find(a);
                    threshold[a] = e.w_ + c / u->size(a);
                }
            }
        }

        // cleanup
        delete[] threshold;
        return u;
    }
}
bool ProcessorVP::generateMap()
{
    map_.reset(new Map(0));
    int frameSize = imageExtractDatas_.size();
    std::vector<std::shared_ptr<MapFrame>> mapFrames(frameSize, nullptr);
    for (int i = 0; i < frameSize; ++i)
        mapFrames[i].reset(new MapFrame(i, imageExtractDatas_[i]));
    map_->setMapFrames(mapFrames);
    std::vector<std::shared_ptr<MapPoint>> mapPoints;
    std::vector<ImageProcess_LYJ::CorrespondPoint> corrPoints;
    int pId = 0;
    for (int i = 0; i < frameSize; ++i)
    {
        auto imgExData = mapFrames[i]->getImageExtractData();
        int kpSize = imgExData->kps_.size();
        uint32_t imgId = mapFrames[i]->getId();
        for (int j = 0; j < kpSize; ++j)
        {
            if (correspondGraph_->isCorrPoint2MapPoint(imgId, j))
                continue;
            correspondGraph_->getCorrespondPoint(imgId, j, corrPoints, 3);
            if (corrPoints.empty())
                continue;
            std::shared_ptr<MapPoint> mapPoint = std::make_shared<MapPoint>(pId);
            mapPoints.push_back(mapPoint);
            ++pId;
            correspondGraph_->setCorrPoint2MapPoint(imgId, j);
            mapPoint->addOb(imgId, j);
            for (int k = 0; k < corrPoints.size(); ++k)
            {
                if (correspondGraph_->isCorrPoint2MapPoint(corrPoints[k].imageId_, corrPoints[k].pointId_))
                    continue;
                correspondGraph_->setCorrPoint2MapPoint(corrPoints[k].imageId_, corrPoints[k].pointId_);
                mapPoint->addOb(corrPoints[k].imageId_, corrPoints[k].pointId_);//这里需要保证imageExtractDatas_和mapFrames的id同步并且从0开始连续
            }
        }
    }
    return true;
}

bool ProcessorVP::optimize()
{
    return false;
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

bool ProcessorVP::trianleMapPoint()
{
    return false;
}



NSP_SLAM_LYJ_SRC_END