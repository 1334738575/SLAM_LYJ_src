#include "SLAM_LYJ.h"
#include "base/PreDefine.h"
#include "common/ThreadPool.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include "base/Triangler.h"
#include "extractor/ORBextractor.h"
#include "extractor/LSDextractor.h"
#include "extractor/Cannyextractor.h"
#include "extractor/SIFTextractor.h"
#include "thirdParty/STLPlus/include/file_system.h"
//#include <flann/flann.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <cstdlib> // for rand()
#include <ctime>   // for time()
#include "matcher/PatchMatcher.h"
#include <common/Diffuser.h>
#include <common/PolarGrid.h>
#include <common/OcTree.h>
#include <common/QuadTree.h>
//#include <cudaImp/CUDAProjector.h>

#include "debugger/debugger.h"

NSP_SLAM_LYJ_BEGIN

SLAM_LYJ_API void testOpenCV() {
    cv::Mat m = cv::imread(LYJOPT->sysHomePath + "other/down.png");
    cv::pyrDown(m, m);
    cv::pyrDown(m, m);
    cv::imshow("test", m);
    cv::waitKey();
}
SLAM_LYJ_API void testEigen() {
    Eigen::Vector3d x;
    x.setZero();
    std::cout << x << std::endl;
}
SLAM_LYJ_API void testKdTree() {
    using TYPE = Eigen::Vector3d;
    std::vector<TYPE> points;
    points.push_back(TYPE(0, 0, 0));
    points.push_back(TYPE(2, 2, 2));
    points.push_back(TYPE(9, 9, 9));
    points.push_back(TYPE(4, 4, 4));
    points.push_back(TYPE(6, 6, 6));
    //SLAM_LYJ_MATH::KdTree2d kdtree;
    SLAM_LYJ_MATH::KdTree<double, 3> kdtree;
    auto funcDist = [](const SLAM_LYJ_MATH::KdTree<double, 3>::Node* _node, const Eigen::Vector3d& _p)->double {
        if (_node->data.first.norm() < 0.1)
            return DBL_MAX;
        return (_p - _node->data.first).norm();
    };
    kdtree.build(points, funcDist);
    std::vector<TYPE> rets;
    std::vector<double> dists;
    TYPE p(-3, -3, -3);
    kdtree.search(p, 2, 1000, rets, dists);
    for (size_t i = 0; i < rets.size(); i++)
    {
        std::cout << i << ": \n" << rets[i] << std::endl;
    }
    return;
}
SLAM_LYJ_API void testPoint() {
    SLAM_LYJ_MATH::Point2d p2d(1, 3);
    std::cout << p2d[0] << std::endl;
    std::cout << p2d << std::endl;
    SLAM_LYJ_MATH::Point2d p2d2(3, 3);
    std::cout << p2d + p2d2 << std::endl;
    p2d += p2d2;
    std::cout << p2d << std::endl;
    std::cout << p2d.x() << std::endl;
    std::cout << p2d.y() << std::endl;
    p2d[0] = 3;
    std::cout << p2d.x() << std::endl;
    SLAM_LYJ_MATH::Point2d p2d3 = p2d + p2d2;
    SLAM_LYJ_MATH::Point2d xx = 2 * p2d * 2;
    std::cout << "xx: " << xx << std::endl;
    //SLAM_LYJ_MATH::Point2i p2i(1, 1);
    //std::cout << (SLAM_LYJ_MATH::Point2i)(++p2i) << std::endl;
}
SLAM_LYJ_API void testArchive() {
    SLAM_LYJ_MATH::Point2d p2d(3, 3);
    SLAM_LYJ_MATH::Point2d p2d2;
    //std::stringstream ss;
    //boost::archive::text_oarchive oar{ ss };
    //oar << p2d;
    //boost::archive::text_iarchive iar{ ss };
    //iar >> p2d2;
    std::string binFile = "test.dat";
    std::ofstream ofs(binFile, std::ios::binary);
    if (!ofs)
        return;
    p2d.write_binary(ofs);
    ofs.close();
    std::ifstream ifs(binFile, std::ios::binary);
    if (!ifs)
        return;
    p2d2.read_binary(ifs);
    ifs.close();
    std::cout << p2d2 << std::endl;
}
SLAM_LYJ_API void testTensor() {
    //Eigen::Matrix<double, 3, 4> mat;
    //mat << 1.0, 2.0, 3.0, 6.0,
    //    4.0, 2.0, 6.0, 12.0,
    //    7.0, 3.0, 9.0, 18.0;
    //Eigen::Matrix3d Q;
    //Eigen::Matrix<double, 3, 4> R;
    //SLAM_LYJ_MATH::QRLYJ<3, 4> qr;
    //qr.QR(mat.data(), Q.data(), R.data());

    //Eigen::Matrix<double, 3, 3> mat;
    //mat << 1.0, 2.0, 3.0,
    //    4.0, 2.0, 6.0,
    //    7.0, 3.0, 9.0;
    //Eigen::Matrix3d Q;
    //Eigen::Matrix<double, 3, 3> R;
    //SLAM_LYJ_MATH::QRLYJ<3, 3> qr;
    //qr.QR(mat.data(), Q.data(), R.data());

    Eigen::Matrix<double, 4, 3> mat;
    mat << 1.0, 2.0, 3.0,
        4.0, 2.0, 6.0,
        7.0, 3.0, 9.0,
        7.0, 3.0, 9.0;
    Eigen::Matrix<double, 4, 4> Q;
    Eigen::Matrix<double, 4, 3> R;
    SLAM_LYJ_MATH::QRLYJ<4, 3> qr;
    qr.QR(mat.data(), Q.data(), R.data());
}
SLAM_LYJ_API void testCommonAlgorithm() {
	//Rodrigues
	auto R = SLAM_LYJ_MATH::Rodrigues2RotMatrix<double>(Eigen::Vector3d(1, 0, 0), 0.523);
	std::cout << R << std::endl;
    //�׳�
    uint64_t res1 = SLAM_LYJ_MATH::factorial(10, 9);
    std::cout << "�׳ˣ�"<<res1 << std::endl;
    //����
    uint64_t res2 = SLAM_LYJ_MATH::pernutateNum(10, 2);
    std::cout << "����:"<<res2 << std::endl;
    //���
    uint64_t res3 = SLAM_LYJ_MATH::selectNum(2, 1);
    std::cout << "���:"<<res3 << std::endl;
    std::vector<std::vector<int>> results;
    SLAM_LYJ_MATH::fullSelection(10, 10, results);
    for (int i = 0; i < (int)results.size(); ++i) {
        std::cout << i << ": ";
        for (int j = 0; j < (int)results[i].size(); ++j) {
            std::cout << results[i][j] << " ";
        }
        std::cout << std::endl;
    }
}
SLAM_LYJ_API void testRANSAC() {
    //ax+by+c=0
    std::vector<SLAM_LYJ_MATH::Point2d> points;
    int pSize = 10;
    for (int i = 0; i < pSize; ++i) {
        points.emplace_back(i, 2 * i + 1); //2x-y+1=0
    }
    points[0].x() += 0.5;
    points[2].y() += 1;
    Eigen::Vector3d params; //a b c
    int eveSize = 2;
    double errTh = 0.01;
    auto funcCalError = [&](const Eigen::Vector3d& _params, const SLAM_LYJ_MATH::Point2d& _p, double& _err)->bool {
        _err = _params[0] * _p[0] + _params[1] * _p[1] + _params[2];
        _err /= (_params.block(0, 0, 2, 1).norm());
        if (std::abs(_err) > errTh) {
            return false;
        }
        return true;
    };
    auto funcCalModule = [](const std::vector<const SLAM_LYJ_MATH::Point2d*>& _samples, Eigen::Vector3d& _params)->bool {
        _params[0] = _samples.at(0)->y() - _samples.at(1)->y();
        _params[1] = -1 * (_samples.at(0)->x() - _samples.at(1)->x());
        _params[2] = -1 * _params[0] * _samples.at(0)->x() - _params[1] * _samples.at(0)->y();
        return true;
    };
    double inlineRatioTh = 0.8;
    SLAM_LYJ_MATH::RANSAC<SLAM_LYJ_MATH::Point2d, double, Eigen::Vector3d> ransac(inlineRatioTh, eveSize);
    ransac.setCallBack(funcCalError, funcCalModule);
    std::vector<double> errs;
    std::vector<bool> inlines;
    double inlineRatio = ransac.run(points, errs, inlines, params);
    std::cout << "line2d: \n" << params << std::endl;
    if (errs.empty()) {
        std::cout << "fail" << std::endl;
        return;
    }
    for (int i = 0; i < pSize; ++i) {
        std::cout << "err: " << errs[i] << ", inline: " << (int)inlines[i] << std::endl;
    }
}
SLAM_LYJ_API void testLine() {
    SLAM_LYJ_MATH::Line3d line3d(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, -1));
    SLAM_LYJ_MATH::Line3d line3d2(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
    std::cout << SLAM_LYJ_MATH::Line3d::angleL2L(line3d, line3d2) << std::endl;
    std::cout << line3d.distP2L(Eigen::Vector3d(1,0,0)) << std::endl;
    return;
    SLAM_LYJ_MATH::Line2d line2d(Eigen::Vector2d(1, 0), Eigen::Vector2d(1, -1.71));
    std::cout << line2d << std::endl;
    std::cout << line2d.angle() << std::endl;
    std::cout << line2d.dist() << std::endl;
    SLAM_LYJ_MATH::Line2d line2d2(Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1));
    std::cout << line2d2 << std::endl;
    Eigen::Vector2d interP;
    if(SLAM_LYJ_MATH::Line2d::interestL2L(line2d, line2d2, interP))
        std::cout << interP << std::endl;
    Eigen::Vector2d p(1, 0);
    std::cout << line2d2.distP2L(p) << std::endl;
    std::cout << line2d2.dir() << std::endl;
    std::cout << line2d2.pointInL(p) << std::endl;
    std::cout << SLAM_LYJ_MATH::Line2d::angleL2L(line2d, line2d2) << std::endl;
    cv::Mat m(100, 100, CV_8UC1, cv::Scalar(0));
    std::vector<Eigen::Vector2i> ps;
    SLAM_LYJ_MATH::bresenhamLine(0, 0, 90, 50, ps);
    for (const auto& point : ps) {
        m.at<uchar>(cv::Point(point.x(), point.y())) = 255;
    }
    cv::imshow("bresenham", m);
    cv::waitKey();
}
SLAM_LYJ_API void testPlane() {
    SLAM_LYJ_MATH::Plane3d plane3d(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1));
    std::cout << plane3d << std::endl;
}
SLAM_LYJ_API void testPose() {
    Pose2D pose2D(1.57, Eigen::Vector2d(0,0));
    std::cout << pose2D.getR() << std::endl;
}
SLAM_LYJ_API void testTriangler() {
    TriangleOption option;
    Trianglerd triangler(option);
}
SLAM_LYJ_API void testBitFlagVec() {
    SLAM_LYJ_MATH::BitFlagVec bfv(10);
    bfv.setFlag(5, true);
    std::cout << bfv[4] << std::endl;
}
SLAM_LYJ_API void testIncrementalAgvAndVar() {
    std::vector<double> v{ 0,1,2,3,4,5,6 };
    double agv = 0;
    double var = 0;
    for (int i = 0; i < (int)v.size(); ++i) {
        SLAM_LYJ_MATH::calculateAgvAndVar<double, 1>(i, &v[i], &agv, &var);
        std::cout << "agv: " << agv << " var: " << var << std::endl;
    }
}
SLAM_LYJ_API void testGrid() {
    std::vector<Eigen::Vector2d> points;
    for (int i = 0; i < 100; ++i) {
        points.push_back(Eigen::Vector2d(i, i));
    }
    //SLAM_LYJ_MATH::Grid<double, 2> grid2Dd(10, points);
    SLAM_LYJ_MATH::Grid<double, 2> grid2Dd(10, std::vector<int>{100, 100}, Eigen::Vector2d(50,50), points);
    const auto& inds = grid2Dd.getIndsNear(Eigen::Vector2d(20, 20));
    for (const auto& ind : inds) {
        std::cout << points[ind] << std::endl << std::endl;
    }
}
SLAM_LYJ_API void testSTLPlus() {
    std::string file1 = "D:/tmp/stlplus.png";
    std::string file2 = "D:";
    std::string path1 = stlplus::create_filespec(file2, "tmp");
    std::string path2 = stlplus::create_filespec(path1, "testImages");
    std::string path3 = stlplus::create_filespec(path2, "11.png");
    stlplus::file_copy(path3, file1);
    return;
    std::string folderPath = "E:/SLAM_LYJ/build/example/Release/testSTLPlus";
    //boost::filesystem::create_directories(folderPath);
    //boost::filesystem::remove(folderPath);
    //boost::filesystem::exi
    if (stlplus::folder_exists(folderPath))
        stlplus::folder_delete(folderPath, true);
    stlplus::folder_create(folderPath);
    for (int i = 0; i < 10; ++i) {
        std::ofstream ifs(folderPath + "/" + std::to_string(i) + ".txt");
        ifs.close();
    }
    std::vector<std::string> files = stlplus::folder_files(folderPath);
    int fSize = (int)files.size();
    std::cout << "Number of files in " << folderPath << " is: " << fSize << std::endl;
    for (int i = 0; i < fSize; ++i) {
        std::cout << files[i] << std::endl;
    }
}
SLAM_LYJ_API void testBuffer(){
    {
        LYJBuffer buffer;
        Pose3D* poseCache = buffer.alloc<Pose3D>("pose");
        if(poseCache)
            std::cout << *poseCache << std::endl;
        Pose2D* poseCache2 = buffer.get<Pose2D>("pose2");
        if (!poseCache2)
            std::cout << "no such cache in buffer." << std::endl;
        Pose2D* poseCache3 = buffer.alloc<Pose2D>("pose");
        if (!poseCache3)
            std::cout << "alloc cache in buffer failed." << std::endl;
    }
}
SLAM_LYJ_API void testGlobalOption() {
    //GlobalInnerOption* gOpt = GlobalInnerOption::get();
    LYJOPT->sysName = "LYJ_SLAM";
    std::cout << LYJOPT->sysName << std::endl;
}
//SLAM_LYJ_API void testFlann()
//{
//    // 创建一些随机数据
//    const int num_points = 100;
//    const int dim = 2;
//    std::vector<std::vector<float>> dataset(num_points, std::vector<float>(dim));
//    for (int i = 0; i < num_points; ++i)
//    {
//        dataset[i][0] = static_cast<float>(rand()) / RAND_MAX; // x
//        dataset[i][1] = static_cast<float>(rand()) / RAND_MAX; // y
//    }
//
//    // 将数据转换为 FLANN 格式
//    flann::Matrix<float> dataset_matrix(&dataset[0][0], num_points, dim);
//
//    // 创建一个 FLANN 索引
//    flann::Index<flann::L2<float>> index(dataset_matrix, flann::KDTreeIndexParams(4));
//    index.buildIndex();
//
//    // 查询点
//    std::vector<float> query_point = { 0.5f, 0.5f };
//    flann::Matrix<float> query_matrix(&query_point[0], 1, dim);
//
//    // 存储最近邻的索引和距离
//    std::vector<std::vector<int>> indices(1);
//    std::vector<std::vector<float>> dists(1);
//
//    // 进行最近邻搜索
//    index.knnSearch(query_matrix, indices, dists, 1, flann::SearchParams(32));
//
//    // 打印结果
//    std::cout << "Nearest neighbor index: " << indices[0][0] << ", Distance: " << dists[0][0] << std::endl;
//
//    return;
//}
//SLAM_LYJ_API void testPCL()
//{
//    // 初始化随机数生成器
//    std::srand(static_cast<unsigned int>(std::time(0)));
//
//    // 创建一个点云对象
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//
//    // 设置点云的宽度和高度
//    cloud->width = 1000;    // 点的数量
//    cloud->height = 1;      // 单行点云
//    cloud->is_dense = true; // 点云是否是稠密的
//
//    // 调整点云大小
//    cloud->points.resize(cloud->width * cloud->height);
//
//    // 生成随机点
//    for (size_t i = 0; i < cloud->points.size(); ++i)
//    {
//        cloud->points[i].x = static_cast<float>(std::rand()) / RAND_MAX * 10.0f; // x 范围 [0, 10]
//        cloud->points[i].y = static_cast<float>(std::rand()) / RAND_MAX * 10.0f; // y 范围 [0, 10]
//        cloud->points[i].z = static_cast<float>(std::rand()) / RAND_MAX * 10.0f; // z 范围 [0, 10]
//    }
//
//    std::cout << "Generated " << cloud->width * cloud->height << " random points." << std::endl;
//
//    // 创建一个可视化对象
//    pcl::visualization::CloudViewer viewer("Random Point Cloud Viewer");
//
//    // 设置点云数据
//    viewer.showCloud(cloud);
//
//    // 等待直到用户关闭窗口
//    while (!viewer.wasStopped())
//    {
//    }
//    return;
//}
SLAM_LYJ_API void testPatchMatch()
{
    int w = 800;
    int h = 600;
    cv::Mat m(h, w, CV_8UC1);
    int pSize = 1000;
    std::vector<cv::KeyPoint> kps(1000);
    for (int i=0;i<pSize;++i) 
    {
        int a = rand() % w;
        int b = rand() % h;
        kps[i].pt.x = a;
        kps[i].pt.y = b;
    }
    SLAM_LYJ_MATH::Grid2Df grid(100, h, w, kps);
    int gw = grid.getGridSize(0);
    int gh = grid.getGridSize(1);
    std::vector<cv::KeyPoint> kps2;
    kps2.reserve(gw * gh);
    for (int i = 0; i < gh; ++i) {
        for (int j = 0; j < gw; ++j) {
            std::list<int> inds = grid.getInds(i, j);
            if (inds.empty())
                continue;
            kps2.push_back(kps[inds.front()]);
            //std::vector<cv::KeyPoint> kps3;
            //for (const auto& ind : inds) {
            //    kps3.push_back(kps[ind]);
            //}
            //cv::Mat m42(h, w, CV_8UC1);
            //SLAM_LYJ_DEBUGGER::drawGrid(m, grid, m42, cv::Scalar(255), 2);
            //cv::Mat m52(h, w, CV_8UC1);
            //SLAM_LYJ_DEBUGGER::drawFeatures(m42, kps3, m52, cv::Scalar(255), 5, 2, false);
            //cv::imshow("every block", m52);
            //cv::waitKey();
        }
    }
    //cv::Mat m2(h, w, CV_8UC1);
    //SLAM_LYJ_DEBUGGER::drawFeatures(m, kps, m2, cv::Scalar(255), 5, 2);
    //cv::imshow("o", m2);
    //cv::Mat m3(h, w, CV_8UC1);
    //SLAM_LYJ_DEBUGGER::drawFeatures(m, kps2, m3, cv::Scalar(255), 5, 1);
    //cv::imshow("d", m3);
    //cv::Mat m4(h, w, CV_8UC1);
    //SLAM_LYJ_DEBUGGER::drawGrid(m, grid, m4, cv::Scalar(255), 2);
    //cv::Mat m5(h, w, CV_8UC1);
    //SLAM_LYJ_DEBUGGER::drawFeatures(m4, kps2, m5, cv::Scalar(255), 5, 2, false);
    //cv::imshow("g", m5);
    //cv::waitKey();

    cv::Mat img1 = cv::Mat::zeros(h, w, CV_8UC1);
    cv::Mat img2 = cv::Mat::zeros(h, w, CV_8UC1);
    int v = 255;
    for (const auto& kp : kps2) {
        img1.at<uchar>(cv::Point(kp.pt.x, kp.pt.y)) = uchar(v);
        if((kp.pt.x + 10) < 0 || (kp.pt.x + 10) > w - 1
            || (kp.pt.y + 10) < 0 || (kp.pt.y + 10) > h - 1)
			continue;
        img2.at<uchar>(cv::Point(kp.pt.x + 10, kp.pt.y + 10)) = uchar(v);
    }
    //cv::imshow("img1", img1);
    //cv::imshow("img2", img2);
    //cv::waitKey();
    //PatchMatcher::Option matchOpt;
    //matchOpt.maxIterNum = 4;
    //PatchMatcher matcher(matchOpt);
    //std::vector<PatchMatchResult> matches;
    //matcher.matchPatch(img1, kps2, img2, false, matches);
    return;
}
SLAM_LYJ_API void testDiffuser()
{
    SLAM_LYJ_MATH::Diffuser2D diffuser(6, 8, std::vector<Eigen::Vector2i>(), Eigen::Vector2i(3, 3));
    std::vector<Eigen::Vector2i> locs;
    while(diffuser.next(locs)){}
    return;
}
SLAM_LYJ_API void testPolarGrid()
{
    std::vector<float> rs(10, 0.1);
    SLAM_LYJ_MATH::MultiPolarGrid<int> multiPolarGrid(rs, 0.1f, 0, PI / 2, PI, 1);
    {
        uint32_t sss = multiPolarGrid.getTotalSize();
        std::ofstream f("multiPolarGrid.txt");
        Eigen::Vector3f loc;
        bool sign = true;
        for (uint32_t i = 0; i < sss; ++i) {
            if (multiPolarGrid.getCoordInGridById(i, loc)) {
                f << loc(0) << "," << loc(1) << "," << loc(2) << std::endl;
                if (i == 1000 && false) {
                    Eigen::Matrix3f rot;
                    Eigen::Vector3f trans;
                    multiPolarGrid.getPoseInGridById(i, rot, trans);
                    Pose3D pose(rot.cast<double>(), trans.cast<double>());
                    //Pose3D invPose = pose.inversed();
                    Pose3D invPose = pose;
                    Eigen::Vector3d ax(0.5, 0, 0);
                    Eigen::Vector3d ay(0, 0.5, 0);
                    Eigen::Vector3d az(0, 0, 0.5);
                    auto ax2 = invPose * ax;
                    auto ay2 = invPose * ay;
                    auto az2 = invPose * az;
                    std::ofstream f2("transform.txt");
                    f2 << 0 << "," << 0 << "," << 0 << std::endl;
                    f2 << ax(0) << "," << ax(1) << "," << ax(2) << std::endl;
                    f2 << ay(0) << "," << ay(1) << "," << ay(2) << std::endl;
                    f2 << az(0) << "," << az(1) << "," << az(2) << std::endl;
                    f2 << loc(0) << "," << loc(1) << "," << loc(2) << std::endl;
                    f2 << ax2(0) << "," << ax2(1) << "," << ax2(2) << std::endl;
                    f2 << ay2(0) << "," << ay2(1) << "," << ay2(2) << std::endl;
                    f2 << az2(0) << "," << az2(1) << "," << az2(2) << std::endl;
                    f2.close();
                }
                if (sign) {
                    std::vector<uint32_t> ids;
                    multiPolarGrid.getAroundIdByCoord(loc, 2, ids);
                    std::ofstream f2("nearPolarGrid2.txt");
                    for (const auto& id : ids) {
                        multiPolarGrid.getCoordInGridById(id, loc);
                        f2 << loc(0) << "," << loc(1) << "," << loc(2) << std::endl;
                    }
                    f2.close();
                    sign = false;
                }
            }
        }
        f.close();
    }
    SLAM_LYJ_MATH::SoloPolarGrid<int> soloPolarGrid(2.0f, 0.1f, 0.1f, 0.1f, 0, PI/2, PI);
    {
        uint32_t sss = soloPolarGrid.getTotalSize();
        std::ofstream f("soloPolarGrid.txt");
        Eigen::Vector3f loc;
        bool sign = true;
        for (uint32_t i = 0; i < sss; ++i) {
            if (soloPolarGrid.getCoordInGridById(i, loc)) {
                f << loc(0) << "," << loc(1) << "," << loc(2) << std::endl;
                if (sign) {
                    std::vector<uint32_t> ids;
                    soloPolarGrid.getAroundIdByCoord(loc, 2, ids);
                    std::ofstream f2("nearPolarGrid.txt");
                    for (const auto& id : ids) {
                        soloPolarGrid.getCoordInGridById(id, loc);
                        f2 << loc(0) << "," << loc(1) << "," << loc(2) << std::endl;
                    }
                    f2.close();
                    sign = false;
                }
            }
        }
        f.close();
    }
    return;
}
SLAM_LYJ_API void testOcTreeAndQuadTree()
{
    SLAM_LYJ_MATH::OcTreed tree1;
    SLAM_LYJ_MATH::OcTreef tree2;
    SLAM_LYJ_MATH::QuadTreed tree3;
    SLAM_LYJ_MATH::QuadTreef tree4;
    return;
}
SLAM_LYJ_API void testCUDA()
{
    //move to thirdparty
    //int sz = 100;
    //int szBit = sz * sizeof(int);
    //std::vector<int> as(100, 0);
    //std::vector<int> bs(100, 1);
    //std::vector<int> cs(100, 0);
    //int* asDev;
    //int* bsDev;
    //int* csDev;
    //cudaMalloc((void**)&asDev, szBit);
    //cudaMalloc((void**)&bsDev, szBit);
    //cudaMalloc((void**)&csDev, szBit);
    //cudaMemcpy(asDev, as.data(), szBit, cudaMemcpyHostToDevice);
    //cudaMemcpy(bsDev, bs.data(), szBit, cudaMemcpyHostToDevice);

    //SLAM_LYJ_CUDA::testCUDA(asDev, bsDev, csDev, sz);

    //cudaMemcpy(cs.data(), csDev, szBit, cudaMemcpyDeviceToHost);

    //for (int i = 0; i < sz; ++i)
    //{
    //    std::cout << cs[i] << std::endl;
    //}
    return;
}

SLAM_LYJ_API int getVersion(){
    return SYS_VERSION;
}

SLAM_LYJ_API void testThreadPool(){
    SLAM_LYJ_MATH::ThreadPool threadPool(5);
    for(int i=0;i<100;++i)
    {
        threadPool.addTask([i]{ std::cout<< "hello " << i << std::endl;});
    }
    threadPool.finish();
}





NSP_SLAM_LYJ_END