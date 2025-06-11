#ifndef SLAM_LYJ_SVDICP_H
#define SLAM_LYJ_SVDICP_H

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>

#include "common/CommonAlgorithm.h"

NSP_SLAM_LYJ_MATH_BEGIN

//using namespace Eigen::;

struct TransformResult {
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    bool is_degenerate; // 标记是否发生退化（共面情况）
};

TransformResult svd_icp(const std::vector<Eigen::Vector3f>& source,
    const std::vector<Eigen::Vector3f>& target) {
    // 1. 输入校验
    assert(source.size() == target.size());
    const size_t N = source.size();

    // 2. 计算质心
    Eigen::Vector3f centroid_p = Eigen::Vector3f::Zero();
    Eigen::Vector3f centroid_q = Eigen::Vector3f::Zero();
    for (const auto& p : source) centroid_p += p;
    for (const auto& q : target) centroid_q += q;
    centroid_p /= N;
    centroid_q /= N;

    // 3. 去中心化并构建H矩阵
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < N; ++i) {
        H += (source[i] - centroid_p) * (target[i] - centroid_q).transpose();
    }

    // 4. SVD分解（计算全U和V）
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    // 5. 处理退化情况（共面点云）
    TransformResult result;
    const float eps = 1e-6f;
    Eigen::Vector3f S = svd.singularValues();
    result.is_degenerate = (S(2) < eps * S(0)); // 第三奇异值接近0视为退化

    // 6. 计算旋转矩阵
    Eigen::Matrix3f R = V * U.transpose();
    if (R.determinant() < 0) {
        V.col(2) *= -1; // 处理反射
        R = V * U.transpose();
    }

    // 7. 若退化则限制为2D变换
    if (result.is_degenerate) {
        // 提取平面法向量（对应最小奇异值的右奇异向量）
        Eigen::Vector3f normal = V.col(2);

        // 构造绕法线旋转的2D变换
        Eigen::Matrix3f R_2d = Eigen::AngleAxisf(0, normal).toRotationMatrix(); // 实际中需优化角度
        R = R_2d * R; // 保持原变换的平面内旋转
    }

    // 8. 计算平移
    result.R = R;
    result.t = centroid_q - R * centroid_p;
    return result;
}

NSP_SLAM_LYJ_MATH_END

#endif