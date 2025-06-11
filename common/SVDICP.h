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
    bool is_degenerate; // ����Ƿ����˻������������
};

TransformResult svd_icp(const std::vector<Eigen::Vector3f>& source,
    const std::vector<Eigen::Vector3f>& target) {
    // 1. ����У��
    assert(source.size() == target.size());
    const size_t N = source.size();

    // 2. ��������
    Eigen::Vector3f centroid_p = Eigen::Vector3f::Zero();
    Eigen::Vector3f centroid_q = Eigen::Vector3f::Zero();
    for (const auto& p : source) centroid_p += p;
    for (const auto& q : target) centroid_q += q;
    centroid_p /= N;
    centroid_q /= N;

    // 3. ȥ���Ļ�������H����
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < N; ++i) {
        H += (source[i] - centroid_p) * (target[i] - centroid_q).transpose();
    }

    // 4. SVD�ֽ⣨����ȫU��V��
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    // 5. �����˻������������ƣ�
    TransformResult result;
    const float eps = 1e-6f;
    Eigen::Vector3f S = svd.singularValues();
    result.is_degenerate = (S(2) < eps * S(0)); // ��������ֵ�ӽ�0��Ϊ�˻�

    // 6. ������ת����
    Eigen::Matrix3f R = V * U.transpose();
    if (R.determinant() < 0) {
        V.col(2) *= -1; // ������
        R = V * U.transpose();
    }

    // 7. ���˻�������Ϊ2D�任
    if (result.is_degenerate) {
        // ��ȡƽ�淨��������Ӧ��С����ֵ��������������
        Eigen::Vector3f normal = V.col(2);

        // �����Ʒ�����ת��2D�任
        Eigen::Matrix3f R_2d = Eigen::AngleAxisf(0, normal).toRotationMatrix(); // ʵ�������Ż��Ƕ�
        R = R_2d * R; // ����ԭ�任��ƽ������ת
    }

    // 8. ����ƽ��
    result.R = R;
    result.t = centroid_q - R * centroid_p;
    return result;
}

NSP_SLAM_LYJ_MATH_END

#endif