#ifndef SLAM_LYJ_KMEANS_H
#define SLAM_LYJ_KMEANS_H

#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <algorithm>

#include "common/CommonAlgorithm.h"

NSP_SLAM_LYJ_MATH_BEGIN

class KMeans
{
private:
    int k;                                      // 聚类数
    int max_iter;                               // 最大迭代次数
    int n_features;                             // 数据维度
    std::vector<std::vector<double>> centroids; // 聚类中心

    // 计算欧氏距离平方（避免开方提升性能）
    double distance(const std::vector<double> &a, const std::vector<double> &b)
    {
        double dist = 0.0;
        for (size_t i = 0; i < a.size(); ++i)
        {
            double diff = a[i] - b[i];
            dist += diff * diff;
        }
        return dist;
    }

public:
    KMeans(int k_, int max_iter_ = 300) : k(k_), max_iter(max_iter_) {}

    // 训练入口
    void fit(const std::vector<std::vector<double>> &data)
    {
        if (data.empty() || k <= 0)
            return;
        n_features = data[0].size();

        // 1. 初始化中心点 (随机选择数据点)
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(data.begin(), data.end(), gen);
        centroids.assign(data.begin(), data.begin() + k);

        // 2. 迭代优化
        for (int iter = 0; iter < max_iter; ++iter)
        {
            // 分配样本到最近中心
            std::vector<std::vector<std::vector<double>>> clusters(k);
            for (const auto &point : data)
            {
                double min_dist = std::numeric_limits<double>::max();
                int cluster_idx = 0;
                for (size_t i = 0; i < centroids.size(); ++i)
                {
                    double dist = distance(point, centroids[i]);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        cluster_idx = i;
                    }
                }
                clusters[cluster_idx].push_back(point);
            }

            // 3. 计算新中心点
            std::vector<std::vector<double>> new_centroids;
            for (const auto &cluster : clusters)
            {
                if (cluster.empty())
                { // 处理空簇
                    new_centroids.push_back(data[gen() % data.size()]);
                    continue;
                }
                std::vector<double> mean(n_features, 0.0);
                for (const auto &pt : cluster)
                {
                    for (int j = 0; j < n_features; ++j)
                    {
                        mean[j] += pt[j];
                    }
                }
                for (auto &val : mean)
                {
                    val /= cluster.size();
                }
                new_centroids.push_back(mean);
            }

            // 4. 收敛检测
            bool converged = true;
            for (size_t i = 0; i < centroids.size(); ++i)
            {
                if (distance(centroids[i], new_centroids[i]) > 1e-6)
                {
                    converged = false;
                    break;
                }
            }
            if (converged)
                break;
            centroids = new_centroids;
        }
    }

    // 预测样本所属聚类
    std::vector<int> predict(const std::vector<std::vector<double>> &data)
    {
        std::vector<int> labels;
        for (const auto &point : data)
        {
            double min_dist = std::numeric_limits<double>::max();
            int label = -1;
            for (size_t i = 0; i < centroids.size(); ++i)
            {
                double dist = distance(point, centroids[i]);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    label = i;
                }
            }
            labels.push_back(label);
        }
        return labels;
    }

    // 获取中心点
    std::vector<std::vector<double>> get_centroids() const
    {
        return centroids;
    }
};

// 示例数据生成器
std::vector<std::vector<double>> generate_sample_data(int samples, int clusters)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, 2.0);

    std::vector<std::vector<double>> data;
    for (int i = 0; i < clusters; ++i)
    {
        double center_x = 10.0 * i;
        for (int j = 0; j < samples / clusters; ++j)
        {
            data.push_back({center_x + dist(gen), dist(gen)});
        }
    }
    return data;
}

// // 替换随机初始化为概率选择
// centroids.push_back(data[gen() % data.size()]);
// for (int i = 1; i < k; ++i) {
//     std::vector<double> distances;
//     for (const auto& pt : data) {
//         double min_dist = std::numeric_limits<double>::max();
//         for (const auto& c : centroids) {
//             min_dist = std::min(min_dist, distance(pt, c));
//         }
//         distances.push_back(min_dist);
//     }
//     std::discrete_distribution<> d(distances.begin(), distances.end());
//     centroids.push_back(data[d(gen)]);
// }

// int main() {
//     // 生成测试数据（3个高斯分布簇）
//     auto data = generate_sample_data(300, 3);

//     // 训练模型
//     KMeans model(3);
//     model.fit(data);

//     // 输出结果
//     std::cout << "Cluster Centers:\n";
//     for (const auto& center : model.get_centroids()) {
//         printf("(%.2f, %.2f)\n", center[0], center[1]);
//     }

//     return 0;
// }


NSP_SLAM_LYJ_MATH_END

#endif