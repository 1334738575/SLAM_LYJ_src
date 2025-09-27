#include "mapPoint.h"



namespace SLAM_LYJ_src
{
    static int DescriptorDistance(const cv::Mat& a, const cv::Mat& b)
    {
        const int* pa = a.ptr<int32_t>();
        const int* pb = b.ptr<int32_t>();

        int dist = 0;

        for (int i = 0; i < 8; i++, pa++, pb++)
        {
            unsigned int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

    void MapPoint::updateDescriptor(const std::vector<cv::Mat>& _descs)
    {
        if (_descs.empty())
            return;
        // Compute distances between them
        const size_t N = _descs.size();
        std::vector<std::vector<int>> Distances(N, std::vector<int>(N, 0));
        for (size_t i = 0; i < N; i++)
        {
            Distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++)
            {
                int distij = DescriptorDistance(_descs[i], _descs[j]);
                Distances[i][j] = distij;
                Distances[j][i] = distij;
            }
        }
        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for (size_t i = 0; i < N; i++)
        {
            std::vector<int>& vDists = Distances[i];
            sort(vDists.begin(), vDists.end());
            int median = vDists[0.5 * (N - 1)];
            if (median < BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }

        descriptor_ = _descs[BestIdx].clone();
    }
}