#include "CUDAProjector.h"

namespace SLAM_LYJ_CUDA
{
	__global__ void testCU(int* _as, int* _bs, int* _cs, int _sz) {
		int idx = threadIdx.x + blockDim.x * blockIdx.x;
		//int idy = threadIdx.y + blockDim.y * blockIdx.y;
		int id = idx;
		if (id >= _sz)
			return;
		_cs[id] = _as[id] + _bs[id];
	}

	void testCUDA(int* _as, int* _bs, int* _cs, int _sz)
	{
		dim3 block(32, 1);
		dim3 grid(32, 1);
		testCU << <grid, block >> > (_as, _bs, _cs, _sz);
	}
}