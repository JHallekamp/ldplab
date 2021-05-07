#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>

#include "../Data.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        __global__ void executeRayBoundingSphereIntersectionTestBruteForce(
            RayBuffer& ray_buffer,
            BoundingVolumeSphere* spheres,
            const size_t num_spheres)
        {
            int thread_index = blockIdx.x * blockDim.x + threadIdx.x;
            if (ray_buffer.index_data[thread_index] < num_spheres)
                return;

            double min_dist = -1.0;
            int 
        }
    }
}

#endif