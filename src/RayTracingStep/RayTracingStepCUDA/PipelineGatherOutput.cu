#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineGatherOutput.hpp"

__global__ void ldplab::rtscuda::PipelineGatherOutput::gatherOutputKernel(
    int32_t* ray_index_buffer, 
    Vec3* force_per_ray, 
    Vec3* torque_per_ray, 
    size_t num_rays_per_batch, 
    Vec3* force_per_particle, 
    Vec3* torque_per_particle, 
    Mat3* p2w_transformations, 
    Vec3* p2w_translations, 
    size_t num_particles, 
    bool particle_space_output)
{
    return __global__ void();
}

#endif