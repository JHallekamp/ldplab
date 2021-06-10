#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineInitial.hpp"

#include "Context.hpp"
#include "GenericBoundingVolume.hpp"
#include "../../Utils/Log.hpp"

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>

std::shared_ptr<ldplab::rtscuda::IPipelineInitialStage> 
    ldplab::rtscuda::IPipelineInitialStage::createInstance(
        const ExperimentalSetup& setup, 
        const RayTracingStepCUDAInfo& info, 
        Context& context)
{
    // Currently just try to create the one implementation we have
    std::shared_ptr<ldplab::rtscuda::IPipelineInitialStage> initial_stage =
        std::make_shared<PipelineInitialHomogenousLightBoundingSpheres>(context);
    if (!initial_stage->allocate(setup, info))
        return nullptr;
    return initial_stage;
}

namespace homogenous_light_bounding_spheres_cuda
{
    __global__ void projectParticlesKernel(
        ldplab::rtscuda::GenericBoundingVolumeData* bounding_volumes,
        double light_source_resolution_per_world_unit);
    __global__ void countTotalRaysKernelFirst(
        size_t num_blocks);
    __global__ void countTotalRaysKernelSecond(
        size_t num_blocks);
    __global__ void createBatchKernel(
        int32_t* ray_index_buffer,
        ldplab::Vec3* ray_origin_buffer,
        ldplab::Vec3* ray_direction_buffer,
        double* ray_intensity_buffer,
        double* ray_min_bv_dist_buffer,
        size_t num_rays_per_batch,
        size_t batch_no);
    __device__ ldplab::rtscuda::pipelineInitialStageCreateBatchKernel_t
        create_batch_kernel_ptr = createBatchKernel;
    __device__ ldplab::rtscuda::PipelineInitialHomogenousLightBoundingSpheres::
        Rect* projection_buffer_ptr;
    __device__ ldplab::rtscuda::PipelineInitialHomogenousLightBoundingSpheres::
        HomogenousLightSource* light_source_buffer_ptr;
    __device__ size_t* num_rays_buffer_ptr;
    __device__ size_t* temp_num_rays_buffer_ptr;
    __device__ size_t total_num_rays;
    __device__ size_t num_particles;
    __device__ size_t num_light_sources;
}

ldplab::rtscuda::PipelineInitialHomogenousLightBoundingSpheres::
    PipelineInitialHomogenousLightBoundingSpheres(Context& context)
    :
    m_context{ context }
{ }

void ldplab::rtscuda::PipelineInitialHomogenousLightBoundingSpheres::setup()
{
    using namespace homogenous_light_bounding_spheres_cuda;
    const size_t grid_size = m_context.parameters.num_particles;
    const size_t block_size = m_context.parameters.num_light_sources;
    const size_t mem_size = block_size * sizeof(size_t);
    projectParticlesKernel<<<grid_size, block_size>>>(
        m_context.resources.bounding_volumes.bounding_volume_per_particle.get(),
        m_context.parameters.light_source_resolution_per_world_unit);
    countTotalRaysKernelFirst<<<grid_size, block_size, mem_size>>>(grid_size);
    countTotalRaysKernelSecond<<<grid_size, block_size, mem_size>>>(grid_size);
    // Download the total number of rays
    size_t total_rays;
    if (cudaMemcpyFromSymbol(
        &total_rays,
        total_num_rays,
        sizeof(total_num_rays)) != cudaSuccess)
    {
        total_rays = 0;
    }
    // Calculate the number of batches
    m_batch_ctr = 0;
    m_total_batch_count = total_rays / m_context.parameters.num_rays_per_batch +
        (total_rays % m_context.parameters.num_rays_per_batch ? 1 : 0);
}

ldplab::rtscuda::pipelineInitialStageCreateBatchKernel_t 
    ldplab::rtscuda::PipelineInitialHomogenousLightBoundingSpheres::getKernel()
{
    using namespace homogenous_light_bounding_spheres_cuda;
    pipelineInitialStageCreateBatchKernel_t kernel = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel,
        create_batch_kernel_ptr,
        sizeof(create_batch_kernel_ptr))
        != cudaSuccess)
        return nullptr;
    return kernel;
}

bool ldplab::rtscuda::PipelineInitialHomogenousLightBoundingSpheres::execute(
    size_t initial_ray_buffer_index)
{
    using namespace homogenous_light_bounding_spheres_cuda;
    const size_t block_size = m_context.parameters.num_threads_per_block;
    const size_t grid_size = m_context.parameters.num_rays_per_batch / block_size;
    createBatchKernel<<<block_size, grid_size>>>(
        m_context.resources.ray_buffer.index_buffers[initial_ray_buffer_index].get(),
        m_context.resources.ray_buffer.origin_buffers[initial_ray_buffer_index].get(),
        m_context.resources.ray_buffer.direction_buffers[initial_ray_buffer_index].get(),
        m_context.resources.ray_buffer.intensity_buffers[initial_ray_buffer_index].get(),
        m_context.resources.ray_buffer.min_bv_dist_buffers[initial_ray_buffer_index].get(),
        m_context.parameters.num_rays_per_batch,
        m_batch_ctr);
    ++m_batch_ctr;
    return m_batch_ctr < m_total_batch_count;
}

bool ldplab::rtscuda::PipelineInitialHomogenousLightBoundingSpheres::allocate(
    const ExperimentalSetup& setup, 
    const RayTracingStepCUDAInfo& info)
{
    using namespace homogenous_light_bounding_spheres_cuda;
    for (size_t i = 0; i < setup.light_sources.size(); ++i)
    {
        if (setup.light_sources[i].direction->type() !=
            ILightDirection::Type::homogeneous ||
            setup.light_sources[i].intensity_distribution->type() !=
            ILightDistribution::Type::homogeneous ||
            setup.light_sources[i].polarisation->type() !=
            ILightPolarisation::Type::unpolarized)
        {
            LDPLAB_LOG_ERROR("RTSCUDA context %i: Homogenous light initial "\
                "stage creation failed, light source %i has unsupported type",
                m_context.uid,
                setup.light_sources[i].uid);
            return false;
        }
    }
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        if (setup.particles[i].bounding_volume->type() !=
            IBoundingVolume::Type::sphere)
        {
            LDPLAB_LOG_ERROR("RTSCUDA context %i: Homogenous light bounding "\
                "sphere initial stage creation failed, particle %i bounding "\
                "volume is not a bounding sphere",
                m_context.uid,
                setup.particles[i].uid);
            return false;
        }
    }
    const size_t num_pl = setup.particles.size() * setup.light_sources.size();
    if (!m_light_source_buffer.allocate(setup.light_sources.size()))
        return false;
    if (!m_num_rays_buffer.allocate(num_pl))
        return false;
    if (!m_temp_num_rays_buffer.allocate(num_pl))
        return false;
    if (!m_projection_buffer.allocate(num_pl))
        return false;
    void* tptr = m_light_source_buffer.get();
    if (cudaMemcpyToSymbol(
        light_source_buffer_ptr, 
        &tptr,
        sizeof(light_source_buffer_ptr)) != cudaSuccess)
        return false;
    tptr = m_num_rays_buffer.get();
    if (cudaMemcpyToSymbol(
        num_rays_buffer_ptr,
        &tptr,
        sizeof(num_rays_buffer_ptr)) != cudaSuccess)
        return false;
    tptr = m_temp_num_rays_buffer.get();
    if (cudaMemcpyToSymbol(
        temp_num_rays_buffer_ptr,
        &tptr,
        sizeof(temp_num_rays_buffer_ptr)) != cudaSuccess)
        return false;
    tptr = m_projection_buffer.get();
    if (cudaMemcpyToSymbol(
        projection_buffer_ptr,
        &tptr,
        sizeof(projection_buffer_ptr)) != cudaSuccess)
        return false;
    if (cudaMemcpyToSymbol(
        num_particles,
        &m_context.parameters.num_particles,
        sizeof(num_particles)) != cudaSuccess)
        return false;
    if (cudaMemcpyToSymbol(
        num_light_sources,
        &m_context.parameters.num_light_sources,
        sizeof(num_light_sources)) != cudaSuccess)
        return false;
    // Create light sources
    std::vector<HomogenousLightSource> host_data;
    for (size_t i = 0; i < setup.light_sources.size(); ++i)
    {
        HomogenousLightSource light_source;
        light_source.origin = setup.light_sources[i].origin_corner;
        light_source.ray_direction = setup.light_sources[i].orientation;
        light_source.ray_intensity = static_cast<LightDistributionHomogeneous*>(
            setup.light_sources[i].intensity_distribution.get())->intensity;
        light_source.width = setup.light_sources[i].horizontal_size * 
            m_context.parameters.light_source_resolution_per_world_unit;
        light_source.x_axis = 
            glm::normalize(setup.light_sources[i].horizontal_direction) /
            m_context.parameters.light_source_resolution_per_world_unit;
        light_source.height = setup.light_sources[i].vertical_size *
            m_context.parameters.light_source_resolution_per_world_unit;
        light_source.y_axis = 
            glm::normalize(setup.light_sources[i].vertical_direction) /
            m_context.parameters.light_source_resolution_per_world_unit;
        host_data.push_back(light_source);
    }
    if (!m_light_source_buffer.upload(host_data.data()))
        return false;
    return true;
}

__global__ void homogenous_light_bounding_spheres_cuda::projectParticlesKernel(
        ldplab::rtscuda::GenericBoundingVolumeData* bounding_volumes,
        double light_source_resolution_per_world_unit)
{
    using namespace ldplab;
    using namespace rtscuda;
    const unsigned int projection_idx = threadIdx.x * blockDim.x + blockIdx.x;

    // ========================================================================
    // Part 1: Project sphere to plane
    PipelineInitialHomogenousLightBoundingSpheres::HomogenousLightSource light = 
        light_source_buffer_ptr[threadIdx.x];
    BoundingSphere::Data bs = *static_cast<BoundingSphere::Data*>(
        bounding_volumes[blockIdx.x].data);

    // Assuming ray direction is always orthogonal to light plane
    const double t = glm::dot(light.ray_direction, light.origin - bs.center) /
        -glm::dot(light.ray_direction, light.ray_direction);
    PipelineInitialHomogenousLightBoundingSpheres::Rect projection;
    if (t < 0.0)
    {
        projection.x = -1;
        projection.y = -1;
        projection.height = 0;
        projection.width = 0;
    }
    else
    {
        const Vec2 projctr = Vec2{
           glm::dot(bs.center - t * light.ray_direction - light.origin, light.x_axis),
           glm::dot(bs.center - t * light.ray_direction - light.origin, light.y_axis) } *
           light_source_resolution_per_world_unit;
        projection.x = static_cast<int>(
            projctr.x - bs.radius * light_source_resolution_per_world_unit);
        projection.y = static_cast<int>(
            projctr.y - bs.radius * light_source_resolution_per_world_unit);
        projection.width = static_cast<int>(projctr.x +
            bs.radius * light_source_resolution_per_world_unit) - projection.x;
        projection.height = static_cast<int>(projctr.y +
            bs.radius * light_source_resolution_per_world_unit) - projection.y;
    }
    projection_buffer_ptr[projection_idx] = projection;

    // ========================================================================
    // Part 2: calculate projection size
    const unsigned int tid = threadIdx.x;
    temp_num_rays_buffer_ptr[projection_idx] = 
        static_cast<size_t>(projection.width * projection.height);
}

__global__ void homogenous_light_bounding_spheres_cuda::countTotalRaysKernelFirst(
    size_t num_blocks)
{
    using namespace ldplab;
    using namespace rtscuda;

    // Shared memory
    extern __shared__ size_t sbuf[];

    // ========================================================================
    // Part 1: Load initial sums
    const unsigned int tid = threadIdx.x;
    const unsigned int gid = blockIdx.x * blockDim.x + tid;
    sbuf[tid] = temp_num_rays_buffer_ptr[gid];
    __syncthreads();

    // ========================================================================
    // Part 2: Compute sums
    for (unsigned int i = 1; i < blockDim.x; ++i)
    {
        if (i == tid)
            sbuf[tid] = sbuf[tid - 1];
        __syncthreads();
    }

    // ========================================================================
    // Part 3: Write back results
    temp_num_rays_buffer_ptr[gid] = sbuf[tid];
}

__global__ void homogenous_light_bounding_spheres_cuda::
    countTotalRaysKernelSecond(size_t num_blocks)
{
    using namespace ldplab;
    using namespace rtscuda;

    // Shared memory
    extern __shared__ size_t sbuf[];

    // ========================================================================
    // Part 1: Load initial sums
    const unsigned int tid = threadIdx.x;
    const unsigned int gid = blockIdx.x * blockDim.x + tid;
    sbuf[tid] = temp_num_rays_buffer_ptr[gid];
    __syncthreads();

    // ========================================================================
    // Part 2: Compute sums for each block up until this one
    for (unsigned int i = 0; i < blockIdx.x; ++i)
    {
        sbuf[tid] += temp_num_rays_buffer_ptr[(i + 1) * blockDim.x - 1];
        __syncthreads();
    }

    // ========================================================================
    // Part 3: Write back results
    num_rays_buffer_ptr[gid] = sbuf[tid];
    if (blockIdx.x == num_blocks - 1 && tid + 1 == blockDim.x)
        total_num_rays = sbuf[tid];
}

__global__ void homogenous_light_bounding_spheres_cuda::createBatchKernel(
        int32_t* ray_index_buffer, 
        ldplab::Vec3* ray_origin_buffer, 
        ldplab::Vec3* ray_direction_buffer, 
        double* ray_intensity_buffer,
        double* ray_min_bv_dist_buffer, 
        size_t num_rays_per_batch, 
        size_t batch_no)
{
    using namespace ldplab;
    using namespace rtscuda;

    // ========================================================================
    // Part 1: Find which projection to use for this instance using binary search
    unsigned int gid = batch_no * num_rays_per_batch + threadIdx.x;
    if (gid >= total_num_rays)
        return;
    size_t low = 0;
    size_t high = (num_particles * num_light_sources) - 1;
    size_t proj_idx, nr;
    PipelineInitialHomogenousLightBoundingSpheres::Rect proj;
    do
    {
        proj_idx = (high - low) / 2;
        nr = num_rays_buffer_ptr[proj_idx];
        proj = projection_buffer_ptr[proj_idx];
        if (gid < nr && gid >= nr - proj.width * proj.height)
            break;
        else if (gid < nr)
            high = proj_idx;
        else
            low = proj_idx;
    } while (low < high);

    // ========================================================================
    // Part 2: Find which ray to create
    unsigned int lid = gid - (nr - proj.width * proj.height);
    int xid = static_cast<int>(lid) % proj.width;
    int yid = static_cast<int>(lid) / proj.width;

    unsigned int ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (proj.x + xid < 0 ||
        proj.y + yid < 0 ||
        proj.x + xid >= proj.width ||
        proj.y + yid >= proj.height)
        ray_index_buffer[ri] = 0;
    else
    {
        PipelineInitialHomogenousLightBoundingSpheres::HomogenousLightSource 
            light = light_source_buffer_ptr[proj_idx % num_particles];
        ray_index_buffer[ri] = num_particles;
        ray_origin_buffer[ri] = light.origin +
            static_cast<double>(proj.x + xid) * light.x_axis +
            static_cast<double>(proj.y + yid) * light.y_axis;
        ray_direction_buffer[ri] = light.ray_direction;
        ray_intensity_buffer[ri] = light.ray_intensity;
        ray_min_bv_dist_buffer[ri] = 0.0;
    }
}

#endif