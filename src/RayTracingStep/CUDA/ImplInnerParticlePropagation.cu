#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "ImplInnerParticlePropagation.hpp"

#include <LDPLAB/RayTracingStep/CUDA/IGenericGeometry.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IGenericMaterial.hpp>

#include "ImplGenericMaterial.hpp"

#include <iostream>

namespace rk4
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;
    constexpr size_t block_size = 128;
    /**
     * @brief Structure keeping all variables of the differential
     *        equation.
     */
    struct Arg
    {
        /**
         * @brief Vector pointing in the direction of light. Its norm
         *        is the index of reflection at position r.
         */
        Vec3 w;
        /**
         * @brief Vector pointing to the light rays origin.
         */
        Vec3 r;
    };
    __global__ void innerParticlePropagationKernel(
        double step_size,
        int32_t* ray_index_buffer,
        Vec3* ray_origin_buffer,
        Vec3* ray_direction_buffer,
        double* ray_intensity_buffer,
        Vec3* intersection_point_buffer,
        Vec3* intersection_normal_buffer,
        size_t num_rays_per_batch,
        IGenericGeometry::intersectSegment* geometry_intersect_segment_functions,
        IGenericGeometry::intersectRay* geometry_intersect_ray_functions,
        void** geometry_data,
        IGenericMaterial::indexOfRefraction* material_ior_functions,
        void** material_data,
        Vec3* particle_center_of_mass,
        Vec3* output_force_per_ray,
        Vec3* output_torque_per_ray,
        size_t num_particles);
    __device__ void rk4(
        const MaterialLinearOneDirectionalData& material_data,
        const Arg& x,
        const double h,
        Arg& x_new);
}

ldplab::rtscuda::InnerParticlePropagationRK4::InnerParticlePropagationRK4(
	const RK4Parameter& parameter)
	:
	m_parameter{ parameter }
{ }

void ldplab::rtscuda::InnerParticlePropagationRK4::execute(
    StreamContext& smctx,
	size_t ray_buffer_index, 
	size_t intersection_buffer_index,
    size_t output_buffer_index,
    size_t num_rays)
{
    using namespace rk4;
    const size_t grid_size = num_rays / block_size + (num_rays % block_size ? 1 : 0);
    innerParticlePropagationKernel<<<grid_size, block_size, 0, smctx.cudaStream()>>>(
        m_parameter.step_size,
        smctx.rayDataBuffers().particle_index_buffers.getDeviceBuffer(ray_buffer_index),
        smctx.rayDataBuffers().origin_buffers.getDeviceBuffer(ray_buffer_index),
        smctx.rayDataBuffers().direction_buffers.getDeviceBuffer(ray_buffer_index),
        smctx.rayDataBuffers().intensity_buffers.getDeviceBuffer(ray_buffer_index),
        smctx.intersectionDataBuffers().point_buffers.getDeviceBuffer(intersection_buffer_index),
        smctx.intersectionDataBuffers().normal_buffers.getDeviceBuffer(intersection_buffer_index),
        num_rays,
        smctx.particleDataBuffers().intersect_segment_fptr_buffer.getDeviceBuffer(),
        smctx.particleDataBuffers().intersect_ray_fptr_buffer.getDeviceBuffer(),
        smctx.particleDataBuffers().geometry_data_buffer.getDeviceBuffer(),
        smctx.particleDataBuffers().index_of_refraction_fptr_buffer.getDeviceBuffer(),
        smctx.particleDataBuffers().material_data_buffer.getDeviceBuffer(),
        smctx.particleDataBuffers().center_of_mass_buffer.getDeviceBuffer(),
        smctx.outputDataBuffers().force_per_ray_buffer.getDeviceBuffer(output_buffer_index),
        smctx.outputDataBuffers().torque_per_ray_buffer.getDeviceBuffer(output_buffer_index),
        smctx.simulationParameter().num_particles);
}

__global__ void rk4::innerParticlePropagationKernel(
    double step_size,
    int32_t* ray_index_buffer,
    Vec3* ray_origin_buffer,
    Vec3* ray_direction_buffer,
    double* ray_intensity_buffer,
    Vec3* intersection_point_buffer,
    Vec3* intersection_normal_buffer,
    size_t num_rays_per_batch,
    IGenericGeometry::intersectSegment* geometry_intersect_segment_functions,
    IGenericGeometry::intersectRay* geometry_intersect_ray_functions,
    void** geometry_data,
    IGenericMaterial::indexOfRefraction* material_ior_functions,
    void** material_data,
    Vec3* particle_center_of_mass,
    Vec3* output_force_per_ray,
    Vec3* output_torque_per_ray,
    size_t num_particles)
{
    __shared__ Vec3 t_force[block_size];
    __shared__ Vec3 t_torque[block_size];

    size_t ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_batch)
        return;

    const int32_t particle_index = ray_index_buffer[ri];
    if (particle_index < 0 ||
        particle_index >= static_cast<int32_t>(num_particles))
        return;

    t_force[threadIdx.x] = Vec3(0);
    t_torque[threadIdx.x] = Vec3(0);

    const double ray_intesity = ray_intensity_buffer[ri];
    const Vec3 ray_origin = ray_origin_buffer[ri];
    Vec3 ray_direction = ray_direction_buffer[ri];
    void* const particle_geometry = geometry_data[particle_index];
    IGenericGeometry::intersectSegment intersectSegment =
        geometry_intersect_segment_functions[particle_index];
    const auto material = *static_cast<MaterialLinearOneDirectionalData*>(
        material_data[particle_index]);
    bool intersected = false;
    bool inter_side_indicator = false;
    Vec3 inter_point;
    Vec3 inter_normal;
    Vec3 center_of_mass = particle_center_of_mass[particle_index];
    Arg x{
        ray_direction * (material.index_of_refraction_minus_partial_dot +
            glm::dot(material.direction_times_gradient, ray_origin)),
        ray_origin
    };
    Arg x_new{ };
    double nx, ny;
    ny = (material.index_of_refraction_minus_partial_dot +
        glm::dot(material.direction_times_gradient, x.r));
    while (!intersected)
    {
        rk4::rk4(
            material,
            x, 
            step_size, 
            x_new);
        intersected = intersectSegment(
            x.r,
            x_new.r,
            particle_geometry,
            inter_point,
            inter_normal,
            inter_side_indicator);
        if (intersected || !inter_side_indicator)
        {
            if (!intersected)
            {
                // The following check is neccessary to test if the ray hits in
                // an extreme point (volume so small that it lies within the 
                // epsilon region). If that is the case, we assume the ray 
                // tunnels through the particle.
                intersected = geometry_intersect_ray_functions[particle_index](
                    ray_origin,
                    ray_direction,
                    particle_geometry,
                    inter_point,
                    inter_normal,
                    inter_side_indicator);
                if (!intersected || inter_side_indicator)
                {
                    // We have found a case, where the ray tunnels through the
                    // Particle. We use the original ray and invalidate the 
                    // surface normal.
                    inter_point = ray_origin;
                    inter_normal = Vec3(0, 0, 0);
                }
                else
                {
                    // We have found a case, where the initial ray is bend 
                    // out of the particle in the initial step due to the 
                    // particle material gradient. In this case we assume that 
                    // it hits in the original intersection point.
                    // We use the previous normal and have to flip it, to 
                    // ensure that we have correct behaviour in the interaction
                    // stage.
                    // To receive the previous normal, we simply perform the
                    // Intersection test again, but this time we reverse the
                    // segment directions. Then we flip the normal.
                    intersectSegment(
                        x_new.r,
                        x.r,
                        particle_geometry,
                        inter_point,
                        inter_normal,
                        inter_side_indicator);
                    inter_point = ray_origin;
                    inter_normal = -inter_normal;
                    ray_direction = glm::normalize(x.w);
                }
            }
            else
            {
                ray_direction = glm::normalize(x.w);
                nx = ny;
                ny = (material.index_of_refraction_minus_partial_dot +
                    glm::dot(material.direction_times_gradient, inter_point));
                const Vec3 delta_momentum = (nx - ny) * ray_direction;
                const Vec3 r = inter_point - center_of_mass;
                t_force[threadIdx.x] +=
                    ray_intesity * delta_momentum;
                t_torque[threadIdx.x] +=
                    ray_intesity * glm::cross(r, delta_momentum);
            }
            intersected = true;
        }
        else
        {
            nx = ny;
            ny = (material.index_of_refraction_minus_partial_dot +
                glm::dot(material.direction_times_gradient, x_new.r));
            const Vec3 t_old_direction = glm::normalize(x.w);
            const Vec3 t_new_direction = glm::normalize(x_new.w);
            const Vec3 delta_momentum =
                nx * t_old_direction -
                ny * t_new_direction;
            const Vec3 r = x_new.r - center_of_mass;
            t_force[threadIdx.x] +=
                ray_intesity * delta_momentum;
            t_torque[threadIdx.x] +=
                ray_intesity * glm::cross(r, delta_momentum);
            x = x_new;
        }
    }
    ray_direction_buffer[ri] = ray_direction;
    intersection_point_buffer[ri] = inter_point;
    intersection_normal_buffer[ri] = inter_normal;
    output_force_per_ray[ri] += t_force[threadIdx.x];
    output_torque_per_ray[ri] += t_torque[threadIdx.x];
}

__device__ void rk4::rk4(
    const MaterialLinearOneDirectionalData& material_data,
    const Arg& x,
    const double h, 
    Arg& x_new)
{
    constexpr double beta[4] = { 1.0, 0.5, 0.5, 1.0 };
    constexpr double c[4] = { 1.0, 2.0, 2.0, 1.0 };
    x_new = { Vec3(0), Vec3(0) };
    Arg k;
    k.w = material_data.direction_times_gradient;
    for (size_t i = 0; i < 4; ++i)
    {
        Arg x_step = x;
        if (i > 0)
        {
            const double hb = h * beta[i];
            x_step.w += k.w * hb;
            x_step.r += k.r * hb;
        }
        // We can inline the call to the indexOfRefraction method, because the
        // material is known by the stage type
        k.r = x_step.w / (material_data.index_of_refraction_minus_partial_dot +
            glm::dot(material_data.direction_times_gradient, x_step.r));
            //index_of_refraction(x_step.r, material_data);
        x_new.w += k.w * c[i];
        x_new.r += k.r * c[i];
    }
    x_new.w *= h / 6.0;
    x_new.r *= h / 6.0;
    x_new.w += x.w;
    x_new.r += x.r;
}

namespace rk4_queue
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;
    using Arg = rk4::Arg;
    constexpr size_t block_size = 32;
    constexpr size_t reload_threshold = 0;
    constexpr size_t steps_per_reload = 6;
    struct RayData
    {
        Vec3 origin;
        double intensity;
        Vec3 direction;
        void* particle_geometry_data;
        Vec3 particle_center_of_mass;
        IGenericGeometry::intersectSegment particle_intersect_segment_function;
        MaterialLinearOneDirectionalData particle_material_data;
    };
    struct RuntimeData
    {
        Arg x;
        Arg x_new;
        double nx;
        double ny;
    };
    __global__ void executeKernel(
        double step_size,
        int32_t* ray_index_buffer,
        Vec3* ray_origin_buffer,
        Vec3* ray_direction_buffer,
        double* ray_intensity_buffer,
        Vec3* intersection_point_buffer,
        Vec3* intersection_normal_buffer,
        size_t num_rays,
        IGenericGeometry::intersectSegment* geometry_intersect_segment_functions,
        IGenericGeometry::intersectRay* geometry_intersect_ray_functions,
        void** geometry_data,
        void** material_data,
        Vec3* particle_center_of_mass,
        Vec3* output_force_per_ray,
        Vec3* output_torque_per_ray,
        uint32_t* queue_ctr);
    __device__ void step(
        const RayData& ray_data,
        RuntimeData& rt_data,
        double step_size,
        IGenericGeometry::intersectRay* geometry_intersect_ray_functions,
        int32_t* ray_index_buffer,
        Vec3* ray_direction_buffer,
        Vec3* intersection_point_buffer,
        Vec3* intersection_normal_buffer,
        Vec3* output_force_per_ray,
        Vec3* output_torque_per_ray,
        bool running_flag[],
        uint32_t load_offset[],
        Vec3 temp_output_force[],
        Vec3 temp_output_torque[]);
    __device__ void countRunningThreads(
        bool running_flag[],
        uint32_t load_offset[],
        uint32_t& running_threads);
    __device__ void reloadRays(
        RayData& ray_data,
        RuntimeData& runtime_data,
        int32_t* ray_index_buffer,
        Vec3* ray_origin_buffer,
        Vec3* ray_direction_buffer,
        double* ray_intensity_buffer,
        IGenericGeometry::intersectSegment* intersect_segment_function_buffer,
        void** geometry_data,
        void** material_data,
        Vec3* particle_center_of_mass,
        size_t num_rays,
        uint32_t* queue_ctr,
        bool running_flag[],
        uint32_t load_offset[],
        Vec3 temp_output_force[],
        Vec3 temp_output_torque[],
        uint32_t& running_threads,
        bool& queue_empty);
}

ldplab::rtscuda::InnerParticlePropagationRK4QueueFill::
InnerParticlePropagationRK4QueueFill(
    const RK4Parameter& parameter,
    std::vector<DeviceBufferPinned<uint32_t>>&& queue_ctr_per_stream)
    :
    m_parameter{ parameter },
    m_queue_ctr_per_stream{ std::move(queue_ctr_per_stream) }
{ }

void ldplab::rtscuda::InnerParticlePropagationRK4QueueFill::execute(
    StreamContext& stream_context,
    size_t ray_buffer_index,
    size_t intersection_buffer_index,
    size_t output_buffer_index,
    size_t num_rays)
{ 
    using namespace rk4_queue;
    
    auto& ctr_buf = m_queue_ctr_per_stream[stream_context.streamId()];    
    ctr_buf.getHostBuffer()[0] = 0;
    ctr_buf.uploadAsync(stream_context.cudaStream());
    stream_context.synchronizeOnStream();

    size_t grid_size = stream_context.deviceProperties().num_mps * 16;
    if (grid_size > num_rays / block_size + (num_rays % block_size ? 1 : 0))
        grid_size = num_rays / block_size + (num_rays % block_size ? 1 : 0);
    executeKernel<<<grid_size, block_size, 0, stream_context.cudaStream()>>>(
        m_parameter.step_size,
        stream_context.rayDataBuffers().particle_index_buffers.getDeviceBuffer(ray_buffer_index),
        stream_context.rayDataBuffers().origin_buffers.getDeviceBuffer(ray_buffer_index),
        stream_context.rayDataBuffers().direction_buffers.getDeviceBuffer(ray_buffer_index),
        stream_context.rayDataBuffers().intensity_buffers.getDeviceBuffer(ray_buffer_index),
        stream_context.intersectionDataBuffers().point_buffers.getDeviceBuffer(intersection_buffer_index),
        stream_context.intersectionDataBuffers().normal_buffers.getDeviceBuffer(intersection_buffer_index),
        num_rays,
        stream_context.particleDataBuffers().intersect_segment_fptr_buffer.getDeviceBuffer(),
        stream_context.particleDataBuffers().intersect_ray_fptr_buffer.getDeviceBuffer(),
        stream_context.particleDataBuffers().geometry_data_buffer.getDeviceBuffer(),
        stream_context.particleDataBuffers().material_data_buffer.getDeviceBuffer(),
        stream_context.particleDataBuffers().center_of_mass_buffer.getDeviceBuffer(),
        stream_context.outputDataBuffers().force_per_ray_buffer.getDeviceBuffer(output_buffer_index),
        stream_context.outputDataBuffers().torque_per_ray_buffer.getDeviceBuffer(output_buffer_index),
        ctr_buf.getDeviceBuffer());
}

__global__ void rk4_queue::executeKernel(
    double step_size, 
    int32_t* ray_index_buffer, 
    Vec3* ray_origin_buffer, 
    Vec3* ray_direction_buffer, 
    double* ray_intensity_buffer, 
    Vec3* intersection_point_buffer, 
    Vec3* intersection_normal_buffer, 
    size_t num_rays,
    IGenericGeometry::intersectSegment* geometry_intersect_segment_functions, 
    IGenericGeometry::intersectRay* geometry_intersect_ray_functions, 
    void** geometry_data,
    void** material_data, 
    Vec3* particle_center_of_mass, 
    Vec3* output_force_per_ray, 
    Vec3* output_torque_per_ray, 
    uint32_t* queue_ctr)
{
    // Shared memory
    __shared__ bool running_flag[block_size];
    __shared__ uint32_t load_offset[block_size];
    __shared__ Vec3 temp_output_force[block_size];
    __shared__ Vec3 temp_output_torque[block_size];
    __shared__ uint32_t running_threads;
    __shared__ bool queue_empty;

    // Setup
    running_flag[threadIdx.x] = false;
    load_offset[threadIdx.x] = threadIdx.x;
    if (threadIdx.x == 0)
    {
        queue_empty = false;
        running_threads = 0;
    }
    __syncthreads();

    RayData ray_data;
    RuntimeData rt_data;

    // Main loop
    while (!queue_empty || running_threads > 0)
    {
        if (running_threads < block_size - reload_threshold && !queue_empty)
        {
            reloadRays(
                ray_data,
                rt_data,
                ray_index_buffer,
                ray_origin_buffer,
                ray_direction_buffer,
                ray_intensity_buffer,
                geometry_intersect_segment_functions,
                geometry_data,
                material_data,
                particle_center_of_mass,
                num_rays,
                queue_ctr,
                running_flag,
                load_offset,
                temp_output_force,
                temp_output_torque,
                running_threads,
                queue_empty);
        }
        for (uint32_t i = 0; i < steps_per_reload; ++i)
            step(
                ray_data,
                rt_data,
                step_size,
                geometry_intersect_ray_functions,
                ray_index_buffer,
                ray_direction_buffer,
                intersection_point_buffer,
                intersection_normal_buffer,
                output_force_per_ray,
                output_torque_per_ray,
                running_flag,
                load_offset,
                temp_output_force,
                temp_output_torque);
        countRunningThreads(running_flag, load_offset, running_threads);
    }

    return;
}

__device__ void rk4_queue::step(
    const RayData& ray_data, 
    RuntimeData& rt_data, 
    double step_size,
    IGenericGeometry::intersectRay* geometry_intersect_ray_functions,
    int32_t* ray_index_buffer,
    Vec3* ray_direction_buffer,
    Vec3* intersection_point_buffer, 
    Vec3* intersection_normal_buffer, 
    Vec3* output_force_per_ray, 
    Vec3* output_torque_per_ray, 
    bool running_flag[], 
    uint32_t load_offset[], 
    Vec3 temp_output_force[], 
    Vec3 temp_output_torque[])
{
    if (!running_flag[threadIdx.x])
        return;
    rk4::rk4(
        ray_data.particle_material_data,
        rt_data.x,
        step_size,
        rt_data.x_new);
    Vec3 inter_point, inter_normal;
    bool inter_inside_indicator;
    bool intersected = ray_data.particle_intersect_segment_function(
        rt_data.x.r,
        rt_data.x_new.r,
        ray_data.particle_geometry_data,
        inter_point,
        inter_normal,
        inter_inside_indicator);
    if (!intersected && inter_inside_indicator)
    {
        rt_data.nx = rt_data.ny;
        rt_data.ny = ray_data.particle_material_data.index_of_refraction_minus_partial_dot +
            glm::dot(ray_data.particle_material_data.direction_times_gradient, rt_data.x_new.r);
        const Vec3 delta_momentum =
            rt_data.nx * glm::normalize(rt_data.x.w) -
            rt_data.ny * glm::normalize(rt_data.x_new.w);
        temp_output_force[threadIdx.x] += ray_data.intensity * delta_momentum;
        temp_output_torque[threadIdx.x] += ray_data.intensity *
            glm::cross(rt_data.x_new.r - ray_data.particle_center_of_mass, delta_momentum);
        rt_data.x = rt_data.x_new;
    }
    else
    {
        const uint32_t ri = load_offset[threadIdx.x];
        if (!intersected)
        {
            // The following check is neccessary to test if the ray hits in
            // an extreme point (volume so small that it lies within the 
            // epsilon region). If that is the case, we assume the ray 
            // tunnels through the particle.
            intersected = geometry_intersect_ray_functions[ray_index_buffer[ri]](
                ray_data.origin,
                ray_data.direction,
                ray_data.particle_geometry_data,
                inter_point,
                inter_normal,
                inter_inside_indicator);
            if (!intersected || inter_inside_indicator)
            {
                // We have found a case, where the ray tunnels through the
                // Particle. We use the original ray and invalidate the 
                // surface normal.
                inter_point = ray_data.origin;
                inter_normal = Vec3(0);
                rt_data.x.w = ray_data.direction;
            }
            else
            {
                // We have found a case, where the initial ray is bend 
                // out of the particle in the initial step due to the 
                // particle material gradient. In this case we assume that 
                // it hits in the original intersection point.
                // We use the previous normal and have to flip it, to 
                // ensure that we have correct behaviour in the interaction
                // stage.
                // To receive the previous normal, we simply perform the
                // Intersection test again, but this time we reverse the
                // segment directions. Then we flip the normal.
                ray_data.particle_intersect_segment_function(
                    rt_data.x_new.r,
                    rt_data.x.r,
                    ray_data.particle_geometry_data,
                    inter_point,
                    inter_normal,
                    inter_inside_indicator);
                inter_point = ray_data.origin;
                inter_normal = -inter_normal;
            }
        }
        else
        {
            rt_data.nx = rt_data.ny;
            rt_data.ny = ray_data.particle_material_data.index_of_refraction_minus_partial_dot +
                glm::dot(ray_data.particle_material_data.direction_times_gradient, inter_point);
            const Vec3 delta_momentum = 
                (rt_data.nx - rt_data.ny) * glm::normalize(rt_data.x.w);
            temp_output_force[threadIdx.x] += ray_data.intensity * delta_momentum;
            temp_output_torque[threadIdx.x] += ray_data.intensity *
                glm::cross(inter_point - ray_data.particle_center_of_mass, delta_momentum);
        }

        ray_direction_buffer[ri] = glm::normalize(rt_data.x.w);
        intersection_point_buffer[ri] = inter_point;
        intersection_normal_buffer[ri] = inter_normal;
        output_force_per_ray[ri] += temp_output_force[threadIdx.x];
        output_torque_per_ray[ri] += temp_output_torque[threadIdx.x];
        running_flag[threadIdx.x] = false;
    }
}

__device__ void rk4_queue::countRunningThreads(
    bool running_flag[], 
    uint32_t load_offset[], 
    uint32_t& running_threads)
{
    __syncthreads();
    if (threadIdx.x == 0)
    {
        running_threads = 0;
        for (size_t i = 0; i < block_size; ++i)
        {
            if (running_flag[i])
                ++running_threads;
            else
                load_offset[i] = i - running_threads;
        }
    }
    __syncthreads();
}

__device__ void rk4_queue::reloadRays(
    RayData& ray_data,
    RuntimeData& runtime_data, 
    int32_t* ray_index_buffer, 
    Vec3* ray_origin_buffer, 
    Vec3* ray_direction_buffer, 
    double* ray_intensity_buffer, 
    IGenericGeometry::intersectSegment* intersect_segment_function_buffer, 
    void** geometry_data, 
    void** material_data, 
    Vec3* particle_center_of_mass, 
    size_t num_rays,
    uint32_t* queue_ctr,
    bool running_flag[], 
    uint32_t load_offset[], 
    Vec3 temp_output_force[], 
    Vec3 temp_output_torque[], 
    uint32_t& running_threads, 
    bool& queue_empty)
{
    __shared__ uint32_t offset;

    __syncthreads();
    if (threadIdx.x == 0)
    {
        if (queue_empty)
            offset = num_rays;
        else
        {
            offset = atomicAdd(queue_ctr, block_size - running_threads);
            if (offset + block_size - running_threads >= num_rays)
                queue_empty = true;
        }
    }
    __syncthreads();

    if (!running_flag[threadIdx.x])
    {
        load_offset[threadIdx.x] += offset;
        const uint32_t ri = load_offset[threadIdx.x];
        if (ri < num_rays)
        {
            const int32_t pi = ray_index_buffer[ri];
            if (pi < 0)
                return;
            
            ray_data.origin = ray_origin_buffer[ri];
            ray_data.direction = ray_direction_buffer[ri];
            ray_data.intensity = ray_intensity_buffer[ri];

            ray_data.particle_intersect_segment_function =
                intersect_segment_function_buffer[pi];
            ray_data.particle_geometry_data =
                geometry_data[pi];
            ray_data.particle_material_data =
                *static_cast<MaterialLinearOneDirectionalData*>(material_data[pi]);
            ray_data.particle_center_of_mass =
                particle_center_of_mass[pi];

            runtime_data.x = {
                ray_data.direction *
                (ray_data.particle_material_data.index_of_refraction_minus_partial_dot +
                    glm::dot(ray_data.particle_material_data.direction_times_gradient, ray_data.origin)),
                ray_data.origin
            };
            runtime_data.ny = ray_data.particle_material_data.index_of_refraction_minus_partial_dot +
                glm::dot(ray_data.particle_material_data.direction_times_gradient, runtime_data.x.r);

            temp_output_force[threadIdx.x] = Vec3(0);
            temp_output_torque[threadIdx.x] = Vec3(0);
            running_flag[threadIdx.x] = true;
        }
    }
}

#endif