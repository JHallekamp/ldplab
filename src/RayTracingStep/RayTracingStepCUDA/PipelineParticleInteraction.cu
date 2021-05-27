#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineParticleInteraction.hpp"

#include "Context.hpp"

ldplab::rtscuda::PipelineParticleInteractionUnpolarized1DLinearIndexGradient::
    PipelineParticleInteractionUnpolarized1DLinearIndexGradient(Context& context)
    :
    m_context{ context }
{ }

ldplab::rtscuda::pipelineParticleInteractionStageKernel_t
    ldplab::rtscuda::PipelineParticleInteractionUnpolarized1DLinearIndexGradient::interaction_kernel_ptr =
        ldplab::rtscuda::PipelineParticleInteractionUnpolarized1DLinearIndexGradient::interactionKernel;

ldplab::rtscuda::pipelineParticleInteractionStageKernel_t 
    ldplab::rtscuda::PipelineParticleInteractionUnpolarized1DLinearIndexGradient::
        getKernel()
{
    // Copy the function pointer to the host
    pipelineParticleInteractionStageKernel_t kernel = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel,
        interaction_kernel_ptr,
        sizeof(interaction_kernel_ptr))
        != cudaSuccess)
        return nullptr;
    return kernel;
}

void ldplab::rtscuda::PipelineParticleInteractionUnpolarized1DLinearIndexGradient::
    execute(
        bool inner_particle_rays, 
        size_t input_ray_buffer_index, 
        size_t reflected_ray_buffer_index, 
        size_t transmitted_ray_buffer_index)
{
    const size_t block_size = 128;
    const size_t grid_size = m_context.parameters.num_rays_per_buffer / block_size;
    interactionKernel << <grid_size, block_size >> > (
        inner_particle_rays,
        m_context.parameters.medium_reflection_index,
        m_context.parameters.intensity_cutoff,
        m_context.resources.ray_buffer.index_buffers[input_ray_buffer_index].get(),
        m_context.resources.ray_buffer.origin_buffers[input_ray_buffer_index].get(),
        m_context.resources.ray_buffer.direction_buffers[input_ray_buffer_index].get(),
        m_context.resources.ray_buffer.intensity_buffers[input_ray_buffer_index].get(),
        m_context.resources.ray_buffer.index_buffers[reflected_ray_buffer_index].get(),
        m_context.resources.ray_buffer.origin_buffers[reflected_ray_buffer_index].get(),
        m_context.resources.ray_buffer.direction_buffers[reflected_ray_buffer_index].get(),
        m_context.resources.ray_buffer.intensity_buffers[reflected_ray_buffer_index].get(),
        m_context.resources.ray_buffer.min_bv_dist_buffers[reflected_ray_buffer_index].get(),
        m_context.resources.ray_buffer.index_buffers[transmitted_ray_buffer_index].get(),
        m_context.resources.ray_buffer.origin_buffers[transmitted_ray_buffer_index].get(),
        m_context.resources.ray_buffer.direction_buffers[transmitted_ray_buffer_index].get(),
        m_context.resources.ray_buffer.intensity_buffers[transmitted_ray_buffer_index].get(),
        m_context.resources.ray_buffer.min_bv_dist_buffers[transmitted_ray_buffer_index].get(),
        m_context.resources.intersection_buffer.intersection_particle_index_buffer.get(),
        m_context.resources.intersection_buffer.intersection_point_buffer.get(),
        m_context.resources.intersection_buffer.intersection_normal_buffer.get(),
        m_context.resources.output_buffer.force_per_ray.get(),
        m_context.resources.output_buffer.torque_per_ray.get(),
        m_context.parameters.num_rays_per_buffer,
        m_context.resources.particles.material_per_particle.get(),
        m_context.resources.particles.center_of_mass_per_particle.get(),
        m_context.parameters.num_particles);
}

__device__ double reflectance(double cos_a, double cos_b, double nr)
{
    const double cos2_a = cos_a * cos_a;
    const double cos2_b = cos_b * cos_b;
    return (cos2_a - cos2_b) * (cos2_a - cos2_b) /
        (((cos2_a + cos2_b) + (nr + 1 / nr) * cos_a * cos_b) *
            ((cos2_a + cos2_b) + (nr + 1 / nr) * cos_a * cos_b));
}

__device__ void ldplab::rtscuda::PipelineParticleInteractionUnpolarized1DLinearIndexGradient::
    interactionKernel(
        bool inner_particle_rays, 
        double medium_reflection_index,
        double intensity_cutoff,
        int32_t* input_ray_index_buffer, 
        Vec3* input_ray_origin_buffer, 
        Vec3* input_ray_direction_buffer, 
        double* input_ray_intensity_buffer, 
        int32_t* reflected_ray_index_buffer,
        Vec3* reflected_ray_origin_buffer,
        Vec3* reflected_ray_direction_buffer,
        double* reflected_ray_intensity_buffer,
        double* reflected_ray_min_bv_dist_buffer,
        int32_t* transmitted_ray_index_buffer,
        Vec3* transmitted_ray_origin_buffer,
        Vec3* transmitted_ray_direction_buffer, 
        double* transmitted_ray_intensity_buffer, 
        double* transmitted_ray_min_bv_dist_buffer, 
        int32_t* intersection_particle_index_buffer,
        Vec3* intersection_point_buffer, 
        Vec3* intersection_normal_buffer,
        Vec3* output_force_per_ray_buffer,
        Vec3* output_torque_per_ray_buffer, 
        size_t num_rays_per_buffer,
        GenericParticleMaterialData* particle_materials,
        Vec3* particle_center_of_mass,
        size_t num_particles)
{
    int ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_buffer)
        return;

    int32_t particle_index = input_ray_index_buffer[ri];
    if (particle_index < 0 || particle_index >= num_particles)
    {
        reflected_ray_index_buffer[ri] = -1;
        transmitted_ray_index_buffer[ri] = -1;
        return;
    }

    // Check if the intersection normal is 0, in which case the ray will 
    // be written into the transmission buffer without changes. This is
    // done because in the inner particle propagation stage, there can
    // occur situations where a ray is tangent to the geometry, in which
    // case it will intersect it, but no inner particle propagation will
    // actually occur. To ensure correct behavior in such case, the 
    // intersection normal is set to 0.
    Vec3 intersection_normal = intersection_normal_buffer[ri];
    if (intersection_normal == Vec3(0, 0, 0))
    {
        reflected_ray_index_buffer[ri] = -1;
        transmitted_ray_index_buffer[ri] = particle_index;
        transmitted_ray_origin_buffer[ri] = input_ray_origin_buffer[ri];
        transmitted_ray_direction_buffer[ri] = input_ray_direction_buffer[ri];
        transmitted_ray_intensity_buffer[ri] = input_ray_intensity_buffer[ri];
        transmitted_ray_min_bv_dist_buffer[ri] = 0.0;
        return;
    }

    // Get the material pointer
    const ParticleLinearOneDirectionalMaterial::Data* material =
        static_cast<ParticleLinearOneDirectionalMaterial::Data*>(
            particle_materials[particle_index].data);
    const Vec3 intersection_point = intersection_point_buffer[ri];
    double nr, nx, ny;    
    if (inner_particle_rays)
    {
        nx = material->indexOfRefraction(intersection_point);
        ny = medium_reflection_index;
    }
    else
    {
        nx = medium_reflection_index;
        ny = material->indexOfRefraction(intersection_point);
    }
    nr = nx / ny;
    const double cos_a = 
        -glm::dot(input_ray_direction_buffer[ri], intersection_normal);
    const Vec3 r = intersection_point -
        particle_center_of_mass[particle_index];
    Vec3 delta_momentum;

    if (1.0 - nr * nr * (1.0 - cos_a * cos_a) >= 0)
    {
        const double cos_b = std::sqrt(1.0 - nr * nr * (1.0 - cos_a * cos_a));
        const double R = reflectance(cos_a, cos_b, nr);

        // Refracted ray
        transmitted_ray_intensity_buffer[ri] =
            input_ray_intensity_buffer[ri] * (1.0 - R);
        if (transmitted_ray_intensity_buffer[ri] > intensity_cutoff)
        {
            transmitted_ray_index_buffer[ri] = particle_index;
            transmitted_ray_min_bv_dist_buffer[ri] = 0;
            transmitted_ray_origin_buffer[ri] = intersection_point;
            transmitted_ray_direction_buffer[ri] =
                nr * input_ray_direction_buffer[ri] +
                intersection_normal * (-cos_b + nr * cos_a);
            delta_momentum =
                nx * input_ray_direction_buffer[ri] -
                ny * transmitted_ray_direction_buffer[ri];
        }
        else
        {
            transmitted_ray_index_buffer[ri] = -1;
            delta_momentum = intersection_normal * (ny * cos_b - nx * cos_a);
        }
        output_force_per_ray_buffer[ri] +=
            transmitted_ray_intensity_buffer[ri] * delta_momentum;
        output_torque_per_ray_buffer[ri] +=
            transmitted_ray_intensity_buffer[ri] * glm::cross(r, delta_momentum);

        // Reflected ray
        reflected_ray_intensity_buffer[ri] =
            input_ray_intensity_buffer[ri] * R;
        if (reflected_ray_intensity_buffer[ri] > intensity_cutoff)
        {
            reflected_ray_index_buffer[ri] = particle_index;
            reflected_ray_min_bv_dist_buffer[ri] = 0;
            reflected_ray_origin_buffer[ri] = intersection_point;
            reflected_ray_direction_buffer[ri] =
                input_ray_direction_buffer[ri] + 
                intersection_normal * 2.0 * cos_a;
            delta_momentum = nx * (input_ray_direction_buffer[ri] -
                reflected_ray_direction_buffer[ri]);
        }
        else
        {
            reflected_ray_index_buffer[ri] = -1;
            delta_momentum = intersection_normal * (nx * -2.0 * cos_a);
        }
        output_force_per_ray_buffer[ri] +=
            transmitted_ray_intensity_buffer[ri] * delta_momentum;
        output_torque_per_ray_buffer[ri] +=
            transmitted_ray_intensity_buffer[ri] * glm::cross(r, delta_momentum);
    }
    else
    {
        // Total reflection
        transmitted_ray_index_buffer[ri] = -1;
        reflected_ray_index_buffer[ri] = particle_index;
        reflected_ray_intensity_buffer[ri] = input_ray_intensity_buffer[ri];
        reflected_ray_min_bv_dist_buffer[ri] = 0;
        reflected_ray_origin_buffer[ri] = intersection_point;
        reflected_ray_direction_buffer[ri] =
            input_ray_direction_buffer[ri] +
            intersection_normal * 2.0 * cos_a;
        delta_momentum = nx * (input_ray_direction_buffer[ri] - 
            reflected_ray_direction_buffer[ri]);
        output_force_per_ray_buffer[ri] +=
            transmitted_ray_intensity_buffer[ri] * delta_momentum;
        output_torque_per_ray_buffer[ri] +=
            transmitted_ray_intensity_buffer[ri] * glm::cross(r, delta_momentum);
    }
}

#endif