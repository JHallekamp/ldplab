#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "ImplSurfaceInteraction.hpp"

#include <LDPLAB/RayTracingStep/CUDA/IGenericMaterial.hpp>

namespace surface_interaction
{
    using namespace ldplab;
    using namespace rtscuda;
    __global__ void surfaceInteractionKernel(
        const int32_t* input_ray_index_buffer,
        const Vec3* input_ray_origin_buffer,
        const Vec3* input_ray_direction_buffer,
        const double* input_ray_intensity_buffer, 
        const int32_t* intersection_particle_index_buffer,
        const Vec3* intersection_point_buffer,
        const Vec3* intersection_normal_buffer, 
        int32_t* output_ray_index_buffer,
        Vec3* output_ray_origin_buffer,
        Vec3* output_ray_direction_buffer,
        double* output_ray_intensity_buffer,
        double* output_ray_min_bv_dist_buffer,
        double intensity_cutoff,
        double medium_reflection_index,
        IGenericMaterial::indexOfRefraction* material_index_of_refraction,
        void** material_data,
        Vec3* particle_center_of_mass,
        Vec3* output_force_per_ray_buffer,
        Vec3* output_torque_per_ray_buffer,
        bool pass_inner_particle_rays,
        bool pass_reflection,
        size_t num_rays_per_batch,
        size_t num_particles);
    __device__ double reflectance(
        double cos_alpha,
        double cos_beta,
        double n_r);
}

void ldplab::rtscuda::SurfaceInteraction::execute(
    const GlobalData& global_data,
    BatchData& batch_data,
    size_t ray_input_buffer_index,
    size_t ray_output_buffer_index,
    size_t intersection_buffer_index,
    size_t output_buffer_index,
    double intensity_cutoff,
    double medium_reflection_index,
    bool input_inner_particle_rays,
    bool reflection_pass,
    size_t pass_no)
{
    const size_t block_size = 128;
    const size_t grid_size =
        global_data.simulation_parameter.num_rays_per_batch / block_size +
        (global_data.simulation_parameter.num_rays_per_batch / block_size ? 1 : 0);
    using namespace surface_interaction;
    surfaceInteractionKernel<<<grid_size, block_size>>>(
        batch_data.ray_data_buffers.particle_index_buffers.getDeviceBuffer(ray_input_buffer_index),
        batch_data.ray_data_buffers.origin_buffers.getDeviceBuffer(ray_input_buffer_index),
        batch_data.ray_data_buffers.direction_buffers.getDeviceBuffer(ray_input_buffer_index),
        batch_data.ray_data_buffers.intensity_buffers.getDeviceBuffer(ray_input_buffer_index),
        batch_data.intersection_data_buffers.particle_index_buffers.getDeviceBuffer(intersection_buffer_index),
        batch_data.intersection_data_buffers.point_buffers.getDeviceBuffer(intersection_buffer_index),
        batch_data.intersection_data_buffers.normal_buffers.getDeviceBuffer(intersection_buffer_index),
        batch_data.ray_data_buffers.particle_index_buffers.getDeviceBuffer(ray_output_buffer_index),
        batch_data.ray_data_buffers.origin_buffers.getDeviceBuffer(ray_output_buffer_index),
        batch_data.ray_data_buffers.direction_buffers.getDeviceBuffer(ray_output_buffer_index),
        batch_data.ray_data_buffers.intensity_buffers.getDeviceBuffer(ray_output_buffer_index),
        batch_data.ray_data_buffers.min_bv_distance_buffers.getDeviceBuffer(ray_output_buffer_index),
        intensity_cutoff,
        medium_reflection_index,
        global_data.particle_data_buffers.index_of_refraction_fptr_buffer.getDeviceBuffer(),
        global_data.particle_data_buffers.material_data_buffer.getDeviceBuffer(),
        global_data.particle_data_buffers.center_of_mass_buffer.getDeviceBuffer(),
        batch_data.output_data_buffers.force_per_ray_buffer.getDeviceBuffer(output_buffer_index),
        batch_data.output_data_buffers.torque_per_ray_buffer.getDeviceBuffer(output_buffer_index),
        input_inner_particle_rays,
        reflection_pass,
        global_data.simulation_parameter.num_rays_per_batch,
        global_data.simulation_parameter.num_particles);
}

__global__ void surface_interaction::surfaceInteractionKernel(
    const int32_t* input_ray_index_buffer, 
    const Vec3* input_ray_origin_buffer, 
    const Vec3* input_ray_direction_buffer, 
    const double* input_ray_intensity_buffer, 
    const int32_t* intersection_particle_index_buffer, 
    const Vec3* intersection_point_buffer, 
    const Vec3* intersection_normal_buffer, 
    int32_t* output_ray_index_buffer, 
    Vec3* output_ray_origin_buffer, 
    Vec3* output_ray_direction_buffer, 
    double* output_ray_intensity_buffer, 
    double* output_ray_min_bv_dist_buffer, 
    double intensity_cutoff, 
    double medium_reflection_index, 
    IGenericMaterial::indexOfRefraction* material_index_of_refraction, 
    void** material_data, 
    Vec3* particle_center_of_mass, 
    Vec3* output_force_per_ray_buffer,
    Vec3* output_torque_per_ray_buffer,
    bool pass_inner_particle_rays, 
    bool pass_reflection,
    size_t num_rays_per_batch,
    size_t num_particles)
{
    unsigned int ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_batch)
        return;

    int32_t particle_index = input_ray_index_buffer[ri];
    if (particle_index < 0 || particle_index >= static_cast<int32_t>(num_particles))
    {
        output_ray_index_buffer[ri] = -1;
        return;
    }

    // Check if the intersection normal is 0, in which case the ray will 
    // be written into the transmission buffer without changes. This is
    // done because in the inner particle propagation stage, there can
    // occur situations where a ray is tangent to the geometry, in which
    // case it will intersect it, but no inner particle propagation will
    // actually occur. To ensure correct behavior in such case, the 
    // intersection normal is set to 0.
    const Vec3 intersection_normal = intersection_normal_buffer[ri];
    if (intersection_normal == Vec3(0, 0, 0))
    {
        if (pass_reflection)
            output_ray_index_buffer[ri] = -1;
        else
        {
            output_ray_index_buffer[ri] = particle_index;
            output_ray_origin_buffer[ri] = input_ray_origin_buffer[ri];
            output_ray_direction_buffer[ri] = input_ray_direction_buffer[ri];
            output_ray_intensity_buffer[ri] = input_ray_intensity_buffer[ri];
            output_ray_min_bv_dist_buffer[ri] = 0.0;
        }
        return;
    }

    const Vec3 intersection_point = intersection_point_buffer[ri];
    const double nx = (pass_inner_particle_rays ?
        material_index_of_refraction[particle_index](
            intersection_point, material_data[particle_index]) :
        medium_reflection_index);
    const double ny = (pass_inner_particle_rays ?
        medium_reflection_index :
        material_index_of_refraction[particle_index](
            intersection_point, material_data[particle_index]));
    const double nr = nx / ny;
    const double cos_a = 
        -glm::dot(input_ray_direction_buffer[ri], intersection_normal);
    const Vec3 r = intersection_point - particle_center_of_mass[particle_index];
    Vec3 delta_momentum;

    if (1.0 - nr * nr * (1.0 - cos_a * cos_a) >= 0)
    {
        const double cos_b = std::sqrt(1.0 - nr * nr * (1.0 - cos_a * cos_a));
        const double R = reflectance(cos_a, cos_b, nr);

        if (!pass_reflection)
        {
            const double intensity = input_ray_intensity_buffer[ri] * (1.0 - R);
            if (intensity > intensity_cutoff)
            {
                output_ray_index_buffer[ri] = particle_index;
                output_ray_intensity_buffer[ri] = intensity;
                output_ray_min_bv_dist_buffer[ri] = 0.0;
                output_ray_origin_buffer[ri] = intersection_point;
                output_ray_direction_buffer[ri] =
                    input_ray_direction_buffer[ri] +
                    intersection_normal * 2.0 * cos_a;
                delta_momentum = nx * (input_ray_direction_buffer[ri] -
                    output_ray_direction_buffer[ri]);
            }
            else
            {
                output_ray_index_buffer[ri] = -1;
                delta_momentum = intersection_normal * (nx * -2.0 * cos_a);
            }
            output_force_per_ray_buffer[ri] += intensity * delta_momentum;
            output_torque_per_ray_buffer[ri] +=
                intensity * glm::cross(r, delta_momentum);
        }
        else
        {
            const double intensity = input_ray_intensity_buffer[ri] * R;
            if (intensity > intensity_cutoff)
            {
                output_ray_index_buffer[ri] = particle_index;
                output_ray_min_bv_dist_buffer[ri] = 0;
                output_ray_origin_buffer[ri] = intersection_point;
                output_ray_direction_buffer[ri] =
                    nr * input_ray_direction_buffer[ri] +
                    intersection_normal * (-cos_b + nr * cos_a);
                delta_momentum =
                    nx * input_ray_direction_buffer[ri] -
                    ny * output_ray_direction_buffer[ri];
            }
            else
            {
                output_ray_index_buffer[ri] = -1;
                delta_momentum = intersection_normal * (ny * cos_b - nx * cos_a);
            }
            output_force_per_ray_buffer[ri] += intensity * delta_momentum;
            output_torque_per_ray_buffer[ri] +=
                intensity * glm::cross(r, delta_momentum);
        }
    }
    else if (pass_reflection) // toal reflected ray
    {
        output_ray_index_buffer[ri] = particle_index;
        output_ray_intensity_buffer[ri] = input_ray_intensity_buffer[ri];
        output_ray_min_bv_dist_buffer[ri] = 0;
        output_ray_origin_buffer[ri] = intersection_point;
        output_ray_direction_buffer[ri] =
            input_ray_direction_buffer[ri] +
            intersection_normal * 2.0 * cos_a;
        delta_momentum = nx * (input_ray_direction_buffer[ri] -
            output_ray_direction_buffer[ri]);
        output_force_per_ray_buffer[ri] +=
            output_ray_intensity_buffer[ri] * delta_momentum;
        output_torque_per_ray_buffer[ri] +=
            output_ray_intensity_buffer[ri] * glm::cross(r, delta_momentum);
    }
    else
        output_ray_index_buffer[ri] = -1;
}

__device__ double surface_interaction::reflectance(
    double cos_alpha, 
    double cos_beta, 
    double n_r)
{
    const double cos2_a = cos_alpha * cos_alpha;
    const double cos2_b = cos_beta * cos_beta;
    return (cos2_a - cos2_b) * (cos2_a - cos2_b) /
        (((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta) *
            ((cos2_a + cos2_b) + (n_r + 1 / n_r) * cos_alpha * cos_beta));
}

#endif