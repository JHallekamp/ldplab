#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "ImplInnerParticlePropagation.hpp"

#include <LDPLAB/RayTracingStep/CUDA/IGenericGeometry.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IGenericMaterial.hpp>

#include "ImplGenericMaterial.hpp"

namespace rk4
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;
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
        IGenericMaterial::indexOfRefraction index_of_refraction,
        MaterialLinearOneDirectionalData* material_data,
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
    const size_t block_size = 192;
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
    using namespace ldplab;
    using namespace rtscuda;
    size_t ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_batch)
        return;

    const int32_t particle_index = ray_index_buffer[ri];
    if (particle_index < 0 ||
        particle_index >= static_cast<int32_t>(num_particles))
        return;

    const double ray_intesity = ray_intensity_buffer[ri];
    const Vec3 ray_origin = ray_origin_buffer[ri];
    Vec3 ray_direction = ray_direction_buffer[ri];
    void* const particle_geometry = geometry_data[particle_index];
    IGenericGeometry::intersectRay intersectRay =
        geometry_intersect_ray_functions[particle_index];
    IGenericGeometry::intersectSegment intersectSegment =
        geometry_intersect_segment_functions[particle_index];
    void* const particle_material = material_data[particle_index];
    IGenericMaterial::indexOfRefraction indexOfRefraction =
        material_ior_functions[particle_index];
    bool intersected = false;
    bool is_inside = false;
    Vec3 inter_point;
    Vec3 inter_normal;
    Arg x{
        ray_direction * indexOfRefraction(ray_origin, particle_material),
        ray_origin
    };
    Arg x_new{ };
    while (!intersected)
    {
        rk4::rk4(
            indexOfRefraction, 
            static_cast<MaterialLinearOneDirectionalData*>(particle_material), 
            x, 
            step_size, 
            x_new);
        intersected = intersectSegment(
            x.r,
            x_new.r,
            particle_geometry,
            inter_point,
            inter_normal,
            is_inside);
        if (intersected || !is_inside)
        {
            if (!intersected)
            {
                // The following check is neccessary to test if the ray hits in
                // an extreme point (volume so small that it lies within the 
                // epsilon region). If that is the case, we assume the ray 
                // tunnels through the particle.
                bool intersect_outside = false;
                Vec3 t_ip, t_in;
                intersected = intersectRay(
                    ray_origin,
                    ray_direction,
                    particle_geometry,
                    t_ip,
                    t_in,
                    intersect_outside);
                if (!intersected || intersect_outside)
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
                        is_inside);
                    inter_point = ray_origin;
                    inter_normal = -inter_normal;
                    ray_direction = glm::normalize(x.w);
                }
            }
            else
            {
                ray_direction = glm::normalize(x.w);
                const double nx = indexOfRefraction(x.r, particle_material);
                const double ny = indexOfRefraction(inter_point, particle_material);
                const Vec3 delta_momentum = (nx - ny) * ray_direction;
                const Vec3 r = inter_point -
                    particle_center_of_mass[particle_index];
                output_force_per_ray[ri] +=
                    ray_intesity * delta_momentum;
                output_torque_per_ray[ri] +=
                    ray_intesity * glm::cross(r, delta_momentum);
            }
            intersected = true;
        }
        else
        {
            const double nx = indexOfRefraction(x.r, particle_material);
            const double ny = indexOfRefraction(x_new.r, particle_material);
            const Vec3 t_old_direction = glm::normalize(x.w);
            const Vec3 t_new_direction = glm::normalize(x_new.w);
            const Vec3 delta_momentum =
                nx * t_old_direction -
                ny * t_new_direction;
            const Vec3 r = x_new.r - particle_center_of_mass[particle_index];
            output_force_per_ray[ri] +=
                ray_intesity * delta_momentum;
            output_torque_per_ray[ri] +=
                ray_intesity * glm::cross(r, delta_momentum);
            x = x_new;
        }
    }
    ray_direction_buffer[ri] = ray_direction;
    intersection_point_buffer[ri] = inter_point;
    intersection_normal_buffer[ri] = inter_normal;
}

__device__ void rk4::rk4(
    IGenericMaterial::indexOfRefraction index_of_refraction, 
    MaterialLinearOneDirectionalData* material_data,
    const Arg& x,
    const double h, 
    Arg& x_new)
{
    using namespace ldplab;
    using namespace rtscuda;
    Arg k[4]{};
    const double beta[4] = { 1.0, 0.5, 0.5, 1.0 };
    const double c[4] = { 1.0, 2.0, 2.0, 1.0 };
    x_new = { {0,0,0}, {0,0,0} };
    for (size_t i = 0; i < 4; ++i)
    {
        Arg x_step = x;
        if (i > 0)
        {
            const double hb = h * beta[i];
            x_step.w += k[i - 1].w * hb;
            x_step.r += k[i - 1].r * hb;
        }
        //k[i].w = material_data->direction * material_data->gradient;
        k[i].w = material_data->direction_times_gradient;
        k[i].r = x_step.w / index_of_refraction(x_step.r, material_data);
        if (c[i] != 0.0)
        {
            x_new.w += k[i].w * c[i];
            x_new.r += k[i].r * c[i];
        }
    }
    x_new.w *= h / 6.0;
    x_new.r *= h / 6.0;
    x_new.w += x.w;
    x_new.r += x.r;
}

#endif