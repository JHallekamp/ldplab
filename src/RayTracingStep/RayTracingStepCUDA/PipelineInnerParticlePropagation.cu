#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineInnerParticlePropagation.hpp"

#include "Context.hpp"
#include "../../Utils/Log.hpp"

std::shared_ptr<ldplab::rtscuda::IPipelineInnerParticlePropagation> 
    ldplab::rtscuda::IPipelineInnerParticlePropagation::createInstance(
        const RayTracingStepCUDAInfo& info, 
        Context& context)
{
    if (info.solver_parameters->type() == IEikonalSolverParameter::Type::rk4)
    {
        return std::make_shared<PipelineInnerParticlePropagationRK4LinearIndexGradient>
            (context, *static_cast<RK4Parameter*>(info.solver_parameters.get()));
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCUDA context %i: Inner particle propagation "\
            "stage creation failed, unsupported solver type",
            context.uid);
        return nullptr;
    }
}

namespace rk4_linear_index_gradient_cuda
{
    __global__ void innerParticlePropagationKernel(
        double step_size,
        int32_t* ray_index_buffer,
        ldplab::Vec3* ray_origin_buffer,
        ldplab::Vec3* ray_direction_buffer,
        double* ray_intensity_buffer,
        ldplab::Vec3* intersection_point_buffer,
        ldplab::Vec3* intersection_normal_buffer,
        size_t num_rays_per_batch,
        ldplab::rtscuda::GenericParticleGeometryData* geometry_per_particle,
        ldplab::rtscuda::GenericParticleMaterialData* material_per_particle,
        ldplab::Vec3* particle_center_of_mass,
        ldplab::Vec3* output_force_per_ray,
        ldplab::Vec3* output_torque_per_ray,
        size_t num_particles);
    __device__ void rk4(
        const ldplab::rtscuda::ParticleLinearOneDirectionalMaterial::Data* material,
        const ldplab::rtscuda::PipelineInnerParticlePropagationRK4LinearIndexGradient::Arg& x,
        const double h,
        ldplab::rtscuda::PipelineInnerParticlePropagationRK4LinearIndexGradient::Arg& x_new);
    __device__ ldplab::rtscuda::pipelineInnerParticlePropagationStageKernel_t
        inner_particle_propagation_kernel_ptr = innerParticlePropagationKernel;
}

ldplab::rtscuda::PipelineInnerParticlePropagationRK4LinearIndexGradient::
    PipelineInnerParticlePropagationRK4LinearIndexGradient(
        Context& context, 
        RK4Parameter parameter)
    :
    m_context{ context },
    m_parameters{ parameter }
{ }

ldplab::rtscuda::pipelineInnerParticlePropagationStageKernel_t 
    ldplab::rtscuda::PipelineInnerParticlePropagationRK4LinearIndexGradient::
        getKernel()
{
    using namespace rk4_linear_index_gradient_cuda;
    // Copy the function pointer to the host
    pipelineInnerParticlePropagationStageKernel_t kernel = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel,
        inner_particle_propagation_kernel_ptr,
        sizeof(inner_particle_propagation_kernel_ptr))
        != cudaSuccess)
        return nullptr;
    return kernel;
}

void ldplab::rtscuda::PipelineInnerParticlePropagationRK4LinearIndexGradient::
    execute(size_t ray_buffer_index)
{
    using namespace rk4_linear_index_gradient_cuda;
    const size_t block_size = m_context.parameters.num_threads_per_block;
    const size_t grid_size = m_context.parameters.num_rays_per_batch / block_size;
    innerParticlePropagationKernel<<<grid_size, block_size>>>(
        m_parameters.step_size,
        m_context.resources.ray_buffer.index_buffers[ray_buffer_index].get(),
        m_context.resources.ray_buffer.origin_buffers[ray_buffer_index].get(),
        m_context.resources.ray_buffer.direction_buffers[ray_buffer_index].get(),
        m_context.resources.ray_buffer.intensity_buffers[ray_buffer_index].get(),
        m_context.resources.intersection_buffer.intersection_point_buffer.get(),
        m_context.resources.intersection_buffer.intersection_normal_buffer.get(),
        m_context.parameters.num_rays_per_batch,
        m_context.resources.particles.geometry_per_particle.get(),
        m_context.resources.particles.material_per_particle.get(),
        m_context.resources.particles.center_of_mass_per_particle.get(),
        m_context.resources.output_buffer.force_per_ray.get(),
        m_context.resources.output_buffer.torque_per_ray.get(),
        m_context.parameters.num_particles);
}

__global__ void rk4_linear_index_gradient_cuda::innerParticlePropagationKernel(
        double step_size,
        int32_t* ray_index_buffer, 
        ldplab::Vec3* ray_origin_buffer, 
        ldplab::Vec3* ray_direction_buffer, 
        double* ray_intensity_buffer, 
        ldplab::Vec3* intersection_point_buffer, 
        ldplab::Vec3* intersection_normal_buffer, 
        size_t num_rays_per_batch, 
        ldplab::rtscuda::GenericParticleGeometryData* geometry_per_particle, 
        ldplab::rtscuda::GenericParticleMaterialData* material_per_particle,
        ldplab::Vec3* particle_center_of_mass,
        ldplab::Vec3* output_force_per_ray, 
        ldplab::Vec3* output_torque_per_ray, 
        size_t num_particles)
{
    using namespace ldplab;
    using namespace rtscuda;
    unsigned int ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_batch)
        return;

    int32_t particle_index = ray_index_buffer[ri];
    if (particle_index < 0 ||
        particle_index >= num_particles)
        return;

    const Vec3 ray_origin = ray_origin_buffer[ri];
    Vec3 ray_direction = ray_direction_buffer[ri];
    const double ray_intesity = ray_intensity_buffer[ri];
    void* const particle_geometry = geometry_per_particle[particle_index].data;
    intersectRayParticleGeometryFunction_t intersectRayParticle = 
        geometry_per_particle[particle_index].intersect_ray_particle;
    ParticleLinearOneDirectionalMaterial::Data* material = 
        static_cast<ParticleLinearOneDirectionalMaterial::Data*>(
            material_per_particle[particle_index].data);
    bool intersected = false;
    bool is_inside = false;
    Vec3 inter_point;
    Vec3 inter_normal;
    PipelineInnerParticlePropagationRK4LinearIndexGradient::Arg x{
        ray_direction * material->indexOfRefraction(ray_origin),
        ray_origin 
    };
    PipelineInnerParticlePropagationRK4LinearIndexGradient::Arg x_new{ };
    while (!intersected)
    {
        rk4(material, x, step_size, x_new);
        intersected = GenericParticlFunctionWrapper::intersectSegment(
            intersectRayParticle,
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
                intersected = GenericParticlFunctionWrapper::intersectRay(
                    intersectRayParticle,
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
                    GenericParticlFunctionWrapper::intersectSegment(
                        intersectRayParticle,
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
                const double nx = material->indexOfRefraction(x.r);
                const double ny = material->indexOfRefraction(inter_point);
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
            const double nx = material->indexOfRefraction(x.r);
            const double ny = material->indexOfRefraction(x_new.r);
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

__device__ void rk4_linear_index_gradient_cuda::rk4(
    const ldplab::rtscuda::ParticleLinearOneDirectionalMaterial::Data* material, 
    const ldplab::rtscuda::PipelineInnerParticlePropagationRK4LinearIndexGradient::Arg& x,
    const double h,
    ldplab::rtscuda::PipelineInnerParticlePropagationRK4LinearIndexGradient::Arg& x_new)
{
    using namespace ldplab;
    using namespace rtscuda;
    PipelineInnerParticlePropagationRK4LinearIndexGradient::Arg k[4]{};
    const double beta[4] = { 1.0, 0.5, 0.5, 1.0 };
    const double c[4] = { 1.0, 2.0, 2.0, 1.0 };
    x_new = { {0,0,0}, {0,0,0} };
    for (size_t i = 0; i < 4; ++i)
    {
        PipelineInnerParticlePropagationRK4LinearIndexGradient::Arg x_step = x;
        if (i > 0)
        {
            const double hb = h * beta[i];
            x_step.w += k[i - 1].w * hb;
            x_step.r += k[i - 1].r * hb;
        }
        // eikonal(particle, x_step)
        k[i].w = material->direction * material->gradient;
        const double index_of_refraction =
            1.0 / material->indexOfRefraction(x_step.r);
        k[i].r = x_step.w * index_of_refraction;
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