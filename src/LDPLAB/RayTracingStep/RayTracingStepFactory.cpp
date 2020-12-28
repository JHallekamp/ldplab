#include "RayTracingStepFactory.hpp"

#include "RayTracingStepCPU/Data.hpp"

std::shared_ptr<ldplab::rtscpu::RayTracingStepCPU> ldplab::RayTracingStepFactory::
    createRayTracingStepCPU(RayTracingStepCPUInfo& info)
{

    std::shared_ptr<rtscpu::Context> ctx{ new rtscpu::Context{} };
    ctx->particles = info.setup->particles;
    ctx->particle_transformations.resize(ctx->particles.size());
    initGeometry(info, ctx);
    ctx->light_sources = info.setup->light_sources;
    ctx->intensity_cutoff = info.intensity_cutoff;
    ctx->number_rays_per_buffer = info.number_rays_per_buffer;
    ctx->number_rays_per_unit = info.number_rays_per_unit;
    ctx->maximum_depth = info.maximum_depth;
    ctx->number_parallel_pipelines = info.number_parallel_pipelines;

    std::shared_ptr<rtscpu::RayTracingStepCPU> rts_cpu{ 
        new rtscpu::RayTracingStepCPU{ctx}};

    return rts_cpu;
}

void ldplab::RayTracingStepFactory::initGeometry(
    RayTracingStepCPUInfo& info, std::shared_ptr<rtscpu::Context> context)
{
    for (size_t i = 0; i < info.setup->particles.size(); ++i)
    {
        Particle& particle = info.setup->particles[i];

        if (particle.geometry->type() ==
            IParticleGeometry::Type::sphere_capped_cylinder)
        {
            RodeParticleGeometry* geometry =
                (RodeParticleGeometry*)particle.geometry.get();
            double h = geometry->kappa * geometry->cylinder_radius;
            double sphere_radius =
                (h + geometry->cylinder_radius * geometry->cylinder_radius / h) / 2.0;
            Vec3 origin_cap{ 0,0, geometry->cylinder_hight + h - sphere_radius };
            Vec3 origin_indentation{ 0,0,h - sphere_radius };
            context->rode_particle_geometry.push_back(rtscpu::RodeParticle{
                geometry->cylinder_radius,
                geometry->cylinder_hight,
                sphere_radius,
                origin_cap,
                origin_indentation });
        }
        else
        {
            // TODO: ERROR
        }
    }
}

