#include "RayTracingStepFactory.hpp"

#include "RayTracingStepCPU/Data.hpp"
#include "RayTracingStepCPU/Pipeline.hpp"
#include "RayTracingStepCPU/RayTracingStepCPU.hpp"

#include "../Log.hpp"

std::shared_ptr<ldplab::rtscpu::RayTracingStepCPU> ldplab::RayTracingStepFactory::
    createRayTracingStepCPU(RayTracingStepCPUInfo& info)
{
    if (!checkTypeUniformity(info))
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Not supporting multiple types of"\
            "objects in the experimental setup.");
        return nullptr;
    }

    std::shared_ptr<rtscpu::Context> ctx{ new rtscpu::Context{} };
    ctx->thread_pool = info.thread_pool;
    ctx->particles = info.setup->particles;
    ctx->particle_transformations.resize(ctx->particles.size());
    initGeometry(info, ctx);
    ctx->light_sources = info.setup->light_sources;
    ctx->intensity_cutoff = info.intensity_cutoff;
    ctx->medium_index_of_reflecation = info.setup->refractive_index;
    ctx->number_rays_per_buffer = info.number_rays_per_buffer;
    ctx->number_rays_per_unit = info.number_rays_per_unit;
    ctx->maximum_depth = info.maximum_depth;
    ctx->number_parallel_pipelines = info.number_parallel_pipelines;
    
    if (info.setup->light_sources[0].direction->type() ==
        ILightDirection::Type::homogenous &&
        info.setup->light_sources[0].intensity_distribution->type() ==
        ILightDistribution::Type::homogenous &&
        info.setup->light_sources[0].polarisation->type() ==
        ILightPolarisation::Type::unpolarized &&
        info.setup->particles[0].bounding_volume->type() ==
        IBoundingVolume::Type::sphere &&
        info.setup->particles[0].geometry->type() ==
        IParticleGeometry::Type::sphere_capped_cylinder &&
        info.setup->particles[0].material->type() ==
        IParticleMaterial::Type::linear_one_directional)
    {
        std::unique_ptr<rtscpu::InitialStageBoundingSpheresHomogenousLight> initial
        { new rtscpu::InitialStageBoundingSpheresHomogenousLight{ ctx } };
        std::unique_ptr<rtscpu::RayBoundingSphereIntersectionTestStageBruteForce> rbvit
        { new rtscpu::RayBoundingSphereIntersectionTestStageBruteForce {ctx} };
        std::unique_ptr<rtscpu::RodeParticleIntersectionTest> rpit
        { new rtscpu::RodeParticleIntersectionTest{ctx} };
        std::unique_ptr<rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction> rpi
        { new rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction{ ctx } };
        std::unique_ptr<rtscpu::LinearIndexGradientRodeParticlePropagation> ipp
        { new rtscpu::LinearIndexGradientRodeParticlePropagation{
            ctx,
            info.initial_step_size,
            info.epsilon,
            info.safety_factor} };
        ctx->pipeline = std::unique_ptr<rtscpu::Pipeline>{ new rtscpu::Pipeline{
            std::move(initial),
            std::move(rbvit),
            std::move(rpit),
            std::move(rpi),
            std::move(ipp),
            ctx} };
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Experimental setup types not supported");
        return nullptr;
    }
    return std::shared_ptr<rtscpu::RayTracingStepCPU>{
        new rtscpu::RayTracingStepCPU{ ctx }};
}

void ldplab::RayTracingStepFactory::initGeometry(
    const RayTracingStepCPUInfo& info, std::shared_ptr<rtscpu::Context> context)
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
            Vec3 origin_cap{ 0.0 , 0.0, geometry->cylinder_length + h - sphere_radius };
            Vec3 origin_indentation{ 0.0 , 0.0,h - sphere_radius };
            context->rode_particle_geometry.push_back(rtscpu::RodeParticle{
                geometry->cylinder_radius,
                geometry->cylinder_length,
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

bool ldplab::RayTracingStepFactory::checkTypeUniformity(
    const RayTracingStepCPUInfo& info)
{
    for (size_t i = 0; i < info.setup->light_sources.size()-1; ++i)
    {
        if (info.setup->light_sources[i].direction->type() !=
            info.setup->light_sources[i + 1].direction->type())
            return false;
        if (info.setup->light_sources[i].intensity_distribution->type() !=
            info.setup->light_sources[i+1].intensity_distribution->type())
            return false;
        if (info.setup->light_sources[i].polarisation->type() !=
            info.setup->light_sources[i+1].polarisation->type())
            return false;
    }
    for (size_t i = 0; i < info.setup->particles.size()-1; ++i)
    {
        if (info.setup->particles[i].bounding_volume->type() !=
            info.setup->particles[i + 1].bounding_volume->type())
            return false;
        if (info.setup->particles[i].geometry->type() !=
            info.setup->particles[i + 1].geometry->type())
            return false;
        if (info.setup->particles[i].material->type() !=
            info.setup->particles[i + 1].material->type())
            return false;
    }
    return true;
}

