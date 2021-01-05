#include "RayTracingStepFactory.hpp"

#include "RayTracingStepCPU/Data.hpp"
#include "RayTracingStepCPU/Pipeline.hpp"
#include "RayTracingStepCPU/RayTracingStepCPU.hpp"

#include "../Log.hpp"

std::shared_ptr<ldplab::rtscpu::RayTracingStepCPU> ldplab::RayTracingStepFactory::
    createRayTracingStepCPU(
        const ExperimentalSetup& setup,
        const RayTracingStepCPUInfo& info)
{
    if (setup.light_sources.size() < 1)
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Experimental setup contains no "\
            "light sources");
        return nullptr;
    }

    if (setup.particles.size() < 1)
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Experimental setup contains no "\
            "particles");
        return nullptr;
    }
    
    if (!checkTypeUniformity(setup))
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Not supporting multiple types of "\
            "objects in the experimental setup");
        return nullptr;
    }

    std::shared_ptr<rtscpu::Context> ctx{ new rtscpu::Context{
        setup.particles, setup.light_sources } };
    ctx->thread_pool = info.thread_pool;
    ctx->particle_transformations.resize(ctx->particles.size());
    ctx->transformed_bounding_spheres.resize(ctx->particles.size(),
        BoundingVolumeSphere(Vec3(0, 0, 0), 0));
    ctx->intensity_cutoff = info.intensity_cutoff;
    ctx->medium_index_of_reflecation = setup.refractive_index;
    ctx->number_rays_per_buffer = info.number_rays_per_buffer;
    ctx->number_rays_per_unit = 
        sqrt(info.light_source_ray_density_per_unit_area);
    ctx->maximum_depth = info.maximum_depth;
    ctx->number_parallel_pipelines = info.number_parallel_pipelines;
    
    if (setup.light_sources[0].direction->type() ==
        ILightDirection::Type::homogenous &&
        setup.light_sources[0].intensity_distribution->type() ==
        ILightDistribution::Type::homogenous &&
        setup.light_sources[0].polarisation->type() ==
        ILightPolarisation::Type::unpolarized &&
        setup.particles[0].bounding_volume->type() ==
        IBoundingVolume::Type::sphere &&
        setup.particles[0].geometry->type() ==
        IParticleGeometry::Type::sphere_capped_cylinder &&
        setup.particles[0].material->type() ==
        IParticleMaterial::Type::linear_one_directional)
    {
        initGeometry(setup, ctx);
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
        return std::shared_ptr<rtscpu::RayTracingStepCPU>{
            new rtscpu::RayTracingStepCPU{ ctx }};
    }

    LDPLAB_LOG_ERROR("RTSCPU factory: The given combination of object "\
        "types in the experimental setup is not yet supported");
    return nullptr;
}

void ldplab::RayTracingStepFactory::initGeometry(
    const ExperimentalSetup& setup, std::shared_ptr<rtscpu::Context> context)
{
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        const Particle& particle = setup.particles[i];
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
    }
}

bool ldplab::RayTracingStepFactory::checkTypeUniformity(
    const ExperimentalSetup& setup)
{
    bool only_homogenous_types = true;

    // Set light source types
    ILightDirection::Type direction_type =
        setup.light_sources[0].direction->type();
    LDPLAB_LOG_INFO("RTSCPU factory: Light direction type is %s",
        setup.light_sources[0].direction->typeString());
    ILightDistribution::Type distribution_type =
        setup.light_sources[0].intensity_distribution->type();
    LDPLAB_LOG_INFO("RTSCPU factory: Light intensity distribution type is %s",
        setup.light_sources[0].intensity_distribution->typeString());
    ILightPolarisation::Type polarisation_type =
        setup.light_sources[0].polarisation->type();
    LDPLAB_LOG_INFO("RTSCPU factory: Light polarisation type is %s",
        setup.light_sources[0].polarisation->typeString());

    // Check for inhomogenous types
    for (size_t i = 1; i < setup.light_sources.size(); ++i)
    {
        if (setup.light_sources[i].direction->type() != 
            direction_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent light "\
                "direction type in light source %i, type was %s but "\
                "expected %s",
                setup.light_sources[i].uid,
                setup.light_sources[i].direction->typeString(),
                setup.light_sources[0].direction->typeString());
        }
        if (setup.light_sources[i].intensity_distribution->type() !=
            distribution_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent light "\
                "intensity distribution type in light source %i, type was "\
                "%s but expected %s",
                setup.light_sources[i].uid,
                setup.light_sources[i].intensity_distribution->typeString(),
                setup.light_sources[0].intensity_distribution->typeString());
        }
        if (setup.light_sources[i].polarisation->type() !=
            polarisation_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent light "\
                "polarisation type in light source %i, type was %s but "\
                "expected %s",
                setup.light_sources[i].uid,
                setup.light_sources[i].polarisation->typeString(),
                setup.light_sources[0].polarisation->typeString());
        }
    }

    // Set particle types
    IBoundingVolume::Type bounding_volume_type =
        setup.particles[0].bounding_volume->type();
    LDPLAB_LOG_INFO("RTSCPU factory: Particle bounding volume type is %s",
        setup.particles[0].bounding_volume->typeString());
    IParticleGeometry::Type geometry_type =
        setup.particles[0].geometry->type();
    LDPLAB_LOG_INFO("RTSCPU factory: Particle geometry type is %s",
        setup.particles[0].geometry->typeString());
    IParticleMaterial::Type material_type =
        setup.particles[0].material->type();
    LDPLAB_LOG_INFO("RTSCPU factory: Particle material type is %s",
        setup.particles[0].material->typeString());

    // Check for inhomogenous types
    for (size_t i = 1; i < setup.particles.size(); ++i)
    {
        if (setup.particles[i].bounding_volume->type() !=
            bounding_volume_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent particle "\
                "bounding volume type in particle %i, type was %s but "\
                "expected %s",
                setup.particles[i].uid,
                setup.particles[i].bounding_volume->typeString(),
                setup.particles[0].bounding_volume->typeString());
        }
        if (setup.particles[i].geometry->type() !=
            geometry_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent particle "\
                "geometry type in particle %i, type was %s but "\
                "expected %s",
                setup.particles[i].uid,
                setup.particles[i].geometry->typeString(),
                setup.particles[0].geometry->typeString());
        }
        if (setup.particles[i].material->type() !=
            material_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent particle "\
                "material type in particle %i, type was %s but "\
                "expected %s",
                setup.particles[i].uid,
                setup.particles[i].material->typeString(),
                setup.particles[0].material->typeString());
        }
    }

    return only_homogenous_types;
}

