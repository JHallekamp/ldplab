#include "RayTracingStepFactory.hpp"

#include "RayTracingStepCPU/Context.hpp"
#include "RayTracingStepCPU/Data.hpp"
#include "RayTracingStepCPU/Pipeline.hpp"
#include "RayTracingStepCPU/RayTracingStep.hpp"

#include "RayTracingStepGPU/OpenGL/Context.hpp"
#include "RayTracingStepGPU/OpenGL/Data.hpp"
#include "RayTracingStepGPU/OpenGL/Pipeline.hpp"
#include "RayTracingStepGPU/OpenGL/RayTracingStep.hpp"

#include "EikonalSolver.hpp"
#include "../Log.hpp"

std::shared_ptr<ldplab::rtscpu::RayTracingStep> ldplab::RayTracingStepFactory::
    createRayTracingStepCPU(
        const ExperimentalSetup& setup,
        const RayTracingStepCPUInfo& info)
{
    LDPLAB_LOG_INFO("RTS factory: Begins creation of "\
        "ldplab::rtscpu::RayTracingStep");

    bool error = false;
    if (setup.light_sources.size() < 1)
    {
        LDPLAB_LOG_ERROR("RTS factory: Experimental setup contains no "\
            "light sources");
        error = true;
    }

    if (setup.particles.size() < 1)
    {
        LDPLAB_LOG_ERROR("RTS factory: Experimental setup contains no "\
            "particles");
        error = true;
    }
    
    if (!checkTypeUniformity(setup))
    {
        LDPLAB_LOG_ERROR("RTS factory: Not supporting multiple types of "\
            "objects in the experimental setup");
        error = true;
    }

    if (error)
    {
        LDPLAB_LOG_INFO("RTS factory: Creation of "\
            "ldplab::rtscpu::RayTracingStep failed");
        return nullptr;
    }
    
    if (setup.light_sources[0].direction->type() ==
        ILightDirection::Type::homogeneous &&
        setup.light_sources[0].intensity_distribution->type() ==
        ILightDistribution::Type::homogeneous &&
        setup.light_sources[0].polarisation->type() ==
        ILightPolarisation::Type::unpolarized &&
        setup.particles[0].bounding_volume->type() ==
        IBoundingVolume::Type::sphere &&
        setup.particles[0].geometry->type() ==
        IParticleGeometry::Type::rod_particle &&
        setup.particles[0].material->type() ==
        IParticleMaterial::Type::linear_one_directional &&
        info.solver_parameters->type() == IEikonalSolver::Type::rk45)
    {
        std::shared_ptr<rtscpu::Context> ctx{ new rtscpu::Context{
        setup.particles, setup.light_sources } };
        ctx->thread_pool = info.thread_pool;
        ctx->particle_transformations.resize(ctx->particles.size());
        ctx->parameters.intensity_cutoff = info.intensity_cutoff;
        ctx->parameters.medium_reflection_index = setup.medium_reflection_index;
        ctx->parameters.number_rays_per_buffer = info.number_rays_per_buffer;
        ctx->parameters.number_rays_per_unit = static_cast<size_t>(
            sqrt(info.light_source_ray_density_per_unit_area));
        ctx->parameters.maximum_branching_depth = info.maximum_branching_depth;
        ctx->parameters.number_parallel_pipelines = info.number_parallel_pipelines;

        ctx->bounding_volume_data =
            std::shared_ptr<rtscpu::IBoundingVolumeData>(
                new rtscpu::BoundingSphereData());
        ((rtscpu::BoundingSphereData*)ctx->bounding_volume_data.get())->
            sphere_data.resize(ctx->particles.size(), 
                BoundingVolumeSphere(Vec3{ 0, 0, 0 }, 0));
        ctx->particle_data =
            std::shared_ptr<rtscpu::IParticleData>(
                new rtscpu::RodParticleData());
        initRodParticleGeometryCPU(setup, ctx);

        std::unique_ptr<rtscpu::InitialStageBoundingSpheresHomogenousLight> initial
        { new rtscpu::InitialStageBoundingSpheresHomogenousLight{ ctx } };
        std::unique_ptr<rtscpu::RayBoundingSphereIntersectionTestStageBruteForce> rbvit
        { new rtscpu::RayBoundingSphereIntersectionTestStageBruteForce {ctx} };
        std::unique_ptr<rtscpu::RodParticleIntersectionTest> rpit
        { new rtscpu::RodParticleIntersectionTest{ctx} };
        std::unique_ptr<rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction> rpi
        { new rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction{ ctx } };
        std::unique_ptr<rtscpu::LinearIndexGradientRodParticlePropagation> ipp
        { new rtscpu::LinearIndexGradientRodParticlePropagation{
            ctx,
            *((RK45*) info.solver_parameters.get())} };
        ctx->pipeline = std::make_unique<rtscpu::Pipeline>(
            std::move(initial),
            std::move(rbvit),
            std::move(rpit),
            std::move(rpi),
            std::move(ipp),
            ctx);
        return std::shared_ptr<rtscpu::RayTracingStep>(
            new rtscpu::RayTracingStep{ ctx });
        LDPLAB_LOG_INFO("RTS factory: Creation of "\
            "ldplab::rtscpu::RayTracingStep completed");
    }

    LDPLAB_LOG_ERROR("RTS factory: The given combination of object "\
        "types in the experimental setup is not yet supported by "\
        "ldplab::rtscpu::RayTracingStep");
    LDPLAB_LOG_INFO("RTS factory: Creation of "\
        "ldplab::rtscpu::RayTracingStep failed");
    return nullptr;
}

std::shared_ptr<ldplab::rtscpu::RayTracingStep> 
    ldplab::RayTracingStepFactory::createRayTracingStepCPUDebug(
        const ExperimentalSetup& setup, 
        const RayTracingStepCPUInfo& info, 
        RayTracingStepCPUDebugInfo& debug)
{
    LDPLAB_LOG_INFO("RTS factory: Begins creation of "\
        "ldplab::rtscpu::RayTracingStep (debug variant)");

    bool error = false;
    if (setup.light_sources.size() < 1)
    {
        LDPLAB_LOG_ERROR("RTS factory: Experimental setup contains no "\
            "light sources");
        error = true;
    }

    if (setup.particles.size() < 1)
    {
        LDPLAB_LOG_ERROR("RTS factory: Experimental setup contains no "\
            "particles");
        error = true;
    }

    if (!checkTypeUniformity(setup))
    {
        LDPLAB_LOG_ERROR("RTS factory: Not supporting multiple types of "\
            "objects in the experimental setup");
        error = true;
    }

    if (error)
    {
        LDPLAB_LOG_INFO("RTS factory: Creation of "\
            "ldplab::rtscpu::RayTracingStep (debug variant) failed");
        return nullptr;
    }

    if (setup.light_sources[0].direction->type() ==
        ILightDirection::Type::homogeneous &&
        setup.light_sources[0].intensity_distribution->type() ==
        ILightDistribution::Type::homogeneous &&
        setup.light_sources[0].polarisation->type() ==
        ILightPolarisation::Type::unpolarized &&
        setup.particles[0].bounding_volume->type() ==
        IBoundingVolume::Type::sphere &&
        setup.particles[0].geometry->type() ==
        IParticleGeometry::Type::rod_particle &&
        setup.particles[0].material->type() ==
        IParticleMaterial::Type::linear_one_directional &&
        info.solver_parameters->type() == IEikonalSolver::Type::rk45)
    {
        std::shared_ptr<rtscpu::Context> ctx{ new rtscpu::Context{
        setup.particles, setup.light_sources } };
        ctx->thread_pool = info.thread_pool;
        ctx->particle_transformations.resize(ctx->particles.size());
        ctx->parameters.intensity_cutoff = info.intensity_cutoff;
        ctx->parameters.medium_reflection_index = setup.medium_reflection_index;
        ctx->parameters.number_rays_per_buffer = info.number_rays_per_buffer;
        ctx->parameters.number_rays_per_unit = static_cast<size_t>(
            sqrt(info.light_source_ray_density_per_unit_area));
        ctx->parameters.maximum_branching_depth = info.maximum_branching_depth;
        ctx->parameters.number_parallel_pipelines = info.number_parallel_pipelines;

        ctx->bounding_volume_data =
            std::shared_ptr<rtscpu::IBoundingVolumeData>(
                new rtscpu::BoundingSphereData());
        ((rtscpu::BoundingSphereData*)ctx->bounding_volume_data.get())->
            sphere_data.resize(ctx->particles.size(),
                BoundingVolumeSphere(Vec3{ 0, 0, 0 }, 0));
        ctx->particle_data =
            std::shared_ptr<rtscpu::IParticleData>(
                new rtscpu::RodParticleData());
        initRodParticleGeometryCPU(setup, ctx);

        debug.context = ctx;
        debug.initial_stage = 
            std::make_unique<rtscpu::InitialStageBoundingSpheresHomogenousLight>(ctx);
        debug.ray_bounding_volume_intersection_test =
            std::make_unique<rtscpu::RayBoundingSphereIntersectionTestStageBruteForce>(ctx);
        debug.ray_particle_intersection_test =
            std::make_unique<rtscpu::RodParticleIntersectionTest>(ctx);
        debug.ray_particle_interaction =
            std::make_unique<rtscpu::UnpolirzedLight1DLinearIndexGradientInteraction>(ctx);
        debug.inner_particle_propagation =
            std::make_unique<rtscpu::LinearIndexGradientRodParticlePropagation>(ctx, *((RK45*)info.solver_parameters.get()));

        return std::shared_ptr<rtscpu::RayTracingStep>(
            new rtscpu::RayTracingStep{ ctx });
        LDPLAB_LOG_INFO("RTS factory: Creation of "\
            "ldplab::rtscpu::RayTracingStep (debug variant) completed");
    }

    LDPLAB_LOG_ERROR("RTS factory: The given combination of object "\
        "types in the experimental setup is not yet supported by "\
        "ldplab::rtscpu::RayTracingStep (debug variant)");
    LDPLAB_LOG_INFO("RTS factory: Creation of "\
        "ldplab::rtscpu::RayTracingStep (debug variant) failed");
    return nullptr;
}

std::shared_ptr<ldplab::rtsgpu_ogl::RayTracingStep>
    ldplab::RayTracingStepFactory::createRayTracingStepGPUOpenGL(
        const ExperimentalSetup& setup, 
        const RayTracingStepGPUOpenGLInfo& info)
{
    LDPLAB_LOG_INFO("RTS factory: Begins creation of "\
        "ldplab::rtsgpu_ogl::RayTracingStep");

    bool error = false;
    if (setup.light_sources.size() < 1)
    {
        LDPLAB_LOG_ERROR("RTS factory: Experimental setup contains no "\
            "light sources");
        error = true;
    }

    if (setup.particles.size() < 1)
    {
        LDPLAB_LOG_ERROR("RTS factory: Experimental setup contains no "\
            "particles");
        error = true;
    }

    if (!checkTypeUniformity(setup))
    {
        LDPLAB_LOG_ERROR("RTS factory: Not supporting multiple types of "\
            "objects in the experimental setup");
        error = true;
    }

    if (error)
    {
        LDPLAB_LOG_INFO("RTS factory: Creation of "\
            "ldplab::rtsgpu_ogl::RayTracingStep failed");
        return nullptr;
    }

    if (setup.light_sources[0].direction->type() ==
        ILightDirection::Type::homogeneous &&
        setup.light_sources[0].intensity_distribution->type() ==
        ILightDistribution::Type::homogeneous &&
        setup.light_sources[0].polarisation->type() ==
        ILightPolarisation::Type::unpolarized &&
        setup.particles[0].bounding_volume->type() ==
        IBoundingVolume::Type::sphere &&
        setup.particles[0].geometry->type() ==
        IParticleGeometry::Type::rod_particle &&
        setup.particles[0].material->type() ==
        IParticleMaterial::Type::linear_one_directional &&
        info.solver_parameters->type() == IEikonalSolver::Type::rk45)
    {
        std::shared_ptr<rtsgpu_ogl::Context> ctx{ new rtsgpu_ogl::Context{
        setup.particles, setup.light_sources } };
        ctx->thread_pool = info.thread_pool;
        ctx->particle_transformations.resize(ctx->particles.size());
        ctx->parameters.intensity_cutoff = info.intensity_cutoff;
        ctx->parameters.medium_reflection_index = setup.medium_reflection_index;
        ctx->parameters.number_rays_per_buffer = info.number_rays_per_buffer;
        ctx->parameters.number_rays_per_unit = static_cast<size_t>(
            sqrt(info.light_source_ray_density_per_unit_area));
        ctx->parameters.maximum_branching_depth = info.maximum_branching_depth;
        ctx->parameters.number_parallel_pipelines = info.number_parallel_pipelines;

        ctx->bounding_volume_data =
            std::shared_ptr<rtsgpu_ogl::IBoundingVolumeData>(
                new rtsgpu_ogl::BoundingSphereData());
        ((rtsgpu_ogl::BoundingSphereData*)ctx->bounding_volume_data.get())->
            sphere_data.resize(ctx->particles.size(),
                BoundingVolumeSphere(Vec3{ 0, 0, 0 }, 0));
        ctx->particle_data =
            std::shared_ptr<rtsgpu_ogl::IParticleData>(
                new rtsgpu_ogl::RodParticleData());
        initRodParticleGeometryGPUOpenGL(setup, ctx);

        std::unique_ptr<rtsgpu_ogl::InitialStageBoundingSpheresHomogenousLight> initial
        { new rtsgpu_ogl::InitialStageBoundingSpheresHomogenousLight{ ctx } };
        std::unique_ptr<rtsgpu_ogl::RayBoundingSphereIntersectionTestStageBruteForce> rbvit
        { new rtsgpu_ogl::RayBoundingSphereIntersectionTestStageBruteForce {ctx} };
        std::unique_ptr<rtsgpu_ogl::RodParticleIntersectionTest> rpit
        { new rtsgpu_ogl::RodParticleIntersectionTest{ctx} };
        std::unique_ptr<rtsgpu_ogl::UnpolirzedLight1DLinearIndexGradientInteraction> rpi
        { new rtsgpu_ogl::UnpolirzedLight1DLinearIndexGradientInteraction{ ctx } };
        std::unique_ptr<rtsgpu_ogl::LinearIndexGradientRodParticlePropagation> ipp
        { new rtsgpu_ogl::LinearIndexGradientRodParticlePropagation{
            ctx,
            *((RK45*)info.solver_parameters.get())} };
        ctx->pipeline = std::make_unique<rtsgpu_ogl::Pipeline>(
            std::move(initial),
            std::move(rbvit),
            std::move(rpit),
            std::move(rpi),
            std::move(ipp),
            ctx);
        return std::shared_ptr<rtsgpu_ogl::RayTracingStep>(
            new rtsgpu_ogl::RayTracingStep{ ctx });
        LDPLAB_LOG_INFO("RTS factory: Creation of "\
            "ldplab::rtsgpu_ogl::RayTracingStep completed");
    }

    LDPLAB_LOG_ERROR("RTS factory: The given combination of object "\
        "types in the experimental setup is not yet supported by "\
        "ldplab::rtsgpu_ogl::RayTracingStep");
    LDPLAB_LOG_INFO("RTS factory: Creation of "\
        "ldplab::rtsgpu_ogl::RayTracingStep failed");
    return nullptr;
}

void ldplab::RayTracingStepFactory::initRodParticleGeometryCPU(
    const ExperimentalSetup& setup, 
    std::shared_ptr<rtscpu::Context> context)
{
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        const Particle& particle = setup.particles[i];
        if (particle.geometry->type() ==
            IParticleGeometry::Type::rod_particle)
        {
            RodParticleGeometry* geometry =
                (RodParticleGeometry*)particle.geometry.get();
            double h = geometry->kappa * geometry->cylinder_radius;
            double sphere_radius =
                (h + geometry->cylinder_radius * geometry->cylinder_radius / h) / 2.0;
            Vec3 origin_cap{ 0.0 , 0.0, geometry->cylinder_length + h - sphere_radius };
            Vec3 origin_indentation{ 0.0 , 0.0,h - sphere_radius };
            ((rtscpu::RodParticleData*)context->particle_data.get())->
                particle_data.push_back(rtscpu::RodParticle{
                    geometry->cylinder_radius,
                    geometry->cylinder_length,
                    sphere_radius,
                    origin_cap,
                    origin_indentation });
        }
    }
}

void ldplab::RayTracingStepFactory::initRodParticleGeometryGPUOpenGL(
    const ExperimentalSetup& setup, 
    std::shared_ptr<rtsgpu_ogl::Context> context)
{
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        const Particle& particle = setup.particles[i];
        if (particle.geometry->type() ==
            IParticleGeometry::Type::rod_particle)
        {
            RodParticleGeometry* geometry =
                (RodParticleGeometry*)particle.geometry.get();
            double h = geometry->kappa * geometry->cylinder_radius;
            double sphere_radius =
                (h + geometry->cylinder_radius * geometry->cylinder_radius / h) / 2.0;
            Vec3 origin_cap{ 0.0 , 0.0, geometry->cylinder_length + h - sphere_radius };
            Vec3 origin_indentation{ 0.0 , 0.0,h - sphere_radius };
            ((rtsgpu_ogl::RodParticleData*)context->particle_data.get())->
                particle_data.push_back(rtsgpu_ogl::RodParticle{
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
    LDPLAB_LOG_INFO("RTS factory: Light direction type is %s",
        setup.light_sources[0].direction->typeString());
    ILightDistribution::Type distribution_type =
        setup.light_sources[0].intensity_distribution->type();
    LDPLAB_LOG_INFO("RTS factory: Light intensity distribution type is %s",
        setup.light_sources[0].intensity_distribution->typeString());
    ILightPolarisation::Type polarisation_type =
        setup.light_sources[0].polarisation->type();
    LDPLAB_LOG_INFO("RTS factory: Light polarisation type is %s",
        setup.light_sources[0].polarisation->typeString());

    // Check for inhomogenous types
    for (size_t i = 1; i < setup.light_sources.size(); ++i)
    {
        if (setup.light_sources[i].direction->type() != 
            direction_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTS factory: Found inconsistent light "\
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
            LDPLAB_LOG_ERROR("RTS factory: Found inconsistent light "\
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
            LDPLAB_LOG_ERROR("RTS factory: Found inconsistent light "\
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
    LDPLAB_LOG_INFO("RTS factory: Particle bounding volume type is %s",
        setup.particles[0].bounding_volume->typeString());
    IParticleGeometry::Type geometry_type =
        setup.particles[0].geometry->type();
    LDPLAB_LOG_INFO("RTS factory: Particle geometry type is %s",
        setup.particles[0].geometry->typeString());
    IParticleMaterial::Type material_type =
        setup.particles[0].material->type();
    LDPLAB_LOG_INFO("RTS factory: Particle material type is %s",
        setup.particles[0].material->typeString());

    // Check for inhomogenous types
    for (size_t i = 1; i < setup.particles.size(); ++i)
    {
        if (setup.particles[i].bounding_volume->type() !=
            bounding_volume_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTS factory: Found inconsistent particle "\
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
            LDPLAB_LOG_ERROR("RTS factory: Found inconsistent particle "\
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
            LDPLAB_LOG_ERROR("RTS factory: Found inconsistent particle "\
                "material type in particle %i, type was %s but "\
                "expected %s",
                setup.particles[i].uid,
                setup.particles[i].material->typeString(),
                setup.particles[0].material->typeString());
        }
    }

    return only_homogenous_types;
}

