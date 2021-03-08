#include "RayTracingStepFactory.hpp"

#include "RayTracingStepCPU/Factory.hpp"

#include "RayTracingStepGPU/OpenGL/Context.hpp"
#include "RayTracingStepGPU/OpenGL/Data.hpp"
#include "RayTracingStepGPU/OpenGL/Pipeline.hpp"
#include "RayTracingStepGPU/OpenGL/RayTracingStep.hpp"

#include "EikonalSolver.hpp"
#include "../Log.hpp"

#include "../Utils/Profiler.hpp"

std::shared_ptr<ldplab::IRayTracingStep> ldplab::RayTracingStepFactory::
    createRayTracingStepCPU(
        const ExperimentalSetup& setup,
        const RayTracingStepCPUInfo& info)
{
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtscpu);
    return rtscpu::Factory::createRTS(setup, info);
}

std::shared_ptr<ldplab::IRayTracingStep> 
    ldplab::RayTracingStepFactory::createRayTracingStepCPUDebug(
        const ExperimentalSetup& setup, 
        const RayTracingStepCPUInfo& info, 
        RayTracingStepCPUDebugInfo& debug)
{
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtscpu);
    return rtscpu::Factory::createRTSDebug(setup, info, debug);
}

std::shared_ptr<ldplab::IRayTracingStep>
    ldplab::RayTracingStepFactory::createRayTracingStepGPUOpenGL(
        const ExperimentalSetup& setup, 
        const RayTracingStepGPUOpenGLInfo& info)
{
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtsgpu_ogl);
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
        ctx->particle_transformation_data.p2w_data.resize(
            ctx->particles.size());
        ctx->particle_transformation_data.p2w_scale_rotation.resize(
            ctx->particles.size());
        ctx->particle_transformation_data.p2w_translation.resize(
            ctx->particles.size());
        ctx->particle_transformation_data.w2p_data.resize(
            ctx->particles.size());
        ctx->particle_transformation_data.w2p_rotation_scale.resize(
            ctx->particles.size());
        ctx->particle_transformation_data.w2p_translation.resize(
            ctx->particles.size());
        ctx->parameters.intensity_cutoff = info.intensity_cutoff;
        ctx->parameters.medium_reflection_index = setup.medium_reflection_index;
        ctx->parameters.number_rays_per_buffer = info.number_rays_per_buffer;
        ctx->parameters.number_rays_per_unit = static_cast<size_t>(
            sqrt(info.light_source_ray_density_per_unit_area));
        ctx->parameters.maximum_branching_depth = info.maximum_branching_depth;
        ctx->parameters.number_parallel_pipelines = info.number_parallel_pipelines;
        ctx->flags.emit_warning_on_maximum_branching_depth_discardment =
            info.emit_warning_on_maximum_branching_depth_discardment;

        ctx->bounding_volume_data =
            std::shared_ptr<rtsgpu_ogl::IBoundingVolumeData>(
                new rtsgpu_ogl::BoundingSphereData());
        ((rtsgpu_ogl::BoundingSphereData*)ctx->bounding_volume_data.get())->
            sphere_properties_data.resize(ctx->particles.size(),
                rtsgpu_ogl::BoundingSphereData::BoundingSphereProperties{ 
                    Vec3{ 0, 0, 0 }, 0 });
        ctx->particle_data =
            std::shared_ptr<rtsgpu_ogl::IParticleData>(
                new rtsgpu_ogl::RodParticleData(ctx));
        ctx->particle_material_data =
            std::shared_ptr<rtsgpu_ogl::IParticleMaterialData>(
                new rtsgpu_ogl::ParticleMaterialLinearOneDirectionalData(ctx));
        if (!initRodParticleGeometryGPUOpenGL(setup, ctx))
        {
            LDPLAB_LOG_ERROR("RTS factory: Could not initialize particle data");
            return nullptr;
        }

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
        std::shared_ptr<rtsgpu_ogl::RayTracingStep> rts(
            new rtsgpu_ogl::RayTracingStep{ ctx });

        // Init OpenGL
        ctx->ogl = std::shared_ptr<rtsgpu_ogl::OpenGLContext>(
            new rtsgpu_ogl::OpenGLContext(ctx));
        if (!ctx->ogl->init())
        {
            LDPLAB_LOG_ERROR("RTS factory: Could not initialize "\
                "ldplab::rtsgpu_ogl::OpenGLContext");
            return nullptr;
        }
        if (!rts->initGPU(info))
        {
            LDPLAB_LOG_ERROR("RTS factory: Could not fully initialize "\
                "ray tracing stage gpu resources");
            return nullptr;
        }

        LDPLAB_LOG_INFO("RTS factory: Creation of "\
            "ldplab::rtsgpu_ogl::RayTracingStep completed");
        return rts;
    }

    LDPLAB_LOG_ERROR("RTS factory: The given combination of object "\
        "types in the experimental setup is not yet supported by "\
        "ldplab::rtsgpu_ogl::RayTracingStep");
    LDPLAB_LOG_INFO("RTS factory: Creation of "\
        "ldplab::rtsgpu_ogl::RayTracingStep failed");
    return nullptr;
}

bool ldplab::RayTracingStepFactory::initRodParticleGeometryGPUOpenGL(
    const ExperimentalSetup& setup, 
    std::shared_ptr<rtsgpu_ogl::Context> context)
{
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        const Particle& particle = setup.particles[i];
        
        // Geometry
        if (particle.geometry->type() ==
            IParticleGeometry::Type::rod_particle)
        {
            RodParticleGeometry* geometry =
                (RodParticleGeometry*)particle.geometry.get();
            double h;
            double sphere_radius;
            if (geometry->kappa <= 0.001)
            {
                h = 0;
                sphere_radius = 0;
            }
            else
            {
                h = geometry->kappa * geometry->cylinder_radius;
                sphere_radius =
                    (h + geometry->cylinder_radius *
                        geometry->cylinder_radius / h) / 2.0;
            }
            Vec3 origin_cap{ 0.0 , 0.0, geometry->cylinder_length + h - sphere_radius };
            Vec3 origin_indentation{ 0.0 , 0.0,h - sphere_radius };
            ((rtsgpu_ogl::RodParticleData*)context->particle_data.get())->
                rod_particles_data.push_back(
                    rtsgpu_ogl::RodParticleData::RodParticleProperties{
                        origin_cap.z,
                        origin_indentation.z,
                        geometry->cylinder_radius,
                        geometry->cylinder_length,
                        sphere_radius });
        }
        else
        {
            LDPLAB_LOG_ERROR("RTS factory: Encountered unsupported particle "\
                "geometry type");
            return false;
        }

        // Material
        if (particle.material->type() ==
            IParticleMaterial::Type::linear_one_directional)
        {
            const ParticleMaterialLinearOneDirectional* pm =
                (ParticleMaterialLinearOneDirectional*)particle.material.get();
            ((rtsgpu_ogl::ParticleMaterialLinearOneDirectionalData*)
                context->particle_material_data.get())->material_data.push_back(
                    rtsgpu_ogl::ParticleMaterialLinearOneDirectionalData::LinearOneDirectionalMaterialProperties
                        {pm->direction_times_gradient, pm->index_of_refraction_minus_partial_dot});
        }
        else
        {
            LDPLAB_LOG_ERROR("RTS factory: Encountered unsupported particle "\
                "material type");
            return false;
        }
    }
    return true;
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

