#ifndef LDPLAB_BUILD_OPTION_DISABLE_RTSGPU_OGL

#include "Factory.hpp"

#include "../../../Constants.hpp"
#include "../../../Utils/Log.hpp"

std::shared_ptr<ldplab::rtsgpu_ogl::RayTracingStep> 
    ldplab::rtsgpu_ogl::Factory::createRTS(
        const ExperimentalSetup& setup, 
        const RayTracingStepGPUOpenGLInfo& info)
{
    // Create the factory instance
    Factory factory;

    // Validate setup
    if (!factory.validateSetup(setup, info))
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create "\
            "rtsgpu_ogl::RayTracingStep instance, invalid experimental setup");
        return nullptr;
    }

    // Create context
    Context& context;
    if (!factory.createContext(setup, info, context))
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create "\
            "rtsgpu_ogl::RayTracingStep instance, context creation failed");
        return nullptr;
    }

    // Create pipeline
    std::unique_ptr<IInitialStage> initial;
    std::unique_ptr<IInnerParticlePropagationStage> ipp;
    std::unique_ptr<IRayBoundingVolumeIntersectionTestStage> rbvit;
    std::unique_ptr<IRayParticleInteractionStage> rpi;
    std::unique_ptr<IRayParticleIntersectionTestStage> rpit;
    if (!factory.createPipelineStages(
        setup,
        info,
        context,
        initial,
        ipp,
        rbvit,
        rpi,
        rpit))
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create "\
            "rtsgpu_ogl::RayTracingStep instance, pipeline stage creation failed");
        return nullptr;
    }
    context->pipeline = std::make_unique<rtsgpu_ogl::Pipeline>(
        std::move(initial),
        std::move(rbvit),
        std::move(rpit),
        std::move(rpi),
        std::move(ipp),
        context);

    // Create ray tracing step
    std::shared_ptr<RayTracingStep> rts(new RayTracingStep(context));

    // Init OpenGL
    context->ogl = std::make_shared<OpenGLContext>(context);
    if (!context->ogl->init() || !rts->initGPU(info))
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create "\
            "rtsgpu_ogl::RayTracingStep instance, gpu context creation failed");
        return nullptr;
    }

    return rts;
}

bool ldplab::rtsgpu_ogl::Factory::validateSetup(
    const ExperimentalSetup& setup,
    const RayTracingStepGPUOpenGLInfo& info)
{
    bool error = false;   
    // Check if setup contains light and particles
    if (setup.light_sources.size() == 0)
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Experimental setup %i contains no "\
            "light sources", setup.uid);
        error = true;
    }
    if (setup.particles.size() == 0)
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Experimental setup %i contains no "\
            "particles", setup.uid);
        error = true;
    }
    if (error)
        return false;
    // Sets the types
    setTypes(setup, info);
    // Check the setup types
    if (!validateTypeHomogeneity(setup))
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Experimental setup %i contains "\
            "inhomogeneous types, which are not supported", setup.uid);
        return false;
    }
    // Setup is valid
    return true;
}

void ldplab::rtsgpu_ogl::Factory::setTypes(
    const ExperimentalSetup& setup,
    const RayTracingStepGPUOpenGLInfo& info)
{
    m_light_direction_type = setup.light_sources[0].direction->type();
    m_light_distribution_type = setup.light_sources[0].intensity_distribution->type();
    m_light_polarization_type = setup.light_sources[0].polarisation->type();
    m_bounding_volume_type = setup.particles[0].bounding_volume->type();
    m_particle_geometry_type = setup.particles[0].geometry->type();
    m_particle_material_type = setup.particles[0].material->type();
    m_solver_type = info.solver_parameters->type();
    LDPLAB_LOG_INFO("RTSGPU factory (OpenGL): Light direction type is %s",
        setup.light_sources[0].direction->typeString());
    LDPLAB_LOG_INFO("RTSGPU factory (OpenGL): Light intensity distribution type is %s",
        setup.light_sources[0].intensity_distribution->typeString());
    LDPLAB_LOG_INFO("RTSGPU factory (OpenGL): Light polarisation type is %s",
        setup.light_sources[0].polarisation->typeString());
    LDPLAB_LOG_INFO("RTSGPU factory (OpenGL): Particle bounding volume type is %s",
        setup.particles[0].bounding_volume->typeString());
    LDPLAB_LOG_INFO("RTSGPU factory (OpenGL): Particle geometry type is %s",
        setup.particles[0].geometry->typeString());
    LDPLAB_LOG_INFO("RTSGPU factory (OpenGL): Particle material type is %s",
        setup.particles[0].material->typeString());
    LDPLAB_LOG_INFO("RTSGPU factory (OpenGL): Eikonal solver type is %s",
        info.solver_parameters->typeString());
}

bool ldplab::rtsgpu_ogl::Factory::validateTypeHomogeneity(
    const ExperimentalSetup& setup)
{
    bool only_homogenous_types = true;
    // Check for inhomogenous types
    for (size_t i = 1; i < setup.light_sources.size(); ++i)
    {
        if (setup.light_sources[i].direction->type() !=
            m_light_direction_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Found inconsistent light "\
                "direction type in light source %i, type was %s but "\
                "expected %s",
                setup.light_sources[i].uid,
                setup.light_sources[i].direction->typeString(),
                setup.light_sources[0].direction->typeString());
        }
        if (setup.light_sources[i].intensity_distribution->type() !=
            m_light_distribution_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Found inconsistent light "\
                "intensity distribution type in light source %i, type was "\
                "%s but expected %s",
                setup.light_sources[i].uid,
                setup.light_sources[i].intensity_distribution->typeString(),
                setup.light_sources[0].intensity_distribution->typeString());
        }
        if (setup.light_sources[i].polarisation->type() !=
            m_light_polarization_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Found inconsistent light "\
                "polarisation type in light source %i, type was %s but "\
                "expected %s",
                setup.light_sources[i].uid,
                setup.light_sources[i].polarisation->typeString(),
                setup.light_sources[0].polarisation->typeString());
        }
    }
    // Check for inhomogenous types
    for (size_t i = 1; i < setup.particles.size(); ++i)
    {
        if (setup.particles[i].bounding_volume->type() !=
            m_bounding_volume_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Found inconsistent particle "\
                "bounding volume type in particle %i, type was %s but "\
                "expected %s",
                setup.particles[i].uid,
                setup.particles[i].bounding_volume->typeString(),
                setup.particles[0].bounding_volume->typeString());
        }
        if (setup.particles[i].geometry->type() !=
            m_particle_geometry_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Found inconsistent particle "\
                "geometry type in particle %i, type was %s but "\
                "expected %s",
                setup.particles[i].uid,
                setup.particles[i].geometry->typeString(),
                setup.particles[0].geometry->typeString());
        }
        if (setup.particles[i].material->type() !=
            m_particle_material_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Found inconsistent particle "\
                "material type in particle %i, type was %s but "\
                "expected %s",
                setup.particles[i].uid,
                setup.particles[i].material->typeString(),
                setup.particles[0].material->typeString());
        }
    }
    return only_homogenous_types;
}

bool ldplab::rtsgpu_ogl::Factory::createContext(
    const ExperimentalSetup& setup,
    const RayTracingStepGPUOpenGLInfo& info,
    std::shared_ptr<Context>& context)
{
    context = std::shared_ptr<Context>(
        new Context{ setup.particles, setup.light_sources });
    context->thread_pool = 
        std::make_shared<utils::ThreadPool>(info.number_parallel_pipelines);
    context->parameters.intensity_cutoff = info.intensity_cutoff;
    context->parameters.medium_reflection_index = setup.medium_reflection_index;
    context->parameters.number_rays_per_buffer = info.number_rays_per_buffer;
    context->parameters.number_rays_per_unit = static_cast<size_t>(
        sqrt(info.light_source_ray_density_per_unit_area));
    context->parameters.maximum_branching_depth = info.maximum_branching_depth;
    context->parameters.number_parallel_pipelines = info.number_parallel_pipelines;
    context->flags.emit_warning_on_maximum_branching_depth_discardment =
        info.emit_warning_on_maximum_branching_depth_discardment;
    const size_t num_particles = context->particles.size();
    context->particle_transformation_data.p2w_data.resize(num_particles);
    context->particle_transformation_data.p2w_scale_rotation.resize(num_particles);
    context->particle_transformation_data.p2w_translation.resize(num_particles);
    context->particle_transformation_data.w2p_data.resize(num_particles);
    context->particle_transformation_data.w2p_rotation_scale.resize(num_particles);
    context->particle_transformation_data.w2p_translation.resize(num_particles);
    return createDataInstances(setup, info, context);
}

bool ldplab::rtsgpu_ogl::Factory::createDataInstances(
    const ExperimentalSetup& setup, 
    const RayTracingStepGPUOpenGLInfo& info,
    Context& context)
{
    bool no_error = true;
    if (m_bounding_volume_type == IBoundingVolume::Type::sphere)
        createBoundingSphereDataInstances(setup, info, context);
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create bounding volume "\
            "data instances, unsupported bounding volume type");
        no_error = false;
    }
    
    if (m_particle_geometry_type == IParticleGeometry::Type::rod_particle)
        createRodParticleDataInstances(setup, info, context);
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create particle "\
            "data instances, unsupported particle geometry type");
        no_error = false;
    }

    if (m_particle_material_type == IParticleMaterial::Type::linear_one_directional)
        createParticleMaterialLinearOneDirectionalInstances(setup, info, context);
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create particle "\
            "material data instances, unsupported particle material type");
        no_error = false;
    }
    return no_error;
}

void ldplab::rtsgpu_ogl::Factory::createBoundingSphereDataInstances(
    const ExperimentalSetup& setup, 
    const RayTracingStepGPUOpenGLInfo& info,
    Context& context)
{
    std::shared_ptr<BoundingSphereData> bv_data = 
        std::make_shared<BoundingSphereData>();
    context->bounding_volume_data = bv_data;
    bv_data->sphere_properties_data.resize(context->particles.size(), 
        BoundingSphereData::BoundingSphereProperties{ Vec3{ 0, 0, 0 }, 0 });
}

void ldplab::rtsgpu_ogl::Factory::createRodParticleDataInstances(
    const ExperimentalSetup& setup, 
    const RayTracingStepGPUOpenGLInfo& info,
    Context& context)
{
    std::shared_ptr<RodParticleData> particle_data = 
        std::make_shared<RodParticleData>(context);
    context->particle_data = particle_data;
    RodParticleData::RodParticleProperties t;
    double h;
    double sphere_radius;
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        const Particle& particle = setup.particles[i];
        RodParticleGeometry* geometry = 
            (RodParticleGeometry*)particle.geometry.get();
        if (geometry->kappa <= constant::rod_particle::min_kappa_threshold)
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
        t.cylinder_length = geometry->cylinder_length;
        t.cylinder_radius = geometry->cylinder_radius;
        t.cap_origin_z = geometry->cylinder_length + h - sphere_radius;
        t.indentation_origin_z = h - sphere_radius;
        t.sphere_radius = sphere_radius;
        particle_data->rod_particles_data.push_back(t);
    }
}

void ldplab::rtsgpu_ogl::Factory::createParticleMaterialLinearOneDirectionalInstances(
    const ExperimentalSetup& setup, 
    const RayTracingStepGPUOpenGLInfo& info, 
    Context& context)
{
    std::shared_ptr<ParticleMaterialLinearOneDirectionalData> pm_data =
        std::make_shared<ParticleMaterialLinearOneDirectionalData>(context);
    context->particle_material_data = pm_data;
    ParticleMaterialLinearOneDirectionalData::
        LinearOneDirectionalMaterialProperties t;
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        const Particle& particle = setup.particles[i];
        const ParticleMaterialLinearOneDirectional* pm =
            (ParticleMaterialLinearOneDirectional*)particle.material.get();
        t.direction_times_gradient = pm->direction_times_gradient;
        t.index_of_refraction_sum_term = pm->index_of_refraction_minus_partial_dot;
        pm_data->material_data.push_back(t);
    }
}

bool ldplab::rtsgpu_ogl::Factory::createPipelineStages(
    const ExperimentalSetup& setup,
    const RayTracingStepGPUOpenGLInfo& info,
    Context& context, 
    std::unique_ptr<IInitialStage>& initial, 
    std::unique_ptr<IInnerParticlePropagationStage>& ipp, 
    std::unique_ptr<IRayBoundingVolumeIntersectionTestStage>& rbvi, 
    std::unique_ptr<IRayParticleInteractionStage>& rpi, 
    std::unique_ptr<IRayParticleIntersectionTestStage>& rpit)
{
    bool no_errors = true;
    if (!createInitialStage(setup, info, context, initial))
        no_errors = false;
    if (!createInnerParticlePropagationStage(setup, info, context, ipp))
        no_errors = false;
    if (!createRayBoundingVolumeIntersectionTestStage(setup, info, context, rbvi))
        no_errors = false;
    if (!createRayParticleInteractionStage(setup, info, context, rpi))
        no_errors = false;
    if (!createRayParticleIntersectionTestStage(setup, info, context, rpit))
        no_errors = false;
    return no_errors;
}

bool ldplab::rtsgpu_ogl::Factory::createInitialStage(
    const ExperimentalSetup& setup,
    const RayTracingStepGPUOpenGLInfo& info,
    Context& context, 
    std::unique_ptr<IInitialStage>& stage)
{
    if (m_bounding_volume_type == IBoundingVolume::Type::sphere &&
        m_light_direction_type == ILightDirection::Type::homogeneous)
    {
        stage = std::unique_ptr<InitialStageBoundingSpheresHomogenousLight>(
            new InitialStageBoundingSpheresHomogenousLight(context));
        return true;
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create initial stage, "\
            "unsupported type combination");
        return false;
    }
}

bool ldplab::rtsgpu_ogl::Factory::createInnerParticlePropagationStage(
    const ExperimentalSetup& setup,
    const RayTracingStepGPUOpenGLInfo& info,
    Context& context, 
    std::unique_ptr<IInnerParticlePropagationStage>& stage)
{
    if (m_particle_material_type == IParticleMaterial::Type::linear_one_directional &&
        m_particle_geometry_type == IParticleGeometry::Type::rod_particle &&
        m_solver_type == IEikonalSolverParameter::Type::rk45)
    {
        stage = std::unique_ptr<LinearIndexGradientRodParticlePropagation>(
            new LinearIndexGradientRodParticlePropagation(
                context, *((RK45Parameter*) info.solver_parameters.get())));
        return true;
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create inner particle "\
            "propagation stage, unsupported type combination");
        return false;
    }
}

bool ldplab::rtsgpu_ogl::Factory::createRayBoundingVolumeIntersectionTestStage(
    const ExperimentalSetup& setup,
    const RayTracingStepGPUOpenGLInfo& info,
    Context& context, 
    std::unique_ptr<IRayBoundingVolumeIntersectionTestStage>& stage)
{
    if (m_bounding_volume_type == IBoundingVolume::Type::sphere)
    {
        stage = std::unique_ptr<RayBoundingSphereIntersectionTestStageBruteForce>(
            new RayBoundingSphereIntersectionTestStageBruteForce(context));
        return true;
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create ray bounding "\
            "volume intersection stage, unsupported type combination");
        return false;
    }
}

bool ldplab::rtsgpu_ogl::Factory::createRayParticleInteractionStage(
    const ExperimentalSetup& setup,
    const RayTracingStepGPUOpenGLInfo& info,
    Context& context,
    std::unique_ptr<IRayParticleInteractionStage>& stage)
{
    if (m_light_polarization_type == ILightPolarisation::Type::unpolarized &&
        m_particle_material_type == IParticleMaterial::Type::linear_one_directional)
    {
        stage = std::unique_ptr<UnpolirzedLight1DLinearIndexGradientInteraction>(
            new UnpolirzedLight1DLinearIndexGradientInteraction(context));
        return true;
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create ray particle "\
            "interaction stage, unsupported type combination");
        return false;
    }
}

bool ldplab::rtsgpu_ogl::Factory::createRayParticleIntersectionTestStage(
    const ExperimentalSetup& setup,
    const RayTracingStepGPUOpenGLInfo& info,
    Context& context, 
    std::unique_ptr<IRayParticleIntersectionTestStage>& stage)
{
    if (m_particle_geometry_type == IParticleGeometry::Type::rod_particle)
    {
        stage = std::unique_ptr<RodParticleIntersectionTest>(
            new RodParticleIntersectionTest(context));
        return true;
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSGPU factory (OpenGL): Failed to create ray particle "\
            "intersection test stage, unsupported type combination");
        return false;
    }
}

#endif