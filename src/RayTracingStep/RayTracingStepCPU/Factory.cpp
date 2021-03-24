#include "Factory.hpp"

#include <LDPLAB/Constants.hpp>
#include "../../Utils/Log.hpp"

std::shared_ptr<ldplab::rtscpu::RayTracingStep> 
    ldplab::rtscpu::Factory::createRTS(
        const ExperimentalSetup& setup, 
        const RayTracingStepCPUInfo& info)
{
    // Create the factory instance
    Factory factory;

    // Validate setup
    if (!factory.validateSetup(setup, info))
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create "\
            "rtscpu::RayTracingStep instance, invalid experimental setup");
        return nullptr;
    }

    // Create context
    std::shared_ptr<Context> context;
    if (!factory.createContext(setup, info, context))
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create "\
            "rtscpu::RayTracingStep instance, context creation failed");
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
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create "\
            "rtscpu::RayTracingStep instance, pipeline stage creation failed");
        return nullptr;
    }
    context->pipeline = std::make_unique<rtscpu::Pipeline>(
        std::move(initial),
        std::move(rbvit),
        std::move(rpit),
        std::move(rpi),
        std::move(ipp),
        context);

    return std::shared_ptr<RayTracingStep>(new RayTracingStep(context));
}

std::shared_ptr<ldplab::rtscpu::RayTracingStep> 
    ldplab::rtscpu::Factory::createRTSDebug(
        const ExperimentalSetup& setup, 
        const RayTracingStepCPUInfo& info, 
        RayTracingStepCPUDebugInfo& debug_info)
{
    // Create the factory instance
    Factory factory;

    // Validate setup
    if (!factory.validateSetup(setup, info))
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create "\
            "rtscpu::RayTracingStep instance, invalid experimental setup");
        return nullptr;
    }

    // Create context
    std::shared_ptr<Context> context;
    if (!factory.createContext(setup, info, context))
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create "\
            "rtscpu::RayTracingStep instance, context creation failed");
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
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create "\
            "rtscpu::RayTracingStep instance, pipeline stage creation failed");
        return nullptr;
    }

    // Add debug
    debug_info.context = context;
    debug_info.initial_stage = std::move(initial);
    debug_info.inner_particle_propagation = std::move(ipp);
    debug_info.ray_bounding_volume_intersection_test = std::move(rbvit);
    debug_info.ray_particle_interaction = std::move(rpi);
    debug_info.ray_particle_intersection_test = std::move(rpit);

    return std::shared_ptr<RayTracingStep>(new RayTracingStep(context));
}

bool ldplab::rtscpu::Factory::validateSetup(
    const ExperimentalSetup& setup,
    const RayTracingStepCPUInfo& info)
{
    bool error = false;   
    // Check if setup contains light and particles
    if (setup.light_sources.size() == 0)
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Experimental setup %i contains no "\
            "light sources", setup.uid);
        error = true;
    }
    if (setup.particles.size() == 0)
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Experimental setup %i contains no "\
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
        LDPLAB_LOG_ERROR("RTSCPU factory: Experimental setup %i contains "\
            "inhomogeneous types, which are not supported", setup.uid);
        return false;
    }
    // Setup is valid
    return true;
}

void ldplab::rtscpu::Factory::setTypes(
    const ExperimentalSetup& setup,
    const RayTracingStepCPUInfo& info)
{
    m_light_direction_type = setup.light_sources[0].direction->type();
    m_light_distribution_type = setup.light_sources[0].intensity_distribution->type();
    m_light_polarization_type = setup.light_sources[0].polarisation->type();
    m_bounding_volume_type = setup.particles[0].bounding_volume->type();
    m_particle_geometry_type = setup.particles[0].geometry->type();
    m_particle_material_type = setup.particles[0].material->type();
    m_solver_type = info.solver_parameters->type();
    LDPLAB_LOG_INFO("RTSCPU factory: Light direction type is %s",
        setup.light_sources[0].direction->typeString());
    LDPLAB_LOG_INFO("RTSCPU factory: Light intensity distribution type is %s",
        setup.light_sources[0].intensity_distribution->typeString());
    LDPLAB_LOG_INFO("RTSCPU factory: Light polarisation type is %s",
        setup.light_sources[0].polarisation->typeString());
    LDPLAB_LOG_INFO("RTSCPU factory: Particle bounding volume type is %s",
        setup.particles[0].bounding_volume->typeString());
    LDPLAB_LOG_INFO("RTSCPU factory: Particle geometry type is %s",
        setup.particles[0].geometry->typeString());
    LDPLAB_LOG_INFO("RTSCPU factory: Particle material type is %s",
        setup.particles[0].material->typeString());
    LDPLAB_LOG_INFO("RTSCPU factory: Eikonal solver type is %s",
        info.solver_parameters->typeString());
}

bool ldplab::rtscpu::Factory::validateTypeHomogeneity(
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
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent light "\
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
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent light "\
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
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent light "\
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
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent particle "\
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
            LDPLAB_LOG_ERROR("RTSCPU factory: Found inconsistent particle "\
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

bool ldplab::rtscpu::Factory::createContext(
    const ExperimentalSetup& setup,
    const RayTracingStepCPUInfo& info,
    std::shared_ptr<Context>& context)
{
    context = std::shared_ptr<Context>(
        new Context{ setup.particles, setup.light_sources });
    context->thread_pool = 
        std::make_shared<utils::ThreadPool>(info.number_parallel_pipelines);
    context->particle_transformations.resize(context->particles.size());
    context->parameters.intensity_cutoff = info.intensity_cutoff;
    context->parameters.medium_reflection_index = setup.medium_reflection_index;
    context->parameters.number_rays_per_buffer = info.number_rays_per_buffer;
    context->parameters.number_rays_per_unit = static_cast<size_t>(
        sqrt(info.light_source_ray_density_per_unit_area));
    context->parameters.maximum_branching_depth = info.maximum_branching_depth;
    context->parameters.number_parallel_pipelines = info.number_parallel_pipelines;
    context->flags.emit_warning_on_maximum_branching_depth_discardment =
        info.emit_warning_on_maximum_branching_depth_discardment;
    return createDataInstances(setup, info, context);
}

bool ldplab::rtscpu::Factory::createDataInstances(
    const ExperimentalSetup& setup, 
    const RayTracingStepCPUInfo& info, 
    std::shared_ptr<Context> context)
{
    bool no_error = true;
    if (m_bounding_volume_type == IBoundingVolume::Type::sphere)
        createBoundingSphereDataInstances(setup, info, context);
    else
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create bounding volume "\
            "data instances, unsupported bounding volume type");
        no_error = false;
    }
    if (m_particle_geometry_type == IParticleGeometry::Type::rod_particle)
        createRodParticleDataInstances(setup, info, context);
    else if (m_particle_geometry_type == IParticleGeometry::Type::sphere)
    {
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create rod particle "\
            "data instances, unsupported particle geometry type");
        no_error = false;
    }
    return no_error;
}

void ldplab::rtscpu::Factory::createBoundingSphereDataInstances(
    const ExperimentalSetup& setup, 
    const RayTracingStepCPUInfo& info, 
    std::shared_ptr<Context> context)
{
    context->bounding_volume_data =
        std::shared_ptr<rtscpu::IBoundingVolumeData>(
            new rtscpu::BoundingSphereData());
    ((rtscpu::BoundingSphereData*)context->bounding_volume_data.get())->
        sphere_data.resize(context->particles.size(),
            BoundingVolumeSphere(Vec3{ 0, 0, 0 }, 0));
}

void ldplab::rtscpu::Factory::createRodParticleDataInstances(
    const ExperimentalSetup& setup, 
    const RayTracingStepCPUInfo& info, 
    std::shared_ptr<Context> context)
{
    std::shared_ptr<RodParticleData> particle_data = 
        std::make_shared<RodParticleData>();
    context->particle_data = particle_data;
    RodParticle t;
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
        t.origin_cap = Vec3(0.0, 0.0, geometry->cylinder_length + h - sphere_radius);
        t.origin_indentation = Vec3(0.0, 0.0, h - sphere_radius);
        t.sphere_radius = sphere_radius;
        particle_data->particle_data.push_back(t);
    }
}

bool ldplab::rtscpu::Factory::createPipelineStages(
    const ExperimentalSetup& setup,
    const RayTracingStepCPUInfo& info,
    std::shared_ptr<Context> context, 
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

bool ldplab::rtscpu::Factory::createInitialStage(
    const ExperimentalSetup& setup,
    const RayTracingStepCPUInfo& info,
    std::shared_ptr<Context> context, 
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
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create initial stage, "\
            "unsupported type combination");
        return false;
    }
}

bool ldplab::rtscpu::Factory::createInnerParticlePropagationStage(
    const ExperimentalSetup& setup,
    const RayTracingStepCPUInfo& info,
    std::shared_ptr<Context> context, 
    std::unique_ptr<IInnerParticlePropagationStage>& stage)
{
    if (m_particle_material_type == IParticleMaterial::Type::linear_one_directional &&
        m_particle_geometry_type == IParticleGeometry::Type::rod_particle &&
        m_solver_type == IEikonalSolver::Type::rk45)
    {
        stage = std::unique_ptr <rtscpu::RK45RodParticlePropagation>(
            new rtscpu::RK45RodParticlePropagation(
                context,
                *((RK45*)info.solver_parameters.get())));
        return true;
    }
    else if (m_particle_material_type == IParticleMaterial::Type::linear_one_directional &&
        m_particle_geometry_type == IParticleGeometry::Type::rod_particle &&
        m_solver_type == IEikonalSolver::Type::rk4)
    {
        stage = std::unique_ptr <rtscpu::RK4RodParticlePropagation>(
            new rtscpu::RK4RodParticlePropagation(
                context,
                *((RK4*)info.solver_parameters.get())));
        return true;
    }
    else if (m_particle_material_type == IParticleMaterial::Type::linear_one_directional &&
        m_particle_geometry_type == IParticleGeometry::Type::sphere &&
        m_solver_type == IEikonalSolver::Type::rk45)
    {
        stage = std::unique_ptr <rtscpu::RK45SphericalParticlePropagation>(
            new rtscpu::RK45SphericalParticlePropagation(
                context,
                *((RK45*)info.solver_parameters.get())) );
        return true;
    }
    else if (m_particle_material_type == IParticleMaterial::Type::linear_one_directional &&
        m_particle_geometry_type == IParticleGeometry::Type::sphere &&
        m_solver_type == IEikonalSolver::Type::rk4)
    {
        stage = std::unique_ptr <rtscpu::RK4SphericalParticlePropagation>( 
            new rtscpu::RK4SphericalParticlePropagation(
                context,
                *((RK4*)info.solver_parameters.get())));
        return true;
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create inner particle "\
            "propagation stage, unsupported type combination");
        return false;
    }
}

bool ldplab::rtscpu::Factory::createRayBoundingVolumeIntersectionTestStage(
    const ExperimentalSetup& setup,
    const RayTracingStepCPUInfo& info,
    std::shared_ptr<Context> context, 
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
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create ray bounding "\
            "volume intersection stage, unsupported type combination");
        return false;
    }
}

bool ldplab::rtscpu::Factory::createRayParticleInteractionStage(
    const ExperimentalSetup& setup,
    const RayTracingStepCPUInfo& info,
    std::shared_ptr<Context> context,
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
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create ray particle "\
            "interaction stage, unsupported type combination");
        return false;
    }
}

bool ldplab::rtscpu::Factory::createRayParticleIntersectionTestStage(
    const ExperimentalSetup& setup,
    const RayTracingStepCPUInfo& info,
    std::shared_ptr<Context> context, 
    std::unique_ptr<IRayParticleIntersectionTestStage>& stage)
{
    if (m_particle_geometry_type == IParticleGeometry::Type::rod_particle)
    {
        stage = std::unique_ptr<RodParticleIntersectionTest>(
            new RodParticleIntersectionTest(context));
        return true;
    }
    else if (m_particle_geometry_type == IParticleGeometry::Type::sphere)
    {
        stage = std::unique_ptr<SphericalParticleIntersectionTest>(
            new SphericalParticleIntersectionTest(context));
        return true;
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCPU factory: Failed to create ray particle "\
            "intersection test stage, unsupported type combination");
        return false;
    }
}
