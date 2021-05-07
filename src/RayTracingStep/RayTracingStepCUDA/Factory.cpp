#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include "Factory.hpp"

#include <LDPLAB/Constants.hpp>
#include "../../Utils/Log.hpp"

std::shared_ptr<ldplab::rtscuda::RayTracingStep> 
    ldplab::rtscuda::Factory::createRTS(
        const ExperimentalSetup& setup, 
        const RayTracingStepCUDAInfo& info)
{
    // Create the factory instance
    Factory factory;

    // Validate setup
    if (!factory.validateSetup(setup, info))
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create "\
            "rtscuda::RayTracingStep instance, invalid experimental setup");
        return nullptr;
    }

    // Create context
    std::unique_ptr<Context> context;
    if (!factory.createContext(setup, info, context))
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create "\
            "rtscuda::RayTracingStep instance, context creation failed");
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
        *context,
        initial,
        ipp,
        rbvit,
        rpi,
        rpit))
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create "\
            "rtscuda::RayTracingStep instance, pipeline stage creation failed");
        return nullptr;
    }
    context->pipeline = std::make_unique<rtscuda::Pipeline>(
        std::move(initial),
        std::move(rbvit),
        std::move(rpit),
        std::move(rpi),
        std::move(ipp),
        *context);

    return std::shared_ptr<RayTracingStep>(new RayTracingStep(std::move(context)));
}

bool ldplab::rtscuda::Factory::validateSetup(
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info)
{
    bool error = false;   
    // Check if setup contains light and particles
    if (setup.light_sources.size() == 0)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Experimental setup %i contains no "\
            "light sources", setup.uid);
        error = true;
    }
    if (setup.particles.size() == 0)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Experimental setup %i contains no "\
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
        LDPLAB_LOG_ERROR("RTSCUDA factory: Experimental setup %i contains "\
            "inhomogeneous types, which are not supported", setup.uid);
        return false;
    }
    // Setup is valid
    return true;
}

void ldplab::rtscuda::Factory::setTypes(
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info)
{
    m_light_direction_type = setup.light_sources[0].direction->type();
    m_light_distribution_type = setup.light_sources[0].intensity_distribution->type();
    m_light_polarization_type = setup.light_sources[0].polarisation->type();
    m_bounding_volume_type = setup.particles[0].bounding_volume->type();
    m_particle_material_type = setup.particles[0].material->type();
    m_solver_type = info.solver_parameters->type();
    LDPLAB_LOG_INFO("RTSCUDA factory: Light direction type is %s",
        setup.light_sources[0].direction->typeString());
    LDPLAB_LOG_INFO("RTSCUDA factory: Light intensity distribution type is %s",
        setup.light_sources[0].intensity_distribution->typeString());
    LDPLAB_LOG_INFO("RTSCUDA factory: Light polarisation type is %s",
        setup.light_sources[0].polarisation->typeString());
    LDPLAB_LOG_INFO("RTSCUDA factory: Particle bounding volume type is %s",
        setup.particles[0].bounding_volume->typeString());
    LDPLAB_LOG_INFO("RTSCUDA factory: Particle material type is %s",
        setup.particles[0].material->typeString());
    LDPLAB_LOG_INFO("RTSCUDA factory: Eikonal solver type is %s",
        info.solver_parameters->typeString());
}

bool ldplab::rtscuda::Factory::validateTypeHomogeneity(
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
            LDPLAB_LOG_ERROR("RTSCUDA factory: Found inconsistent light "\
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
            LDPLAB_LOG_ERROR("RTSCUDA factory: Found inconsistent light "\
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
            LDPLAB_LOG_ERROR("RTSCUDA factory: Found inconsistent light "\
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
            LDPLAB_LOG_ERROR("RTSCUDA factory: Found inconsistent particle "\
                "bounding volume type in particle %i, type was %s but "\
                "expected %s",
                setup.particles[i].uid,
                setup.particles[i].bounding_volume->typeString(),
                setup.particles[0].bounding_volume->typeString());
        }
        if (setup.particles[i].material->type() !=
            m_particle_material_type)
        {
            only_homogenous_types = false;
            LDPLAB_LOG_ERROR("RTSCUDA factory: Found inconsistent particle "\
                "material type in particle %i, type was %s but "\
                "expected %s",
                setup.particles[i].uid,
                setup.particles[i].material->typeString(),
                setup.particles[0].material->typeString());
        }
    }
    return only_homogenous_types;
}

bool ldplab::rtscuda::Factory::createContext(
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info,
    std::unique_ptr<Context>& context)
{
    context = std::unique_ptr<Context>(
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
    context->flags.return_force_in_particle_coordinate_system =
        info.return_force_in_particle_coordinate_system;
    return createDataInstances(setup, info, context);
}

bool ldplab::rtscuda::Factory::createDataInstances(
    const ExperimentalSetup& setup, 
    const RayTracingStepCUDAInfo& info, 
    std::unique_ptr<Context>& context)
{
    bool no_error = true;
    if (m_bounding_volume_type == IBoundingVolume::Type::sphere)
        createBoundingSphereDataInstances(setup, info, *context);
    else
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create bounding volume "\
            "data instances, unsupported bounding volume type");
        no_error = false;
    }
    if (!createParticleGenericGeometryInstances(setup, info, *context))
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create generic particle "\
            "geometry data instances");
        no_error = false;
    }
    return no_error;
}

bool ldplab::rtscuda::Factory::createParticleGenericGeometryInstances(
    const ExperimentalSetup& setup, 
    const RayTracingStepCUDAInfo& info, 
    Context& context)
{
    context.particle_data = std::make_shared<ParticleData>();
    std::vector<std::shared_ptr<IGenericGeometry>>& geometries =
        context.particle_data->geometries;
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        IParticleGeometry* particle_geometry = setup.particles[i].geometry.get();
        if (particle_geometry->type() == IParticleGeometry::Type::rod_particle)
        {
            geometries.emplace_back(new RodGeometry(
                (RodParticleGeometry*) particle_geometry));
        }
        else if (particle_geometry->type() == IParticleGeometry::Type::sphere)
        {
            geometries.emplace_back(new SphericalGeometry(
                (SphericalParticleGeometry*) particle_geometry));
        }
        else if (particle_geometry->type() == IParticleGeometry::Type::triangle_mesh)
        {
            std::shared_ptr<IGenericGeometry> geometry;
            if (!constructTriangleMeshAcceleratorStructure(
                (TriangleMeshParticleGeometry*)particle_geometry,
                info,
                geometry))
                return false;
            geometries.emplace_back(std::move(geometry));
        }
        else
        {
            LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create generic "\
                "geometry instance for particle %i, particle geometry type "\
                "%s unknown", 
                setup.particles[0].uid,
                particle_geometry->typeString());
            return false;
        }
    }
    return true;
}

bool ldplab::rtscuda::Factory::constructTriangleMeshAcceleratorStructure(
    const TriangleMeshParticleGeometry* particle_geometry, 
    const RayTracingStepCUDAInfo& info, 
    std::shared_ptr<IGenericGeometry>& output)
{
    if (info.accelerator_structure_parameters == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create particle "\
            "geometry accelerator structure, no accelerator "\
            "paramters given");
        return false;
    }
    else if (info.accelerator_structure_parameters->type() ==
        IAcceleratorStructureParameter::Type::brute_force)
    {
        std::shared_ptr<TriangleMeshGeometryList> list =
            std::make_shared<TriangleMeshGeometryList>();
        if (!list->construct(particle_geometry->mesh,
            info.accelerator_structure_parameters.get()))
            return false;
        output = std::move(list);
    }
    else if (info.accelerator_structure_parameters->type() ==
        IAcceleratorStructureParameter::Type::octree)
    {
        std::shared_ptr<TriangleMeshGeometryList> octree =
            std::make_shared<TriangleMeshGeometryList>();
        if (!octree->construct(particle_geometry->mesh,
            info.accelerator_structure_parameters.get()))
            return false;
        output = std::move(octree);
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create particle "\
            "geometry accelerator structure, accelerator structure "\
            "type %s unknown",
            info.accelerator_structure_parameters->typeString());
        return false;
    }
    return true;
}

void ldplab::rtscuda::Factory::createBoundingSphereDataInstances(
    const ExperimentalSetup& setup, 
    const RayTracingStepCUDAInfo& info, 
    Context& context)
{
    context.bounding_volume_data =
        std::shared_ptr<rtscuda::IBoundingVolumeData>(
            new rtscuda::BoundingSphereData());
    ((rtscuda::BoundingSphereData*)context.bounding_volume_data.get())->
        sphere_data.resize(context.particles.size(),
            BoundingVolumeSphere(Vec3{ 0, 0, 0 }, 0));
}

bool ldplab::rtscuda::Factory::createPipelineStages(
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info,
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

bool ldplab::rtscuda::Factory::createInitialStage(
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info,
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
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create initial stage, "\
            "unsupported type combination");
        return false;
    }
}

bool ldplab::rtscuda::Factory::createInnerParticlePropagationStage(
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info,
    Context& context, 
    std::unique_ptr<IInnerParticlePropagationStage>& stage)
{
    if (m_particle_material_type == IParticleMaterial::Type::linear_one_directional &&
        m_solver_type == IEikonalSolverParameter::Type::rk45)
    {
        stage = std::make_unique<rtscuda::EikonalSolverRK45LinearIndexGradient>(
            context,
            *((RK45Parameter*)info.solver_parameters.get()));
        return true;
    }
    else if (m_particle_material_type == IParticleMaterial::Type::linear_one_directional &&
        m_solver_type == IEikonalSolverParameter::Type::rk4)
    {
        stage = std::make_unique<rtscuda::EikonalSolverRK4LinearIndexGradient>(
            context,
            *((RK4Parameter*)info.solver_parameters.get()));
        return true;
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create inner particle "\
            "propagation stage, unsupported type combination");
        return false;
    }
}

bool ldplab::rtscuda::Factory::createRayBoundingVolumeIntersectionTestStage(
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info,
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
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create ray bounding "\
            "volume intersection stage, unsupported type combination");
        return false;
    }
}

bool ldplab::rtscuda::Factory::createRayParticleInteractionStage(
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info,
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
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create ray particle "\
            "interaction stage, unsupported type combination");
        return false;
    }
}

bool ldplab::rtscuda::Factory::createRayParticleIntersectionTestStage(
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info,
    Context& context, 
    std::unique_ptr<IRayParticleIntersectionTestStage>& stage)
{
    stage = std::make_unique<RayParticleGenericGeometryIntersectionTest>(context);
    return true;
    //if (m_particle_geometry_type == IParticleGeometry::Type::rod_particle)
    //{
    //    stage = std::unique_ptr<RodParticleIntersectionTest>(
    //        new RodParticleIntersectionTest(context));
    //    return true;
    //}
    //else if (m_particle_geometry_type == IParticleGeometry::Type::sphere)
    //{
    //    stage = std::unique_ptr<SphericalParticleIntersectionTest>(
    //        new SphericalParticleIntersectionTest(context));
    //    return true;
    //}
    //else if (m_particle_geometry_type == IParticleGeometry::Type::triangle_mesh)
    //{
    //    stage = std::unique_ptr<MeshParticleIntersectionTest>(
    //        new MeshParticleIntersectionTest(context));
    //    return true;
    //}
    //else
    //{
    //    LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create ray particle "\
    //        "intersection test stage, unsupported type combination");
    //    return false;
    //}
}

#endif