#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "Factory.hpp"

#include "../../Utils/Log.hpp"

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>
#include <LDPLAB/RayTracingStep/CUDA/Factories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultBoundingVolumeIntersectionFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultGenericGeometryFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultInitialStageFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultInnerParticlePropagationFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultParticleIntersectionFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultSurfaceInteractionFactories.hpp>

std::shared_ptr<ldplab::rtscuda::RayTracingStepCUDA> 
    ldplab::rtscuda::Factory::createRTS(
        const RayTracingStepCUDAInfo& info, 
        ExperimentalSetup&& setup)
{
    return createRTS(info, std::move(setup), PipelineConfiguration{ }, true);
}

std::shared_ptr<ldplab::rtscuda::RayTracingStepCUDA> 
    ldplab::rtscuda::Factory::createRTS(
        const RayTracingStepCUDAInfo& info, 
        ExperimentalSetup&& setup, 
        PipelineConfiguration& user_configuration,
        bool allow_default_stage_overwrite_on_compability_error)
{
    // First, retrieve the default configuration
    PipelineConfiguration default_configuration;
    createDefaultConfiguration(info, setup, default_configuration);

    // Retrieve the present geometry type set
    std::set<IParticleGeometry::Type> present_geometry_types =
        std::move(getPresentGeometryTypes(setup));

    // Now combine user given config and default configuration
    PipelineConfiguration pipeline_configuration;
    if (!combineConfigurations(
        present_geometry_types,
        default_configuration,
        user_configuration,
        pipeline_configuration))
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Failed to create "\
            "RayTracingStepCPU");
        return nullptr;
    }

    // Create the interface mapping
    InterfaceMapping interface_mapping = createInterfaceMapping(setup);

    // Find a viable pipeline configuration
    if (!createViableConfiguration(
        info,
        setup,
        interface_mapping,
        present_geometry_types,
        pipeline_configuration,
        default_configuration,
        user_configuration,
        allow_default_stage_overwrite_on_compability_error))
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Failed to create "\
            "RayTracingStepCPU");
        return nullptr;
    }

    // Log viable configuration
    logViableConfiguration(pipeline_configuration);

    // With a viable configuration it is now time to actually 
    std::shared_ptr<RayTracingStepCUDA> rts =
        std::make_shared<RayTracingStepCUDA>();

    // Create pipeline
    if (!createPipeline(
        info,
        std::move(interface_mapping),
        std::move(setup),
        pipeline_configuration,
        rts))
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Failed to create "\
            "pipeline");
        return nullptr;
    }
    return rts;
}

void ldplab::rtscuda::Factory::createDefaultConfiguration(
    const RayTracingStepCUDAInfo& info, 
    const ExperimentalSetup& setup, 
    PipelineConfiguration& default_config)
{
    default_config.initial_stage = std::make_shared<
        default_factories::InitialStageHomogenousLightBoundingSphereProjectionFactory>(140);
    default_config.bounding_volume_intersection = std::make_shared<
        default_factories::BoundingSphereIntersectionBruteforceFactory>();
    default_config.particle_intersection = std::make_shared<
        default_factories::ParticleIntersectionFactory>();
    default_config.surface_interaction = std::make_shared<
        default_factories::SurfaceInteractionFactory>();
    RK4Parameter rk4_parameter = { 0.005 };
    default_config.inner_particle_propagation = std::make_shared<
        default_factories::InnerParticlePropagationRK4Factory>(rk4_parameter);

    default_config.generic_geometries.emplace(
        IParticleGeometry::Type::rod_particle,
        std::make_shared<default_factories::GenericGeometryRodFactory>());
    default_config.generic_geometries.emplace(
        IParticleGeometry::Type::sphere,
        std::make_shared<default_factories::GenericGeometrySphereFactory>());
}

std::set<ldplab::IParticleGeometry::Type> 
    ldplab::rtscuda::Factory::getPresentGeometryTypes(
        const ExperimentalSetup& setup)
{
    std::set<ldplab::IParticleGeometry::Type> types;
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        if (types.find(setup.particles[i].geometry->type()) ==
            types.end())
        {
            types.insert(setup.particles[i].geometry->type());
        }
    }
    return types;
}

bool ldplab::rtscuda::Factory::combineConfigurations(
    std::set<IParticleGeometry::Type>& geometry_types, 
    PipelineConfiguration& default_config, 
    PipelineConfiguration& user_config, 
    PipelineConfiguration& combination)
{
    // Add user config
    if (user_config.bounding_volume_intersection != nullptr)
        combination.bounding_volume_intersection = user_config.bounding_volume_intersection;
    if (user_config.initial_stage != nullptr)
        combination.initial_stage = user_config.initial_stage;
    if (user_config.inner_particle_propagation != nullptr)
        combination.inner_particle_propagation = user_config.inner_particle_propagation;
    if (user_config.particle_intersection != nullptr)
        combination.particle_intersection = user_config.particle_intersection;
    if (user_config.surface_interaction != nullptr)
        combination.surface_interaction = user_config.surface_interaction;
    auto it = user_config.generic_geometries.begin();
    for (; it != user_config.generic_geometries.end(); it++)
    {
        if (it->second != nullptr ||
            geometry_types.find(it->first) != geometry_types.end())
            combination.generic_geometries.emplace(it->first, it->second);
    }

    // Add default config for slots that aren't set by the user config
    if (combination.bounding_volume_intersection == nullptr)
        combination.bounding_volume_intersection = default_config.bounding_volume_intersection;
    if (combination.initial_stage == nullptr)
        combination.initial_stage = default_config.initial_stage;
    if (combination.inner_particle_propagation == nullptr)
        combination.inner_particle_propagation = default_config.inner_particle_propagation;
    if (combination.particle_intersection == nullptr)
        combination.particle_intersection = default_config.particle_intersection;
    if (combination.surface_interaction == nullptr)
        combination.surface_interaction = default_config.surface_interaction;
    it = default_config.generic_geometries.begin();
    for (; it != default_config.generic_geometries.end(); it++)
    {
        if (it->second != nullptr &&
            geometry_types.find(it->first) != geometry_types.end())
        {
            if (combination.generic_geometries.find(it->first) ==
                combination.generic_geometries.end())
            {
                combination.generic_geometries.emplace(it->first, it->second);
            }
        }
    }

    // Check if the combined configuration is fully cast
    bool error = false;
    if (combination.bounding_volume_intersection == nullptr)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
            "is missing a suitable bounding volume interaction factory");
        error = true;
    }
    else if (combination.initial_stage == nullptr)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
            "is missing a suitable initial stage factory");
        error = true;
    }
    else if (combination.inner_particle_propagation == nullptr)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
            "is missing a suitable inner particle propagation factory");
        error = true;
    }
    else if (combination.particle_intersection == nullptr)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
            "is missing a suitable particle intersection factory");
        error = true;
    }
    else if (combination.surface_interaction == nullptr)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
            "is missing a suitable surface interaction factory");
        error = true;
    }
    auto types_it = geometry_types.begin();
    for (; types_it != geometry_types.end(); types_it++)
    {
        if (combination.generic_geometries.find(*types_it) ==
            combination.generic_geometries.end())
        {
            LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
                "is missing a suitable generic geometry factory for the "\
                "particle geometry type \"%s\"",
                IParticleGeometry::typeToString(*types_it));
            error = true;
        }
    }

    return !error;
}

ldplab::rtscuda::InterfaceMapping 
    ldplab::rtscuda::Factory::createInterfaceMapping(
        const ExperimentalSetup& setup)
{
    InterfaceMapping mapping;
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        mapping.particle_index_to_uid.emplace(i, setup.particles[i].uid);
        mapping.particle_uid_to_index.emplace(setup.particles[i].uid, i);
    }
    return mapping;
}

bool ldplab::rtscuda::Factory::createViableConfiguration(
    const RayTracingStepCUDAInfo& info, 
    const ExperimentalSetup& setup, 
    const InterfaceMapping& interface_mapping, 
    std::set<IParticleGeometry::Type>& geometry_types, 
    PipelineConfiguration& configuration, 
    PipelineConfiguration& default_configuration, 
    PipelineConfiguration& user_config, 
    bool allow_default_stage_overwrite_on_compability_error)
{
    // Validate first
    PipelineConfigurationBooleanState config_state =
        validateConfigurationCompability(
            info,
            setup,
            interface_mapping,
            configuration);

    // Check if compatible
    bool compatible = checkForConfigurationStateUniformity(config_state, true);
    if (compatible)
        return true;

    // Swap stages to defaults, if allowed
    if (allow_default_stage_overwrite_on_compability_error)
    {
        // Begin to swap stages to default
        PipelineConfigurationBooleanState user_defined_stages =
            checkConfigurationCast(geometry_types, user_config);
        bool has_user_defined_stages =
            !checkForConfigurationStateUniformity(user_defined_stages, false);
        while (has_user_defined_stages)
        {
            // Swap stages and update user defined stages states
            if (!config_state.bounding_volume_intersection_state)
            {
                if (!user_defined_stages.bounding_volume_intersection_state)
                    break;
                else
                {
                    LDPLAB_LOG_WARNING("Ray Tracing Step CPU Factory: "\
                        "Swapping incompatible user defined bounding volume "\
                        "intersection \"%s\" with default \"%s\"",
                        configuration.bounding_volume_intersection->implementationName(),
                        default_configuration.bounding_volume_intersection->implementationName());
                    configuration.bounding_volume_intersection =
                        default_configuration.bounding_volume_intersection;
                    user_defined_stages.bounding_volume_intersection_state = false;
                }
            }
            if (!config_state.initial_stage_state)
            {
                if (!user_defined_stages.initial_stage_state)
                    break;
                else
                {
                    LDPLAB_LOG_WARNING("Ray Tracing Step CPU Factory: "\
                        "Swapping incompatible user defined initial stage "\
                        "\"%s\" with default \"%s\"",
                        configuration.initial_stage->implementationName(),
                        default_configuration.initial_stage->implementationName());
                    configuration.initial_stage = default_configuration.initial_stage;
                    user_defined_stages.initial_stage_state = false;
                }
            }
            if (!config_state.inner_particle_propagation_state)
            {
                if (!user_defined_stages.inner_particle_propagation_state)
                    break;
                else
                {
                    LDPLAB_LOG_WARNING("Ray Tracing Step CPU Factory: "\
                        "Swapping incompatible user defined inner particle "\
                        "propagation \"%s\" with default \"%s\"",
                        configuration.inner_particle_propagation->implementationName(),
                        default_configuration.inner_particle_propagation->implementationName());
                    configuration.inner_particle_propagation =
                        default_configuration.inner_particle_propagation;
                    user_defined_stages.inner_particle_propagation_state = false;
                }
            }
            if (!config_state.particle_intersection_state)
            {
                if (!user_defined_stages.particle_intersection_state)
                    break;
                else
                {
                    LDPLAB_LOG_WARNING("Ray Tracing Step CPU Factory: "\
                        "Swapping incompatible user defined particle "\
                        "intersection \"%s\" with default \"%s\"",
                        configuration.particle_intersection->implementationName(),
                        default_configuration.particle_intersection->implementationName());
                    configuration.particle_intersection =
                        default_configuration.particle_intersection;
                    user_defined_stages.particle_intersection_state = false;
                }
            }
            if (!config_state.surface_interaction_state)
            {
                if (!user_defined_stages.surface_interaction_state)
                    break;
                else
                {
                    LDPLAB_LOG_WARNING("Ray Tracing Step CPU Factory: "\
                        "Swapping incompatible user defined surface "\
                        "interaction \"%s\" with default \"%s\"",
                        configuration.surface_interaction->implementationName(),
                        default_configuration.surface_interaction->implementationName());
                    configuration.surface_interaction =
                        default_configuration.surface_interaction;
                    user_defined_stages.surface_interaction_state = false;
                }
            }

            auto cs_it = config_state.generic_geometry_state.begin();
            for (; cs_it != config_state.generic_geometry_state.end(); cs_it++)
            {
                auto ud_geo_state =
                    user_defined_stages.generic_geometry_state.find(cs_it->first);
                if (ud_geo_state == user_defined_stages.generic_geometry_state.end() ||
                    ud_geo_state->second == false)
                    break;
                else
                {
                    auto config_stage =
                        configuration.generic_geometries.find(cs_it->first);
                    auto default_stage =
                        default_configuration.generic_geometries.find(cs_it->first);
                    if (config_stage == configuration.generic_geometries.end() ||
                        default_stage == default_configuration.generic_geometries.end())
                        break;
                    LDPLAB_LOG_WARNING("Ray Tracing Step CPU Factory: "\
                        "Swapping incompatible generic geometry \"%s\" with "\
                        "default implementation \"%s\"",
                        config_stage->second->implementationName(),
                        default_stage->second->implementationName());
                    config_stage->second = default_stage->second;
                    ud_geo_state->second = false;
                }
            }

            // Check configuration
            config_state = validateConfigurationCompability(
                info,
                setup,
                interface_mapping,
                configuration);

            // Check if state is compatible
            compatible =
                checkForConfigurationStateUniformity(config_state, true);
            if (compatible)
                return true;

            // Check if there are still user defined stages
            has_user_defined_stages =
                !checkForConfigurationStateUniformity(
                    user_defined_stages, false);
        }
    }

    // Print errors
    if (!config_state.bounding_volume_intersection_state)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline bounding "\
            "volume intersection stage \"%s\" is incompatible with the given "\
            "configuration or experimental setup",
            configuration.bounding_volume_intersection->implementationName());
    }
    if (!config_state.initial_stage_state)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline initial "\
            "stage \"%s\" is incompatible with the given configuration or "\
            "experimental setup",
            configuration.initial_stage->implementationName());
    }
    if (!config_state.inner_particle_propagation_state)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline inner "\
            "particle propagation stage \"%s\" is incompatible with the given "\
            "configuration or experimental setup",
            configuration.inner_particle_propagation->implementationName());
    }
    if (!config_state.particle_intersection_state)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline particle "\
            "intersection stage \"%s\" is incompatible with the given "\
            "configuration or experimental setup",
            configuration.particle_intersection->implementationName());
    }
    if (!config_state.surface_interaction_state)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline surface "\
            "interaction stage \"%s\" is incompatible with the given "\
            "configuration or experimental setup",
            configuration.surface_interaction->implementationName());
    }
    for (auto it = config_state.generic_geometry_state.begin();
        it != config_state.generic_geometry_state.end();
        it++)
    {
        if (!it->second)
        {
            auto geo = configuration.generic_geometries.find(it->first);
            LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Generic "\
                "geometry implementation \"%s\" for geometry type \"%s\" is "\
                "incompatible with the given configuration or "\
                "experimental setup",
                geo->second->implementationName(),
                IParticleGeometry::typeToString(it->first));
        }
    }
    return false;
}

ldplab::rtscuda::Factory::PipelineConfigurationBooleanState 
    ldplab::rtscuda::Factory::validateConfigurationCompability(
        const RayTracingStepCUDAInfo& info, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping, 
        PipelineConfiguration& configuration)
{
    return PipelineConfigurationBooleanState();
}

ldplab::rtscuda::Factory::PipelineConfigurationBooleanState 
    ldplab::rtscuda::Factory::checkConfigurationCast(
        std::set<IParticleGeometry::Type>& geometry_types, 
        PipelineConfiguration& configuration)
{
    PipelineConfigurationBooleanState state;
    state.bounding_volume_intersection_state =
        (configuration.bounding_volume_intersection != nullptr);
    state.initial_stage_state =
        (configuration.initial_stage != nullptr);
    state.inner_particle_propagation_state =
        (configuration.inner_particle_propagation != nullptr);
    state.particle_intersection_state =
        (configuration.particle_intersection != nullptr);
    state.surface_interaction_state =
        (configuration.surface_interaction != nullptr);
    for (auto it = geometry_types.begin(); it != geometry_types.end(); ++it)
    {
        auto geo = configuration.generic_geometries.find(*it);
        if (geo == configuration.generic_geometries.end())
            state.generic_geometry_state.emplace(*it, false);
        else
            state.generic_geometry_state.emplace(*it, geo->second != nullptr);
    }
    return state;
}

bool ldplab::rtscuda::Factory::checkForConfigurationStateUniformity(
    const PipelineConfigurationBooleanState& configuration_state, 
    bool desired_uniform_state)
{
    if (configuration_state.bounding_volume_intersection_state != desired_uniform_state ||
        configuration_state.initial_stage_state != desired_uniform_state ||
        configuration_state.inner_particle_propagation_state != desired_uniform_state ||
        configuration_state.particle_intersection_state != desired_uniform_state ||
        configuration_state.surface_interaction_state != desired_uniform_state)
        return false;
    for (auto it = configuration_state.generic_geometry_state.begin();
        it != configuration_state.generic_geometry_state.end();
        it++)
    {
        if (it->second != desired_uniform_state)
            return false;
    }
    return true;
}

void ldplab::rtscuda::Factory::logViableConfiguration(
    PipelineConfiguration& config)
{
    LDPLAB_LOG_INFO("Ray Tracing Step CPU Factory: "\
        "Pipeline configuration uses initial stage \"%s\"",
        config.initial_stage->implementationName());
    LDPLAB_LOG_INFO("Ray Tracing Step CPU Factory: "\
        "Pipeline configuration uses bounding volume intersection stage \"%s\"",
        config.bounding_volume_intersection->implementationName());
    LDPLAB_LOG_INFO("Ray Tracing Step CPU Factory: "\
        "Pipeline configuration uses particle intersection stage \"%s\"",
        config.particle_intersection->implementationName());
    LDPLAB_LOG_INFO("Ray Tracing Step CPU Factory: "\
        "Pipeline configuration uses surface interaction stage \"%s\"",
        config.surface_interaction->implementationName());
    LDPLAB_LOG_INFO("Ray Tracing Step CPU Factory: "\
        "Pipeline configuration uses inner particle propagation stage \"%s\"",
        config.inner_particle_propagation->implementationName());
    auto it = config.generic_geometries.begin();
    for (; it != config.generic_geometries.end(); ++it)
    {
        LDPLAB_LOG_INFO("Ray Tracing Step CPU Factory: "\
            "Pipeline configuration uses generic geometry \"%s\" for "\
            "geometry type \"%s\"",
            it->second->implementationName(),
            IParticleGeometry::typeToString(it->first));
    }
}

bool ldplab::rtscuda::Factory::createPipeline(
    const RayTracingStepCUDAInfo& info, 
    InterfaceMapping&& interface_mapping, 
    ExperimentalSetup&& setup, 
    PipelineConfiguration& pipeline_config,
    std::shared_ptr<RayTracingStepCUDA>& rts)
{
    return false;
}


#endif