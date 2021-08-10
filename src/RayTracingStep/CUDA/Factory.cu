#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "Factory.hpp"

#include "../../Utils/Log.hpp"

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>
#include <LDPLAB/RayTracingStep/CUDA/Factories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultBoundingVolumeIntersectionFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultGenericGeometryFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultGenericMaterialFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultInitialStageFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultInnerParticlePropagationFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultParticleIntersectionFactories.hpp>
#include <LDPLAB/RayTracingStep/CUDA/DefaultSurfaceInteractionFactories.hpp>

#include "PipelineDeviceBound.hpp"
#include "PipelineHostBound.hpp"
#include "StageBufferSetup.hpp"
#include "StageGatherOutput.hpp"
#include "StageRayBufferReduce.hpp"
#include "RayTracingStepCUDA.hpp"

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

    // Get present material type set
    std::set<IParticleMaterial::Type> present_material_types =
        std::move(getPresentMaterialTypes(setup));

    // Now combine user given config and default configuration
    PipelineConfiguration pipeline_configuration;
    if (!combineConfigurations(
        present_geometry_types,
        present_material_types,
        default_configuration,
        user_configuration,
        pipeline_configuration))
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create "\
            "RayTracingStepCPU");
        return nullptr;
    }

    // Create the interface mapping
    InterfaceMapping interface_mapping = createInterfaceMapping(setup);

    // Get device data
    GlobalData::DeviceProperties device_props;
    if (!getDeviceData(device_props))
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create "\
            "RayTracingStepCPU");
        return nullptr;
    }

    // Find a viable pipeline configuration
    if (!createViableConfiguration(
        info,
        setup,
        interface_mapping,
        device_props,
        present_geometry_types,
        present_material_types,
        pipeline_configuration,
        default_configuration,
        user_configuration,
        allow_default_stage_overwrite_on_compability_error))
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create "\
            "RayTracingStepCPU");
        return nullptr;
    }

    // Log viable configuration
    logViableConfiguration(pipeline_configuration);

    // Declare the ray tracing step
    std::shared_ptr<RayTracingStepCUDA> rts = 
        std::make_shared<RayTracingStepCUDA>();

    // Create pipeline
    if (!createPipeline(
        info,
        std::move(device_props),
        std::move(interface_mapping),
        std::move(setup),
        pipeline_configuration,
        rts))
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create "\
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
    default_config.generic_materials.emplace(
        IParticleMaterial::Type::linear_one_directional,
        std::make_shared<default_factories::GenericMaterialLinearOneDirectionalFactory>());
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

std::set<ldplab::IParticleMaterial::Type> 
    ldplab::rtscuda::Factory::getPresentMaterialTypes(
        const ExperimentalSetup& setup)
{
    std::set<ldplab::IParticleMaterial::Type> types;
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        if (types.find(setup.particles[i].material->type()) ==
            types.end())
        {
            types.insert(setup.particles[i].material->type());
        }
    }
    return types;
}

bool ldplab::rtscuda::Factory::combineConfigurations(
    std::set<IParticleGeometry::Type>& geometry_types, 
    std::set<IParticleMaterial::Type>& material_types,
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
    for (auto it = user_config.generic_geometries.begin();
        it != user_config.generic_geometries.end(); 
        it++)
    {
        if (it->second != nullptr ||
            geometry_types.find(it->first) != geometry_types.end())
            combination.generic_geometries.emplace(it->first, it->second);
    }
    for (auto it = user_config.generic_materials.begin();
        it != user_config.generic_materials.end();
        it++)
    {
        if (it->second != nullptr ||
            material_types.find(it->first) != material_types.end())
            combination.generic_materials.emplace(it->first, it->second);
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
    for (auto it = default_config.generic_geometries.begin();
        it != default_config.generic_geometries.end(); 
        it++)
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
    for (auto it = default_config.generic_materials.begin();
        it != default_config.generic_materials.end();
        it++)
    {
        if (it->second != nullptr &&
            material_types.find(it->first) != material_types.end())
        {
            if (combination.generic_materials.find(it->first) ==
                combination.generic_materials.end())
            {
                combination.generic_materials.emplace(it->first, it->second);
            }
        }
    }

    // Check if the combined configuration is fully cast
    bool error = false;
    if (combination.bounding_volume_intersection == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline configuration "\
            "is missing a suitable bounding volume interaction factory");
        error = true;
    }
    else if (combination.initial_stage == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline configuration "\
            "is missing a suitable initial stage factory");
        error = true;
    }
    else if (combination.inner_particle_propagation == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline configuration "\
            "is missing a suitable inner particle propagation factory");
        error = true;
    }
    else if (combination.particle_intersection == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline configuration "\
            "is missing a suitable particle intersection factory");
        error = true;
    }
    else if (combination.surface_interaction == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline configuration "\
            "is missing a suitable surface interaction factory");
        error = true;
    }
    for (auto types_it = geometry_types.begin(); 
        types_it != geometry_types.end(); 
        types_it++)
    {
        if (combination.generic_geometries.find(*types_it) ==
            combination.generic_geometries.end())
        {
            LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline configuration "\
                "is missing a suitable generic geometry factory for the "\
                "particle geometry type \"%s\"",
                IParticleGeometry::typeToString(*types_it));
            error = true;
        }
    }
    for (auto types_it = material_types.begin();
        types_it != material_types.end();
        types_it++)
    {
        if (combination.generic_materials.find(*types_it) ==
            combination.generic_materials.end())
        {
            LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline configuration "\
                "is missing a suitable generic material factory for the "\
                "particle material type \"%s\"",
                IParticleMaterial::typeToString(*types_it));
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
    const GlobalData::DeviceProperties& device_properties,
    std::set<IParticleGeometry::Type>& geometry_types, 
    std::set<IParticleMaterial::Type>& material_types,
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
            device_properties,
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
            checkConfigurationCast(geometry_types, material_types, user_config);
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
                    LDPLAB_LOG_WARNING("RTSCUDA factory: "\
                        "Swapping incompatible user defined bounding volume "\
                        "intersection \"%s\" with default \"%s\"",
                        configuration.bounding_volume_intersection->implementationName().c_str(),
                        default_configuration.bounding_volume_intersection->implementationName().c_str());
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
                    LDPLAB_LOG_WARNING("RTSCUDA factory: "\
                        "Swapping incompatible user defined initial stage "\
                        "\"%s\" with default \"%s\"",
                        configuration.initial_stage->implementationName().c_str(),
                        default_configuration.initial_stage->implementationName().c_str());
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
                    LDPLAB_LOG_WARNING("RTSCUDA factory: "\
                        "Swapping incompatible user defined inner particle "\
                        "propagation \"%s\" with default \"%s\"",
                        configuration.inner_particle_propagation->implementationName().c_str(),
                        default_configuration.inner_particle_propagation->implementationName().c_str());
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
                    LDPLAB_LOG_WARNING("RTSCUDA factory: "\
                        "Swapping incompatible user defined particle "\
                        "intersection \"%s\" with default \"%s\"",
                        configuration.particle_intersection->implementationName().c_str(),
                        default_configuration.particle_intersection->implementationName().c_str());
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
                    LDPLAB_LOG_WARNING("RTSCUDA factory: "\
                        "Swapping incompatible user defined surface "\
                        "interaction \"%s\" with default \"%s\"",
                        configuration.surface_interaction->implementationName().c_str(),
                        default_configuration.surface_interaction->implementationName().c_str());
                    configuration.surface_interaction =
                        default_configuration.surface_interaction;
                    user_defined_stages.surface_interaction_state = false;
                }
            }

            for (auto cs_it = config_state.generic_geometry_state.begin(); 
                cs_it != config_state.generic_geometry_state.end(); 
                cs_it++)
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
                    LDPLAB_LOG_WARNING("RTSCUDA factory: "\
                        "Swapping incompatible generic geometry \"%s\" with "\
                        "default implementation \"%s\"",
                        config_stage->second->implementationName().c_str(),
                        default_stage->second->implementationName().c_str());
                    config_stage->second = default_stage->second;
                    ud_geo_state->second = false;
                }
            }

            for (auto cs_it = config_state.generic_material_state.begin();
                cs_it != config_state.generic_material_state.end();
                cs_it++)
            {
                auto ud_mat_state =
                    user_defined_stages.generic_material_state.find(cs_it->first);
                if (ud_mat_state == user_defined_stages.generic_material_state.end() ||
                    ud_mat_state->second == false)
                    break;
                else
                {
                    auto config_stage =
                        configuration.generic_materials.find(cs_it->first);
                    auto default_stage =
                        default_configuration.generic_materials.find(cs_it->first);
                    if (config_stage == configuration.generic_materials.end() ||
                        default_stage == default_configuration.generic_materials.end())
                        break;
                    LDPLAB_LOG_WARNING("RTSCUDA factory: "\
                        "Swapping incompatible generic material \"%s\" with "\
                        "default implementation \"%s\"",
                        config_stage->second->implementationName().c_str(),
                        default_stage->second->implementationName().c_str());
                    config_stage->second = default_stage->second;
                    ud_mat_state->second = false;
                }
            }

            // Check configuration
            config_state = validateConfigurationCompability(
                info,
                setup,
                interface_mapping,
                device_properties,
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
        LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline bounding "\
            "volume intersection stage \"%s\" is incompatible with the given "\
            "configuration or experimental setup",
            configuration.bounding_volume_intersection->implementationName().c_str());
    }
    if (!config_state.initial_stage_state)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline initial "\
            "stage \"%s\" is incompatible with the given configuration or "\
            "experimental setup",
            configuration.initial_stage->implementationName().c_str());
    }
    if (!config_state.inner_particle_propagation_state)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline inner "\
            "particle propagation stage \"%s\" is incompatible with the given "\
            "configuration or experimental setup",
            configuration.inner_particle_propagation->implementationName().c_str());
    }
    if (!config_state.particle_intersection_state)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline particle "\
            "intersection stage \"%s\" is incompatible with the given "\
            "configuration or experimental setup",
            configuration.particle_intersection->implementationName().c_str());
    }
    if (!config_state.surface_interaction_state)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Pipeline surface "\
            "interaction stage \"%s\" is incompatible with the given "\
            "configuration or experimental setup",
            configuration.surface_interaction->implementationName().c_str());
    }
    for (auto it = config_state.generic_geometry_state.begin();
        it != config_state.generic_geometry_state.end();
        it++)
    {
        if (!it->second)
        {
            auto geo = configuration.generic_geometries.find(it->first);
            LDPLAB_LOG_ERROR("RTSCUDA factory: Generic "\
                "geometry implementation \"%s\" for geometry type \"%s\" is "\
                "incompatible with the given configuration or "\
                "experimental setup",
                geo->second->implementationName().c_str(),
                IParticleGeometry::typeToString(it->first));
        }
    }
    for (auto it = config_state.generic_material_state.begin();
        it != config_state.generic_material_state.end();
        it++)
    {
        if (!it->second)
        {
            auto geo = configuration.generic_materials.find(it->first);
            LDPLAB_LOG_ERROR("RTSCUDA factory: Generic "\
                "material implementation \"%s\" for material type \"%s\" is "\
                "incompatible with the given configuration or "\
                "experimental setup",
                geo->second->implementationName().c_str(),
                IParticleMaterial::typeToString(it->first));
        }
    }
    return false;
}

ldplab::rtscuda::Factory::PipelineConfigurationBooleanState 
    ldplab::rtscuda::Factory::validateConfigurationCompability(
        const RayTracingStepCUDAInfo& info, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping, 
        const GlobalData::DeviceProperties& device_properties,
        PipelineConfiguration& configuration)
{
    PipelineConfigurationBooleanState state;
    state.bounding_volume_intersection_state =
        configuration.bounding_volume_intersection->checkCompability(
            info,
            device_properties,
            configuration,
            setup,
            interface_mapping);
    state.initial_stage_state =
        configuration.initial_stage->checkCompability(
            info,
            device_properties,
            configuration,
            setup,
            interface_mapping);
    state.inner_particle_propagation_state =
        configuration.inner_particle_propagation->checkCompability(
            info,
            device_properties,
            configuration,
            setup,
            interface_mapping);
    state.particle_intersection_state =
        configuration.particle_intersection->checkCompability(
            info,
            device_properties,
            configuration,
            setup,
            interface_mapping);
    state.surface_interaction_state =
        configuration.surface_interaction->checkCompability(
            info,
            device_properties,
            configuration,
            setup,
            interface_mapping);
    auto it = configuration.generic_geometries.begin();
    for (; it != configuration.generic_geometries.end(); ++it)
    {
        state.generic_geometry_state.emplace(it->first,
            it->second->checkCompability(
                it->first,
                info,
                device_properties,
                configuration,
                setup,
                interface_mapping));
    }
    return state;
}

ldplab::rtscuda::Factory::PipelineConfigurationBooleanState 
    ldplab::rtscuda::Factory::checkConfigurationCast(
        std::set<IParticleGeometry::Type>& geometry_types, 
        std::set<IParticleMaterial::Type>& material_types,
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
    for (auto it = material_types.begin(); it != material_types.end(); ++it)
    {
        auto mat = configuration.generic_materials.find(*it);
        if (mat == configuration.generic_materials.end())
            state.generic_material_state.emplace(*it, false);
        else
            state.generic_material_state.emplace(*it, mat->second != nullptr);
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
    for (auto it = configuration_state.generic_material_state.begin();
        it != configuration_state.generic_material_state.end();
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
    LDPLAB_LOG_INFO("RTSCUDA factory: "\
        "Pipeline configuration uses initial stage \"%s\"",
        config.initial_stage->implementationName().c_str());
    LDPLAB_LOG_INFO("RTSCUDA factory: "\
        "Pipeline configuration uses bounding volume intersection stage \"%s\"",
        config.bounding_volume_intersection->implementationName().c_str());
    LDPLAB_LOG_INFO("RTSCUDA factory: "\
        "Pipeline configuration uses particle intersection stage \"%s\"",
        config.particle_intersection->implementationName().c_str());
    LDPLAB_LOG_INFO("RTSCUDA factory: "\
        "Pipeline configuration uses surface interaction stage \"%s\"",
        config.surface_interaction->implementationName().c_str());
    LDPLAB_LOG_INFO("RTSCUDA factory: "\
        "Pipeline configuration uses inner particle propagation stage \"%s\"",
        config.inner_particle_propagation->implementationName().c_str());
    auto git = config.generic_geometries.begin();
    for (; git != config.generic_geometries.end(); ++git)
    {
        LDPLAB_LOG_INFO("RTSCUDA factory: "\
            "Pipeline configuration uses generic geometry \"%s\" for "\
            "geometry type \"%s\"",
            git->second->implementationName().c_str(),
            IParticleGeometry::typeToString(git->first));
    }
    auto mit = config.generic_materials.begin();
    for (; mit != config.generic_materials.end(); ++mit)
    {
        LDPLAB_LOG_INFO("RTSCUDA factory: "\
            "Pipeline configuration uses generic material \"%s\" for "\
            "geometry type \"%s\"",
            mit->second->implementationName().c_str(),
            IParticleMaterial::typeToString(mit->first));
    }
}

bool ldplab::rtscuda::Factory::getDeviceData(
    GlobalData::DeviceProperties& device_props)
{
    int device_count = 0;
    cudaError_t error_id = cudaGetDeviceCount(&device_count);

    if (error_id != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to get cuda device count, "\
            "cudaGetDeviceCount returned error code %i: %s",
            error_id,
            cudaGetErrorString(error_id));
        return false;
    }

    if (device_count == 0)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to find any cuda devices.");
        return false;
    }

    cudaSetDevice(0);
    cudaDeviceProp device_prop;
    cudaGetDeviceProperties(&device_prop, 0);
    LDPLAB_LOG_INFO("RTSCUDA factory: Using cuda device %s", device_prop.name);

    device_props.max_block_size.x = device_prop.maxThreadsDim[0];
    device_props.max_block_size.y = device_prop.maxThreadsDim[1];
    device_props.max_block_size.z = device_prop.maxThreadsDim[2];
    device_props.max_grid_size.x = device_prop.maxGridSize[0];
    device_props.max_grid_size.y = device_prop.maxGridSize[1];
    device_props.max_grid_size.z = device_prop.maxGridSize[2];
    device_props.max_num_threads_per_block = device_prop.maxThreadsPerBlock;
    device_props.max_num_threads_per_mp = device_prop.maxThreadsPerMultiProcessor;
    device_props.num_mps = device_prop.multiProcessorCount;
    device_props.registers_per_block = device_prop.regsPerBlock;
    device_props.shared_mem_per_block = device_prop.sharedMemPerBlock;
    device_props.shared_mem_per_mp = device_prop.sharedMemPerMultiprocessor;
    device_props.warp_size = device_prop.warpSize;

    return true;
}

bool ldplab::rtscuda::Factory::createGlobalData(
    const RayTracingStepCUDAInfo& info, 
    PipelineConfiguration& pipeline_config,
    ExperimentalSetup&& p_setup, 
    InterfaceMapping&& p_interface_mapping, 
    GlobalData::DeviceProperties&& p_device_properties, 
    std::unique_ptr<GlobalData>& global_data)
{
    // Create the instance
    global_data = std::make_unique<GlobalData>();

    // Move interface mapping and device properties
    global_data->experimental_setup = std::move(p_setup);
    global_data->interface_mapping = std::move(p_interface_mapping);
    global_data->device_properties = std::move(p_device_properties);

    // Helper for often used names in the following
    const ExperimentalSetup& setup = global_data->experimental_setup;
    const InterfaceMapping& interface_mapping = global_data->interface_mapping;

    // Set simulation parameter
    global_data->simulation_parameter.intensity_cutoff =
        info.intensity_cutoff;
    global_data->simulation_parameter.max_branching_depth = 
        info.maximum_branching_depth;
    global_data->simulation_parameter.num_particles = 
        setup.particles.size();
    global_data->simulation_parameter.num_rays_per_batch = 
        info.number_rays_per_batch;
    global_data->simulation_parameter.num_parallel_batches =
        info.number_parallel_batches;
    global_data->simulation_parameter.num_surface_interaction_reflection_passes = 
        info.number_reflections;
    global_data->simulation_parameter.num_surface_interaction_transmission_passes = 
        info.number_transmissions;
    global_data->simulation_parameter.ray_world_space_index = 
        global_data->simulation_parameter.num_particles;
    global_data->simulation_parameter.ray_invalid_index = -1;
    global_data->simulation_parameter.output_in_particle_space =
        info.return_force_in_particle_coordinate_system;

    // Create generic geometries and materials
    bool error = false;
    std::map<const IParticleGeometry*, size_t> reusable_geometry;
    std::map<const IParticleMaterial*, size_t> reusable_material;
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        auto reusable_geometry_index_it = reusable_geometry.find(
            setup.particles[i].geometry.get());
        if (reusable_geometry_index_it == reusable_geometry.end())
        {
            // No reusable geometry present
            auto geo_factory_it = pipeline_config.generic_geometries.find(
                setup.particles[i].geometry->type());
            if (geo_factory_it == pipeline_config.generic_geometries.end())
            {
                LDPLAB_LOG_ERROR("RTSCUDA factory: "\
                    "Could not find generic geometry factory for particle "\
                    "type \"%s\" in the given particle configuration",
                    IParticleGeometry::typeToString(
                        setup.particles[i].geometry->type()));
                error = true;
            }
            else
            {
                std::shared_ptr<IGenericGeometry> generic_geometry =
                    geo_factory_it->second->create(
                        setup.particles[i].geometry,
                        info,
                        global_data->device_properties,
                        pipeline_config,
                        setup,
                        interface_mapping);
                if (generic_geometry == nullptr)
                {
                    LDPLAB_LOG_ERROR("RTSCUDA factory: "\
                        "Could not create generic geometry for particle %i "\
                        "of type \"%s\"",
                        setup.particles[i].uid,
                        IParticleGeometry::typeToString(
                            setup.particles[i].geometry->type()));
                    error = true;
                }
                else
                {
                    reusable_geometry.emplace(
                        setup.particles[i].geometry.get(),
                        global_data->particle_data_buffers.geometry_instances.size());
                    global_data->particle_data_buffers.geometry_instances.
                        emplace_back(std::move(generic_geometry));
                }
            }
        }
        else
        {
            // Reuse geometry
            global_data->particle_data_buffers.geometry_instances.emplace_back(
                global_data->particle_data_buffers.geometry_instances[
                    reusable_geometry_index_it->second]);
        }

        auto reusable_material_index_it = reusable_material.find(
            setup.particles[i].material.get());
        if (reusable_material_index_it == reusable_material.end())
        {
            // No reusable material present
            auto mat_factory_it = pipeline_config.generic_materials.find(
                setup.particles[i].material->type());
            if (mat_factory_it == pipeline_config.generic_materials.end())
            {
                LDPLAB_LOG_ERROR("RTSCUDA factory: "\
                    "Could not find generic material factory for particle "\
                    "type \"%s\" in the given particle configuration",
                    IParticleMaterial::typeToString(
                        setup.particles[i].material->type()));
                error = true;
            }
            else
            {
                std::shared_ptr<IGenericMaterial> generic_material =
                    mat_factory_it->second->create(
                        setup.particles[i].material,
                        info,
                        global_data->device_properties,
                        pipeline_config,
                        setup,
                        interface_mapping);
                if (generic_material == nullptr)
                {
                    LDPLAB_LOG_ERROR("RTSCUDA factory: "\
                        "Could not create generic material for particle %i "\
                        "of type \"%s\"",
                        setup.particles[i].uid,
                        IParticleMaterial::typeToString(
                            setup.particles[i].material->type()));
                    error = true;
                }
                else
                {
                    reusable_material.emplace(
                        setup.particles[i].material.get(),
                        global_data->particle_data_buffers.material_instances.size());
                    global_data->particle_data_buffers.material_instances.
                        emplace_back(std::move(generic_material));
                }
            }
        }
        else
        {
            // Reuse material
            global_data->particle_data_buffers.material_instances.emplace_back(
                global_data->particle_data_buffers.material_instances[
                    reusable_material_index_it->second]);
        }
    }

    if (error)
        return false;

    // Allocate particle data
    auto& pd = global_data->particle_data_buffers;
    
    const size_t num_particles = setup.particles.size();
    error = error || !pd.p2w_transformation_buffer.allocate(num_particles, true);
    error = error || !pd.p2w_translation_buffer.allocate(num_particles, true);
    error = error || !pd.w2p_transformation_buffer.allocate(num_particles, true);
    error = error || !pd.w2p_translation_buffer.allocate(num_particles, true);
    if (error)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create particle "\
            "transformation device buffers.");
        return false;
    }

    error = error || !pd.geometry_data_buffer.allocate(num_particles, true);
    error = error || !pd.intersect_ray_fptr_buffer.allocate(num_particles, true);
    error = error || !pd.intersect_segment_fptr_buffer.allocate(num_particles, true);
    error = error || !pd.index_of_refraction_fptr_buffer.allocate(num_particles, true);
    error = error || !pd.material_data_buffer.allocate(num_particles, true);
    if (error)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to create particle "\
            "material and geometry device buffers.");
        return false;
    }

    for (size_t i = 0; i < num_particles; ++i)
    {
        pd.geometry_data_buffer.getHostBuffer()[i] =
            pd.geometry_instances[i]->getDeviceData();
        pd.intersect_ray_fptr_buffer.getHostBuffer()[i] =
            pd.geometry_instances[i]->getDeviceIntersectRayFunction();
        pd.intersect_segment_fptr_buffer.getHostBuffer()[i] =
            pd.geometry_instances[i]->getDeviceIntersectSegmentFunction();
        pd.index_of_refraction_fptr_buffer.getHostBuffer()[i] =
            pd.material_instances[i]->getDeviceIndexOfRefractionFunction();
        pd.material_data_buffer.getHostBuffer()[i] =
            pd.material_instances[i]->getDeviceData();
        error = error || (pd.geometry_data_buffer.getHostBuffer()[i] == nullptr);
        error = error || (pd.intersect_ray_fptr_buffer.getHostBuffer()[i] == nullptr);
        error = error || (pd.intersect_segment_fptr_buffer.getHostBuffer()[i] == nullptr);
        error = error || (pd.index_of_refraction_fptr_buffer.getHostBuffer()[i] == nullptr);
        error = error || (pd.material_data_buffer.getHostBuffer()[i] == nullptr);
    }
    if (error)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to receive a device function "\
            "pointer for generic geometry or material functions.");
        return false;
    }

    error = error || !pd.geometry_data_buffer.upload();
    error = error || !pd.intersect_ray_fptr_buffer.upload();
    error = error || !pd.intersect_segment_fptr_buffer.upload();
    error = error || !pd.index_of_refraction_fptr_buffer.upload();
    error = error || !pd.material_data_buffer.upload();
    if (error)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to upload buffers for "\
            "generic geometry or material data.");
        return false;
    }

    if (!pd.center_of_mass_buffer.allocate(num_particles, true))
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to allocate "\
            "particle center of mass device buffer.");
        return false;
    }
    
    for (size_t i = 0; i < num_particles; ++i)
    {
        pd.center_of_mass_buffer.getHostBuffer()[i] =
            setup.particles[i].centre_of_mass;
    }

    if (!pd.center_of_mass_buffer.upload())
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to upload "\
            "particle center of mass device buffer.");
        return false;
    }

    // Create batches
    const size_t num_rays = 
        global_data->simulation_parameter.num_rays_per_batch;
    const size_t branching_depth = 
        global_data->simulation_parameter.max_branching_depth;
    for (size_t i = 0; i < global_data->simulation_parameter.num_parallel_batches; ++i)
    {
        global_data->batch_data.emplace_back();
        auto& bd = global_data->batch_data.back();
        bd.batch_data_index = i;

        // Create ray buffer
        error = error || !bd.ray_data_buffers.direction_buffers.allocate(
            num_rays, branching_depth + 2, 0);
        error = error || !bd.ray_data_buffers.intensity_buffers.allocate(
            num_rays, branching_depth + 2, 0);
        error = error || !bd.ray_data_buffers.min_bv_distance_buffers.allocate(
            num_rays, branching_depth + 2, 0);
        error = error || !bd.ray_data_buffers.origin_buffers.allocate(
            num_rays, branching_depth + 2, 0);
        error = error || !bd.ray_data_buffers.particle_index_buffers.allocate(
            num_rays, branching_depth + 2, 0);
        if (error)
        {
            LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to allocate ray data "\
                "device buffers.");
            return false;
        }

        // Create intersection buffer
        error = error || !bd.intersection_data_buffers.normal_buffers.allocate(
            num_rays, branching_depth + 1, 0);
        error = error || !bd.intersection_data_buffers.particle_index_buffers.allocate(
            num_rays, branching_depth + 1, 0);
        error = error || !bd.intersection_data_buffers.point_buffers.allocate(
            num_rays, branching_depth + 1, 0);
        if (error)
        {
            LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to allocate intersection "\
                "data device buffers.");
            return false;
        }

        // Create output buffer
        error = error || !bd.output_data_buffers.force_per_particle_buffer.allocate(
            num_particles, true);
        error = error || !bd.output_data_buffers.torque_per_particle_buffer.allocate(
            num_particles, true);
        error = error || !bd.output_data_buffers.force_per_ray_buffer.allocate(
            num_rays, branching_depth + 1, 0);
        error = error || !bd.output_data_buffers.torque_per_ray_buffer.allocate(
            num_rays, branching_depth + 1, 0);
        if (error)
        {
            LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to allocate output "\
                "data device buffers.");
            return false;
        }
    }

    return true;
}

bool ldplab::rtscuda::Factory::createPipeline(
    const RayTracingStepCUDAInfo& info, 
    GlobalData::DeviceProperties&& device_properties,
    InterfaceMapping&& interface_mapping, 
    ExperimentalSetup&& setup, 
    PipelineConfiguration& pipeline_config,
    std::shared_ptr<RayTracingStepCUDA>& rts)
{
    std::unique_ptr<GlobalData> global_data;
    if (!createGlobalData(
        info,
        pipeline_config,
        std::move(setup),
        std::move(interface_mapping),
        std::move(device_properties),
        global_data))
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: "\
            "Failed to allocate pipeline device and host data");
        return false;
    }

    // Create the pipeline stage instances
    bool error = false;

    std::shared_ptr<IInitialStage> stage_is =
        pipeline_config.initial_stage->create(
            info,
            pipeline_config,
            *global_data);
    if (stage_is == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: "\
            "Failed to create initial stage");
        error = true;
    }
    else
        stage_is->m_parent_rts_uid = rts->uid();

    std::shared_ptr<IBoundingVolumeIntersection> stage_bvi =
        pipeline_config.bounding_volume_intersection->create(
            info,
            pipeline_config,
            *global_data);
    if (stage_bvi == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: "\
            "Failed to create bounding volume intersection");
        error = true;
    }
    else
        stage_bvi->m_parent_rts_uid = rts->uid();

    std::shared_ptr<IParticleIntersection> stage_pi =
        pipeline_config.particle_intersection->create(
            info,
            pipeline_config,
            *global_data);
    if (stage_pi == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: "\
            "Failed to create particle intersection");
        error = true;
    }
    else
        stage_pi->m_parent_rts_uid = rts->uid();

    std::shared_ptr<ISurfaceInteraction> stage_si =
        pipeline_config.surface_interaction->create(
            info,
            pipeline_config,
            *global_data);
    if (stage_si == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: "\
            "Failed to create surface interaction");
        error = true;
    }
    else
        stage_si->m_parent_rts_uid = rts->uid();

    std::shared_ptr<IInnerParticlePropagation> stage_ipp =
        pipeline_config.inner_particle_propagation->create(
            info,
            pipeline_config,
            *global_data);
    if (stage_ipp == nullptr)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: "\
            "Failed to create inner particle propagation");
        error = true;
    }
    else
        stage_ipp->m_parent_rts_uid = rts->uid();

    if (error)
        return false;

    std::unique_ptr<IPipeline> pipeline;
    if (info.host_bound_pipeline)
    {
        std::shared_ptr<utils::ThreadPool> thread_pool =
            std::make_shared<utils::ThreadPool>(info.number_parallel_batches);
        pipeline = std::make_unique<PipelineHostBound>(thread_pool);
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: "\
            "Device bound pipeline is not implemented yet.");
        return false;
    }

    // Move context and stages to the pipeline
    pipeline->m_context = std::move(global_data);
    pipeline->m_stage_bvi = std::move(stage_bvi);
    pipeline->m_stage_is = std::move(stage_is);
    pipeline->m_stage_ipp = std::move(stage_ipp);
    pipeline->m_stage_pi = std::move(stage_pi);
    pipeline->m_stage_si = std::move(stage_si);
    
    // Allocate pipeline data
    for (size_t i = 0; i < pipeline->m_context->batch_data.size(); ++i)
    {
        pipeline->m_pipeline_data.emplace_back();
        if (!BufferSetup::allocateData(
            *pipeline->m_context,
            pipeline->m_pipeline_data.back()))
        {
            LDPLAB_LOG_ERROR("RTSCUDA factory: "\
                "Failed to allocate buffer setup stage pipeline data.");
            return false;
        }
        if (!GatherOutput::allocateData(
            *pipeline->m_context,
            pipeline->m_pipeline_data.back()))
        {
            LDPLAB_LOG_ERROR("RTSCUDA factory: "\
                "Failed to allocate gather output stage pipeline data.");
            return false;
        }
        if (!RayBufferReduce::allocateData(
            *pipeline->m_context,
            pipeline->m_pipeline_data.back()))
        {
            LDPLAB_LOG_ERROR("RTSCUDA factory: "\
                "Failed to allocate ray buffer reduction stage pipeline data.");
            return false;
        }
    }

    rts->m_pipeline = std::move(pipeline);
    return true;
}


#endif