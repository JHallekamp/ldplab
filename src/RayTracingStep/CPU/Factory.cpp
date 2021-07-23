#include "Factory.hpp"

#include "../../Utils/Log.hpp"

#include <LDPLAB/RayTracingStep/CPU/StageFactories.hpp>

std::shared_ptr<ldplab::rtscpu::RayTracingStepCPU> 
    ldplab::rtscpu::Factory::createRTS(
        const RayTracingStepCPUInfo& info, 
        ExperimentalSetup&& setup)
{
    return createRTS(info, std::move(setup), PipelineConfiguration{ }, true);
}

std::shared_ptr<ldplab::rtscpu::RayTracingStepCPU> 
    ldplab::rtscpu::Factory::createRTS(
        const RayTracingStepCPUInfo& info, 
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

    // Find a viable pipeline configuration
    if (!createViableConfiguration(
        info,
        setup,
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
}

std::set<ldplab::IParticleGeometry::Type> 
    ldplab::rtscpu::Factory::getPresentGeometryTypes(
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

bool ldplab::rtscpu::Factory::combineConfigurations(
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
    if (combination.bounding_volume_intersection == nullptr)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
            "is missing a suitable bounding volume interaction factory");
        return false;
    }
    else if (combination.initial_stage == nullptr)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
            "is missing a suitable initial stage factory");
        return false;
    }
    else if (combination.inner_particle_propagation == nullptr)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
            "is missing a suitable inner particle propagation factory");
        return false;
    }
    else if (combination.particle_intersection == nullptr)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
            "is missing a suitable particle intersection factory");
        return false;
    }
    else if (combination.surface_interaction == nullptr)
    {
        LDPLAB_LOG_ERROR("Ray Tracing Step CPU Factory: Pipeline configuration "\
            "is missing a suitable surface interaction factory");
        return false;
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
            return false;
        }
    }

    return true;
}

ldplab::rtscpu::InterfaceMapping 
    ldplab::rtscpu::Factory::createInterfaceMapping(
        const ExperimentalSetup& setup)
{
    InterfaceMapping mapping;
    

    return mapping;
}

bool ldplab::rtscpu::Factory::createViableConfiguration(
    const RayTracingStepCPUInfo& info,
    const ExperimentalSetup& setup,
    std::set<IParticleGeometry::Type>& geometry_types,
    PipelineConfiguration& configuration, 
    PipelineConfiguration& default_configuration, 
    PipelineConfiguration& user_config,
    bool allow_default_stage_overwrite_on_compability_error)
{

}

ldplab::rtscpu::Factory::PipelineConfigurationBooleanState 
    ldplab::rtscpu::Factory::validateConfigurationCompability(
        const RayTracingStepCPUInfo& info,
        const ExperimentalSetup& setup,
        PipelineConfiguration& configuration)
{
    PipelineConfigurationBooleanState state;

    return state;
}

ldplab::rtscpu::Factory::PipelineConfigurationBooleanState 
    ldplab::rtscpu::Factory::checkConfigurationCast(
        std::set<IParticleGeometry::Type>& geometry_types, 
        PipelineConfiguration& configuration)
{
    PipelineConfigurationBooleanState state;
}
