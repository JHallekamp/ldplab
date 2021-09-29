#include <LDPLAB/RayTracingStep/CPU/DefaultSurfaceInteractionFactories.hpp>
#include <LDPLAB/RayTracingStep/CPU/DefaultInitialStageFactories.hpp>
#include "ImplSurfaceInteraction.hpp"

std::string ldplab::rtscpu::default_factories::SurfaceInteractionFactory::name()
{
    return "SurfaceInteraction";
}

std::string ldplab::rtscpu::default_factories::SurfaceInteractionFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_factories::SurfaceInteractionFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_factories::SurfaceInteractionFactory::checkCompability(
    const RayTracingStepCPUInfo& step_info, 
    const PipelineConfiguration& configuration, 
    const ExperimentalSetup& setup, 
    const InterfaceMapping& interface_mapping)
{
    for (size_t i = 0; i < setup.light_sources.size(); ++i)
    {
        if (setup.light_sources[i].polarization->type() !=
            ILightPolarisation::Type::unpolarized)
        {
            return false;
        }
    }
    return true;
}

std::shared_ptr<ldplab::rtscpu::ISurfaceInteraction> 
    ldplab::rtscpu::default_factories::SurfaceInteractionFactory::create(
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<SurfaceInteraction>();
}

std::string ldplab::rtscpu::default_factories::SurfaceInteractionPolarizedLightFactory::name()
{
    return "SurfacePolarizedLightInteraction";
}

std::string ldplab::rtscpu::default_factories::SurfaceInteractionPolarizedLightFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_factories::SurfaceInteractionPolarizedLightFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_factories::SurfaceInteractionPolarizedLightFactory::checkCompability(
    const RayTracingStepCPUInfo& step_info,
    const PipelineConfiguration& configuration,
    const ExperimentalSetup& setup,
    const InterfaceMapping& interface_mapping)
{
    for (size_t i = 0; i < setup.light_sources.size(); ++i)
    {
        if (setup.light_sources[i].polarization->type() !=
            ILightPolarisation::Type::polarized)
        {
            return false;
        }
    }
    return true;
}

std::shared_ptr<ldplab::rtscpu::ISurfaceInteraction>
ldplab::rtscpu::default_factories::SurfaceInteractionPolarizedLightFactory::create(
    const RayTracingStepCPUInfo& step_info,
    const PipelineConfiguration& configuration,
    const ExperimentalSetup& setup,
    const InterfaceMapping& interface_mapping)
{
    return std::make_shared<SurfaceInteractionPolarizedLight>();
}

bool ldplab::rtscpu::default_factories::SurfaceInteractionPolarizedLightFactory::
    createStageDependentData(
        const MemoryInfo& memory_info, 
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping, 
        std::shared_ptr<void>& stage_dependent_data)
{
    auto init_stage = (InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory*)
        configuration.initial_stage.get();
    stage_dependent_data = init_stage->getPolatizationData(memory_info, 
        step_info);
    return true;
}
