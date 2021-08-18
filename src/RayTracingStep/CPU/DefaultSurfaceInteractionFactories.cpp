#include <LDPLAB/RayTracingStep/CPU/DefaultSurfaceInteractionFactories.hpp>
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
        if (setup.light_sources[i].polarisation->type() !=
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
