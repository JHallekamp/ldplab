#include <LDPLAB/RayTracingStep/CPU/DefaultSurfaceInteractionFactories.hpp>
#include "ImplSurfaceInteraction.hpp"

std::string ldplab::rtscpu::default_stages::SurfaceInteractionFactory::name()
{
    return "SurfaceInteraction";
}

std::string ldplab::rtscpu::default_stages::SurfaceInteractionFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_stages::SurfaceInteractionFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_stages::SurfaceInteractionFactory::checkCompability(
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
    ldplab::rtscpu::default_stages::SurfaceInteractionFactory::create(
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<SurfaceInteraction>();
}
