#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include <LDPLAB/RayTracingStep/CUDA/DefaultSurfaceInteractionFactories.hpp>

#include "ImplSurfaceInteraction.hpp"

std::string ldplab::rtscuda::default_factories::SurfaceInteractionFactory::
name()
{
    return "SurfaceInteraction";
}

std::string ldplab::rtscuda::default_factories::SurfaceInteractionFactory::
implementationName() const
{
    return name();
}

bool ldplab::rtscuda::default_factories::SurfaceInteractionFactory::
userDefined() const
{
    return false;
}

bool ldplab::rtscuda::default_factories::SurfaceInteractionFactory::
checkCompability(
    const RayTracingStepCUDAInfo& step_info,
    const GlobalData::DeviceProperties& device_properties,
    const PipelineConfiguration& configuration,
    const ExperimentalSetup& setup,
    const InterfaceMapping& interface_mapping)
{
    return true;
}

std::shared_ptr<ldplab::rtscuda::ISurfaceInteraction>
ldplab::rtscuda::default_factories::SurfaceInteractionFactory::
create(
    const RayTracingStepCUDAInfo& step_info,
    const PipelineConfiguration& configuration,
    const GlobalData& global_data)
{
    return std::make_shared<SurfaceInteraction>();
}

#endif