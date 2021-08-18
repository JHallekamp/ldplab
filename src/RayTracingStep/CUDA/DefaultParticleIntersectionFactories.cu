#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include <LDPLAB/RayTracingStep/CUDA/DefaultParticleIntersectionFactories.hpp>

#include "ImplParticleIntersection.hpp"

std::string ldplab::rtscuda::default_factories::ParticleIntersectionFactory::
name()
{
    return "ParticleIntersection";
}

std::string ldplab::rtscuda::default_factories::ParticleIntersectionFactory::
implementationName() const
{
    return name();
}

bool ldplab::rtscuda::default_factories::ParticleIntersectionFactory::
userDefined() const
{
    return false;
}

bool ldplab::rtscuda::default_factories::ParticleIntersectionFactory::
checkCompability(
    const RayTracingStepCUDAInfo& step_info,
    const GlobalData::DeviceProperties& device_properties,
    const PipelineConfiguration& configuration,
    const ExperimentalSetup& setup,
    const InterfaceMapping& interface_mapping)
{
    return true;
}

std::shared_ptr<ldplab::rtscuda::IParticleIntersection>
ldplab::rtscuda::default_factories::ParticleIntersectionFactory::
create(
    const RayTracingStepCUDAInfo& step_info,
    const PipelineConfiguration& configuration,
    const GlobalData& global_data)
{
    return std::make_shared<ParticleIntersection>();
}

#endif