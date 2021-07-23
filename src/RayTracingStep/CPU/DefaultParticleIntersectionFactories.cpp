#include <LDPLAB/RayTracingStep/CPU/DefaultParticleIntersectionFactories.hpp>

#include "ImplParticleIntersection.hpp"

std::string ldplab::rtscpu::default_stages::ParticleIntersectionFactory::name()
{
    return "ParticleIntersection";
}

std::string ldplab::rtscpu::default_stages::ParticleIntersectionFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_stages::ParticleIntersectionFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_stages::ParticleIntersectionFactory::checkCompability(
    const RayTracingStepCPUInfo& step_info, 
    const PipelineConfiguration& configuration, 
    const ExperimentalSetup& setup, 
    const InterfaceMapping& interface_mapping)
{
    return true;
}

std::shared_ptr<ldplab::rtscpu::IParticleIntersection> 
    ldplab::rtscpu::default_stages::ParticleIntersectionFactory::create(
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<ParticleIntersection>();
}
