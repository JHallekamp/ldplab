#include <LDPLAB/RayTracingStep/CPU/DefaultBoundingVolumeIntersectionFactories.hpp>
#include "ImplBoundingVolumeIntersection.hpp"

std::string ldplab::rtscpu::default_factories::
    BoundingSphereIntersectionBruteforceFactory::name()
{
    return "BoundingSphereIntersectionBruteforce";
}

std::string ldplab::rtscpu::default_factories::
    BoundingSphereIntersectionBruteforceFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_factories::
    BoundingSphereIntersectionBruteforceFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_factories::
    BoundingSphereIntersectionBruteforceFactory::checkCompability(
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        if (setup.particles[i].bounding_volume->type() != IBoundingVolume::Type::sphere)
            return false;
    }
    return true;
}

std::shared_ptr<ldplab::rtscpu::IBoundingVolumeIntersection> 
    ldplab::rtscpu::default_factories::BoundingSphereIntersectionBruteforceFactory::create(
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<BoundingSphereIntersectionBruteforce>();
}
