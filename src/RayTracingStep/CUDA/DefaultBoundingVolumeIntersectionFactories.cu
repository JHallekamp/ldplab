#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include <LDPLAB/RayTracingStep/CUDA/DefaultBoundingVolumeIntersectionFactories.hpp>

#include "ImplBoundingVolumeIntersection.hpp"

std::string ldplab::rtscuda::default_factories::BoundingSphereIntersectionBruteforceFactory::
name()
{
    return "BoundingSphereIntersectionBruteforce";
}

std::string ldplab::rtscuda::default_factories::BoundingSphereIntersectionBruteforceFactory::
implementationName() const
{
    return name();
}

bool ldplab::rtscuda::default_factories::BoundingSphereIntersectionBruteforceFactory::
userDefined() const
{
    return false;
}

bool ldplab::rtscuda::default_factories::BoundingSphereIntersectionBruteforceFactory::
checkCompability(
    const RayTracingStepCUDAInfo& step_info,
    const GlobalData::DeviceProperties& device_properties,
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

std::shared_ptr<ldplab::rtscuda::IBoundingVolumeIntersection> 
ldplab::rtscuda::default_factories::BoundingSphereIntersectionBruteforceFactory::
create(
    const RayTracingStepCUDAInfo& step_info,
    const PipelineConfiguration& configuration,
    const GlobalData& global_data)
{
    DeviceBuffer<BoundingSphere> bounding_spheres;
    if (!bounding_spheres.allocate(global_data.experimental_setup.particles.size(), true))
        return nullptr;
    return std::make_shared<BoundingSphereIntersectionBruteforce>(std::move(bounding_spheres));
}

#endif