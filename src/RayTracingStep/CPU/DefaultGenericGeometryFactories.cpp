#include <LDPLAB/RayTracingStep/CPU/DefaultGenericGeometryFactories.hpp>
#include "ImplGenericGeometry.hpp"

std::string ldplab::rtscpu::default_factories::
    GenericGeometryRodFactory::name()
{
    return "RodGeometry";
}

std::string ldplab::rtscpu::default_factories::
    GenericGeometryRodFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_factories::
    GenericGeometryRodFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_factories::
    GenericGeometryRodFactory::checkCompability(
        IParticleGeometry::Type geometry_type, 
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return (geometry_type == IParticleGeometry::Type::rod_particle);
}

std::shared_ptr<ldplab::rtscpu::IGenericGeometry> 
    ldplab::rtscpu::default_factories::GenericGeometryRodFactory::create(
        const std::shared_ptr<IParticleGeometry>& particle_geometry, 
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<RodGeometry>(
        static_cast<const RodParticleGeometry*>(particle_geometry.get()));
}

std::string ldplab::rtscpu::default_factories::
    GenericGeometrySphereFactory::name()
{
    return "SphericalGeometry";
}

std::string ldplab::rtscpu::default_factories::
    GenericGeometrySphereFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_factories::
    GenericGeometrySphereFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_factories::
    GenericGeometrySphereFactory::checkCompability(
        IParticleGeometry::Type geometry_type, 
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return (geometry_type == IParticleGeometry::Type::sphere);
}

std::shared_ptr<ldplab::rtscpu::IGenericGeometry> 
    ldplab::rtscpu::default_factories::GenericGeometrySphereFactory::create(
        const std::shared_ptr<IParticleGeometry>& particle_geometry, 
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<SphericalGeometry>(
        static_cast<const SphericalParticleGeometry*>(particle_geometry.get()));
}
