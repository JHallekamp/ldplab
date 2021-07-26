#include <LDPLAB/RayTracingStep/CPU/DefaultGenericGeometryFactories.hpp>
#include "ImplGenericGeometry.hpp"

std::string ldplab::rtscpu::default_generic_geometry::
    GenericGeometryRodFactory::name()
{
    return "RodGeometry";
}

std::string ldplab::rtscpu::default_generic_geometry::
    GenericGeometryRodFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_generic_geometry::
    GenericGeometryRodFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_generic_geometry::
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
    ldplab::rtscpu::default_generic_geometry::GenericGeometryRodFactory::create(
        const std::shared_ptr<IParticleGeometry>& particle_geometry, 
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<RodGeometry>(
        static_cast<const RodParticleGeometry*>(particle_geometry.get()));
}

std::string ldplab::rtscpu::default_generic_geometry::
    GenericGeometrySphereFactory::name()
{
    return "SphericalGeometry";
}

std::string ldplab::rtscpu::default_generic_geometry::
    GenericGeometrySphereFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_generic_geometry::
    GenericGeometrySphereFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_generic_geometry::
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
    ldplab::rtscpu::default_generic_geometry::GenericGeometrySphereFactory::create(
        const std::shared_ptr<IParticleGeometry>& particle_geometry, 
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<SphericalGeometry>(
        static_cast<const SphericalParticleGeometry*>(particle_geometry.get()));
}
