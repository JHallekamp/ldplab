#include <LDPLAB/RayTracingStep/CPU/DefaultParticleIntersectionFactories.hpp>
#include <LDPLAB/RayTracingStep/CPU/DefaultInitialStageFactories.hpp>

#include "ImplParticleIntersection.hpp"

std::string ldplab::rtscpu::default_factories::ParticleIntersectionFactory::name()
{
    return "ParticleIntersection";
}

std::string ldplab::rtscpu::default_factories::ParticleIntersectionFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_factories::ParticleIntersectionFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_factories::ParticleIntersectionFactory::checkCompability(
    const RayTracingStepCPUInfo& step_info, 
    const PipelineConfiguration& configuration, 
    const ExperimentalSetup& setup, 
    const InterfaceMapping& interface_mapping)
{
    return true;
}

std::shared_ptr<ldplab::rtscpu::IParticleIntersection> 
    ldplab::rtscpu::default_factories::ParticleIntersectionFactory::create(
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<ParticleIntersection>();
}

std::string ldplab::rtscpu::default_factories::ParticleIntersectionPolarizedLightFactory::name()
{
    return "ParticleIntersectionPolarizedLight";
}

std::string ldplab::rtscpu::default_factories::ParticleIntersectionPolarizedLightFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_factories::ParticleIntersectionPolarizedLightFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_factories::ParticleIntersectionPolarizedLightFactory::
checkCompability(
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

std::shared_ptr <ldplab::rtscpu::IParticleIntersection> ldplab::rtscpu::default_factories::ParticleIntersectionPolarizedLightFactory::
create(
    const RayTracingStepCPUInfo& step_info,
    const PipelineConfiguration& configuration,
    const ExperimentalSetup& setup,
    const InterfaceMapping& interface_mapping)
{
    return std::make_shared<PolarizedLightParticleIntersection>();
}

bool ldplab::rtscpu::default_factories::ParticleIntersectionPolarizedLightFactory::
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
