#include <LDPLAB/RayTracingStep/CPU/DefaultInnerParticlePropagationFactories.hpp>
#include <LDPLAB/RayTracingStep/CPU/DefaultInitialStageFactories.hpp>
#include "ImplInnerParticlePropagation.hpp"

ldplab::rtscpu::default_factories::InnerParticlePropagationRK4Factory::
    InnerParticlePropagationRK4Factory(RK4Parameter params)
    :
    m_params{ params }
{ }

std::string ldplab::rtscpu::default_factories::
    InnerParticlePropagationRK4Factory::name()
{
    return "EikonalSolverRK4LinearIndexGradient";
}

std::string ldplab::rtscpu::default_factories::
    InnerParticlePropagationRK4Factory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_factories::
    InnerParticlePropagationRK4Factory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_factories::
    InnerParticlePropagationRK4Factory::checkCompability(
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        if (setup.particles[i].material->type() !=
            IParticleMaterial::Type::linear_one_directional)
        {
            return false;
        }
    }
    for (size_t i = 0; i < setup.light_sources.size(); ++i)
    {
        if (setup.light_sources[i].polarization->type() !=
            ILightPolarisation::Type::unpolarized)
        {
            return false;
        }
    }
    return true;
}

std::shared_ptr<ldplab::rtscpu::IInnerParticlePropagation> 
    ldplab::rtscpu::default_factories::InnerParticlePropagationRK4Factory::create(
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<EikonalSolverRK4LinearIndexGradient>(m_params);
}

ldplab::rtscpu::default_factories::InnerParticlePropagationRK4PolarizationFactory::InnerParticlePropagationRK4PolarizationFactory(RK4Parameter params)
    :
    m_params{ params }
{
}

std::string ldplab::rtscpu::default_factories::InnerParticlePropagationRK4PolarizationFactory::name() 
{
    return "EikonalSolverRK4LinearIndexGradientPolarization";
}

std::string ldplab::rtscpu::default_factories::InnerParticlePropagationRK4PolarizationFactory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_factories::InnerParticlePropagationRK4PolarizationFactory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_factories::InnerParticlePropagationRK4PolarizationFactory::checkCompability(const RayTracingStepCPUInfo& step_info, const PipelineConfiguration& configuration, const ExperimentalSetup& setup, const InterfaceMapping& interface_mapping)
{
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        if (setup.particles[i].material->type() !=
            IParticleMaterial::Type::linear_one_directional)
        {
            return false;
        }
    }
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

std::shared_ptr<ldplab::rtscpu::IInnerParticlePropagation> ldplab::rtscpu::default_factories::InnerParticlePropagationRK4PolarizationFactory::create(
    const RayTracingStepCPUInfo& step_info, 
    const PipelineConfiguration& configuration,
    const ExperimentalSetup& setup,
    const InterfaceMapping& interface_mapping)
{
    return std::make_shared<EikonalSolverRK4LinearIndexGradientPolarization>(m_params);
}


bool ldplab::rtscpu::default_factories::InnerParticlePropagationRK4PolarizationFactory::
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
