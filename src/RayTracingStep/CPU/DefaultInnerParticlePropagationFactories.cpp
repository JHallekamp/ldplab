#include <LDPLAB/RayTracingStep/CPU/DefaultInnerParticlePropagationFactories.hpp>
#include "ImplInnerParticlePropagation.hpp"

ldplab::rtscpu::default_stages::InnerParticlePropagationRK4Factory::
    InnerParticlePropagationRK4Factory(RK4Parameter params)
    :
    m_params{ params }
{ }

std::string ldplab::rtscpu::default_stages::
    InnerParticlePropagationRK4Factory::name()
{
    return "EikonalSolverRK4LinearIndexGradient";
}

std::string ldplab::rtscpu::default_stages::
    InnerParticlePropagationRK4Factory::implementationName() const
{
    return name();
}

bool ldplab::rtscpu::default_stages::
    InnerParticlePropagationRK4Factory::userDefined() const
{
    return false;
}

bool ldplab::rtscpu::default_stages::
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
    return true;
}

std::shared_ptr<ldplab::rtscpu::IInnerParticlePropagation> 
    ldplab::rtscpu::default_stages::InnerParticlePropagationRK4Factory::create(
        const RayTracingStepCPUInfo& step_info, 
        const PipelineConfiguration& configuration, 
        const ExperimentalSetup& setup, 
        const InterfaceMapping& interface_mapping)
{
    return std::make_shared<EikonalSolverRK4LinearIndexGradient>(m_params);
}
