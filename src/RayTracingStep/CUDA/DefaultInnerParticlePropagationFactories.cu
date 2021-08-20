#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include <LDPLAB/RayTracingStep/CUDA/DefaultInnerParticlePropagationFactories.hpp>

#include "ImplInnerParticlePropagation.hpp"

ldplab::rtscuda::default_factories::InnerParticlePropagationRK4Factory::
InnerParticlePropagationRK4Factory(
    const RK4Parameter& parameter)
    :
    m_parameter{ parameter }
{ }

std::string ldplab::rtscuda::default_factories::InnerParticlePropagationRK4Factory::
name()
{
    return "InnerParticlePropagationRK4";
}

std::string ldplab::rtscuda::default_factories::InnerParticlePropagationRK4Factory::
implementationName() const
{
    return name();
}

bool ldplab::rtscuda::default_factories::InnerParticlePropagationRK4Factory::
userDefined() const
{
    return false;
}

bool ldplab::rtscuda::default_factories::InnerParticlePropagationRK4Factory::
checkCompability(
    const RayTracingStepCUDAInfo& step_info,
    const ExecutionModel& execution_model,
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

std::shared_ptr<ldplab::rtscuda::IInnerParticlePropagation>
ldplab::rtscuda::default_factories::InnerParticlePropagationRK4Factory::
create(
    const RayTracingStepCUDAInfo& step_info,
    const PipelineConfiguration& configuration,
    const SharedStepData& shared_data)
{
    return std::make_shared<InnerParticlePropagationRK4>(m_parameter);
}

#endif