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

ldplab::rtscuda::default_factories::InnerParticlePropagationRK4QueueFillFactory::
InnerParticlePropagationRK4QueueFillFactory(
    const RK4Parameter& parameter)
    :
    m_parameter{ parameter }
{ }

std::string ldplab::rtscuda::default_factories::InnerParticlePropagationRK4QueueFillFactory::
name()
{
    return "InnerParticlePropagationRK4QueueFill";
}

std::string ldplab::rtscuda::default_factories::InnerParticlePropagationRK4QueueFillFactory::
implementationName() const
{
    return name();
}

bool ldplab::rtscuda::default_factories::InnerParticlePropagationRK4QueueFillFactory::
userDefined() const
{
    return false;
}

bool ldplab::rtscuda::default_factories::InnerParticlePropagationRK4QueueFillFactory::
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
ldplab::rtscuda::default_factories::InnerParticlePropagationRK4QueueFillFactory::
create(
    const RayTracingStepCUDAInfo& step_info,
    const PipelineConfiguration& configuration,
    const SharedStepData& shared_data)
{
    const auto& streams = shared_data.execution_model.stream_contexts;
    std::vector<DeviceBufferPinned<uint32_t>> queue_ctr_per_stream(streams.size());
    for (size_t i = 0; i < streams.size(); ++i)
    {
        streams[i].deviceContext().activateDevice();
        if (!queue_ctr_per_stream[i].allocate(1, true))
            return nullptr;
    }
    return std::make_shared<InnerParticlePropagationRK4QueueFill>(
        m_parameter, 
        std::move(queue_ctr_per_stream));
}

#endif