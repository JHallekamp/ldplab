#ifndef WWU_LDPLAB_RTSCUDA_DEFAULT_INNER_PRATICLE_PROPAGATION_FACTORIES_HPP
#define WWU_LDPLAB_RTSCUDA_DEFAULT_INNER_PRATICLE_PROPAGATION_FACTORIES_HPP

#include "Factories.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        namespace default_factories
        {
            class InnerParticlePropagationRK4Factory :
                public IInnerParticlePropagationFactory
            {
            public:
                InnerParticlePropagationRK4Factory(
                    const RK4Parameter& parameter);
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    const RayTracingStepCUDAInfo& step_info,
                    const ExecutionModel& execution_model,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IInnerParticlePropagation> create(
                    const RayTracingStepCUDAInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const SharedStepData& shared_data) override;
            private:
                RK4Parameter m_parameter;
            };

            class InnerParticlePropagationRK4QueueFillFactory :
                public IInnerParticlePropagationFactory
            {
            public:
                InnerParticlePropagationRK4QueueFillFactory(
                    const RK4Parameter& parameter);
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    const RayTracingStepCUDAInfo& step_info,
                    const ExecutionModel& execution_model,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IInnerParticlePropagation> create(
                    const RayTracingStepCUDAInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const SharedStepData& shared_data) override;
            private:
                RK4Parameter m_parameter;
            };
        }
    }
}

#endif