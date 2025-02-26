#ifndef WWU_LDPLAB_RTSCPU_DEFAULT_INNER_PARTICLE_PROPAGATION_FACTORIES_HPP
#define WWU_LDPLAB_RTSCPU_DEFAULT_INNER_PARTICLE_PROPAGATION_FACTORIES_HPP

#include "StageFactories.hpp"

#include <LDPLAB/RayTracingStep/EikonalSolverParameter.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        namespace default_factories
        {
            /** @brief Uses RK4 method to solve inner particle propagation. */
            class InnerParticlePropagationRK4Factory :
                public IInnerParticlePropagationFactory
            {
            public:
                InnerParticlePropagationRK4Factory(RK4Parameter params);
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IInnerParticlePropagation> create(
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
            private:
                RK4Parameter m_params;
            };
            /**
             * @brief Uses RK4 method to solve inner particle propagation,
             *        including polarization.
             */
            class InnerParticlePropagationRK4PolarizationFactory :
                public IInnerParticlePropagationFactory
            {
            public:
                InnerParticlePropagationRK4PolarizationFactory(RK4Parameter params);
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IInnerParticlePropagation> create(
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                bool createStageDependentData(
                    const MemoryInfo& memory_info,
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping,
                    std::shared_ptr<void>& stage_dependent_data) override;
            private:
                RK4Parameter m_params;
            };
        }
    }
}

#endif