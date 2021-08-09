#ifndef WWU_LDPLAB_RTSCUDA_DEFAULT_INNER_PRATICLE_PROPAGATION_FACTORIES_HPP
#define WWU_LDPLAB_RTSCUDA_DEFAULT_INNER_PRATICLE_PROPAGATION_FACTORIES_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

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
                    const GlobalData::DeviceProperties& device_properties,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IInnerParticlePropagation> create(
                    const RayTracingStepCUDAInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const GlobalData& global_data) override;
            private:
                RK4Parameter m_parameter;
            };
        }
    }
}

#endif
#endif