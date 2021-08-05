#ifndef WWU_LDPLAB_RTSCUDA_DEFAULT_PARTICLE_INTERSECTION_FACTORIES_HPP
#define WWU_LDPLAB_RTSCUDA_DEFAULT_PARTICLE_INTERSECTION_FACTORIES_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include "Factories.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        namespace default_factories
        {
            class ParticleIntersectionFactory :
                public IParticleIntersectionFactory
            {
            public:
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    const RayTracingStepCUDAInfo& step_info,
                    const GlobalData::DeviceProperties& device_properties,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                bool createStageDependentData(
                    const GlobalData& global_data,
                    const RayTracingStepCUDAInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping,
                    std::shared_ptr<void>& stage_dependent_data) override;
                std::shared_ptr<IParticleIntersection> create(
                    const GlobalData& global_data,
                    const RayTracingStepCUDAInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
            };
        }
    }
}

#endif
#endif