#ifndef WWU_LDPLAB_RTSCUDA_DEFAULT_INITIAL_STAGE_FACTORIES_HPP
#define WWU_LDPLAB_RTSCUDA_DEFAULT_INITIAL_STAGE_FACTORIES_HPP

#include "Factories.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        namespace default_factories
        {
            class InitialStageHomogenousLightBoundingSphereProjectionFactory :
                public IInitialStageFactory
            {
            public:
                InitialStageHomogenousLightBoundingSphereProjectionFactory(
                    double light_resolution_per_world_unit);
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    const RayTracingStepCUDAInfo& step_info,
                    const GlobalData::DeviceProperties& device_properties,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IInitialStage> create(
                    const RayTracingStepCUDAInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const GlobalData& global_data) override;
            private:
                double m_light_resolution_per_world_unit;
            };
        }
    }
}

#endif