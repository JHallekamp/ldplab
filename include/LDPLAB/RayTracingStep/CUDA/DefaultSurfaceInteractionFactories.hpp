#ifndef WWU_LDPLAB_RTSCUDA_DEFAULT_SURFACE_INTERACTION_FACTORIES_HPP
#define WWU_LDPLAB_RTSCUDA_DEFAULT_SURFACE_INTERACTION_FACTORIES_HPP

#include "Factories.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        namespace default_factories
        {
            class SurfaceInteractionFactory :
                public ISurfaceInteractionFactory
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
                std::shared_ptr<ISurfaceInteraction> create(
                    const RayTracingStepCUDAInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const GlobalData& global_data) override;
            };
        }
    }
}

#endif