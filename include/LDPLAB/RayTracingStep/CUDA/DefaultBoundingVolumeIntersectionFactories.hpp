#ifndef WWU_LDPLAB_RTSCUDA_DEFAULT_BOUNDING_VOLUME_INTERSECTION_FACTORIES_HPP
#define WWU_LDPLAB_RTSCUDA_DEFAULT_BOUNDING_VOLUME_INTERSECTION_FACTORIES_HPP

#include "Factories.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        namespace default_factories
        {
            class BoundingSphereIntersectionBruteforceFactory :
                public IBoundingVolumeIntersectionFactory
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
                std::shared_ptr<IBoundingVolumeIntersection> create(
                    const RayTracingStepCUDAInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const GlobalData& global_data) override;
            };
        }
    }
}

#endif