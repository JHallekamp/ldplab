#ifndef WWU_LDPLAB_RTSCPU_DEFAULT_BOUNDING_VOLUME_INTERSECTION_FACTORIES_HPP
#define WWU_LDPLAB_RTSCPU_DEFAULT_BOUNDING_VOLUME_INTERSECTION_FACTORIES_HPP

#include "StageFactories.hpp"

namespace ldplab
{
    namespace rtscpu
    {
        namespace default_factories
        {
            /**
             * @brief Bruteforces the bounding volume intersection test for
             *        bounding spheres.
             */
            class BoundingSphereIntersectionBruteforceFactory :
                public IBoundingVolumeIntersectionFactory
            {
            public:
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IBoundingVolumeIntersection> create(
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
            };
        }
    }
}

#endif