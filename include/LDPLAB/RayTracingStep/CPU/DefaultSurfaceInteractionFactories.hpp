#ifndef WWU_LDPLAB_RTSCPU_DEFAULT_SURFACE_INTERACTION_FACTORIES_HPP
#define WWU_LDPLAB_RTSCPU_DEFAULT_SURFACE_INTERACTION_FACTORIES_HPP

#include "StageFactories.hpp"

namespace ldplab
{
    namespace rtscpu
    {
        namespace default_factories
        {
            /**
             * @brief Implements particle surface interaction for materials with a
             *        linear index of refraction gradient in one direction.
             */
            class SurfaceInteractionFactory :
                public ISurfaceInteractionFactory
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
                std::shared_ptr<ISurfaceInteraction> create(
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
            };
        }
    }
}

#endif