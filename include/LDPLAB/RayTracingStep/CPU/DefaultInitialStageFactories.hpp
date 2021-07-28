#ifndef WWU_LDPLAB_RTSCPU_DEFAULT_INITIAL_STAGE_FACTORIES_HPP
#define WWU_LDPLAB_RTSCPU_DEFAULT_INITIAL_STAGE_FACTORIES_HPP

#include "StageFactories.hpp"

namespace ldplab
{
    namespace rtscpu
    {
        namespace default_stages
        {
            /** @brief Projects bounding spheres onto a homogenous light source. */
            class InitialStageHomogenousLightBoundingSphereProjectionFactory :
                public IInitialStageFactory
            {
            public:
                InitialStageHomogenousLightBoundingSphereProjectionFactory(
                    double num_rays_per_world_space_unit);
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IInitialStage> create(
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
            private:
                double m_num_rays_per_world_space_unit;
            };
        }
    }
}

#endif