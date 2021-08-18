#ifndef WWU_LDPLAB_RTSCPU_DEFAULT_PARTICLE_INTERSECIONT_FACTORIES_HPP
#define WWU_LDPLAB_RTSCPU_DEFAULT_PARTICLE_INTERSECIONT_FACTORIES_HPP

#include "StageFactories.hpp"

namespace ldplab
{
    namespace rtscpu
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
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IParticleIntersection> create(
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
            };
        }
    }
}

#endif