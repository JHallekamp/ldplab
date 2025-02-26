#ifndef WWU_LDPLAB_RTSCPU_default_factories_FACTORIES_HPP
#define WWU_LDPLAB_RTSCPU_default_factories_FACTORIES_HPP

#include "StageFactories.hpp"

namespace ldplab
{
    namespace rtscpu
    {
        namespace default_factories
        {
            class GenericGeometryRodFactory : public IGenericGeometryFactory
            {
            public:
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    IParticleGeometry::Type geometry_type,
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IGenericGeometry> create(
                    const std::shared_ptr<IParticleGeometry>& particle_geometry,
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
            };

            class GenericGeometrySphereFactory : public IGenericGeometryFactory
            {
            public:
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    IParticleGeometry::Type geometry_type,
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IGenericGeometry> create(
                    const std::shared_ptr<IParticleGeometry>& particle_geometry,
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
            };
        }
    }
}

#endif