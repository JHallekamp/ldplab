#ifndef WWU_LDPLAB_RTSCUDA_DEFAULT_GENERIC_GEOMETRY_FACTORIES_HPP
#define WWU_LDPLAB_RTSCUDA_DEFAULT_GENERIC_GEOMETRY_FACTORIES_HPP

#include "Factories.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        namespace default_factories
        {
            class GenericGeometrySphereFactory :
                public IGenericGeometryFactory
            {
            public:
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    IParticleGeometry::Type geometry_type,
                    const RayTracingStepCUDAInfo& step_info,
                    const ExecutionModel& execution_model,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IGenericGeometry> create(
                    const std::shared_ptr<IParticleGeometry>& particle_geometry,
                    const RayTracingStepCUDAInfo& step_info,
                    const DeviceProperties& device_properties,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
            };

            class GenericGeometryRodFactory :
                public IGenericGeometryFactory
            {
            public:
                static std::string name();
                std::string implementationName() const override;
                bool userDefined() const override;
                bool checkCompability(
                    IParticleGeometry::Type geometry_type,
                    const RayTracingStepCUDAInfo& step_info,
                    const ExecutionModel& execution_model,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
                std::shared_ptr<IGenericGeometry> create(
                    const std::shared_ptr<IParticleGeometry>& particle_geometry,
                    const RayTracingStepCUDAInfo& step_info,
                    const DeviceProperties& device_properties,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping) override;
            };
        }
    }
}

#endif