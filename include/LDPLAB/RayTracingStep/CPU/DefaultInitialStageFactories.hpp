#ifndef WWU_LDPLAB_RTSCPU_DEFAULT_INITIAL_STAGE_FACTORIES_HPP
#define WWU_LDPLAB_RTSCPU_DEFAULT_INITIAL_STAGE_FACTORIES_HPP

#include "StageFactories.hpp"

namespace ldplab
{
    namespace rtscpu
    {
        namespace default_factories
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

            class InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory :
                public IInitialStageFactory
            {
                friend class SurfaceInteractionPolarizedLightFactory;
                friend class ParticleIntersectionPolarizedLightFactory;
            public:
                struct Polarization
                {
                    Vec4 stokes_parameter;
                    Vec3 direction;
                    bool is_particle_system = false;
                };
                struct PolarizationBuffer
                {
                    PolarizationBuffer(size_t buffer_index, size_t size)
                        :
                        polarization_data{ nullptr },
                        buffer_index{ buffer_index },
                        size{ size }
                    { }
                    Polarization* polarization_data;
                    const size_t buffer_index;
                    const size_t size;
                };
                struct PolarizationData
                {
                    std::vector<PolarizationBuffer> polarization_buffers;
                    std::vector<Polarization> polarization_data;
                };
            private:
                std::shared_ptr<PolarizationData> getPolatizationData(
                    const MemoryInfo& memory_info,
                    const RayTracingStepCPUInfo& step_info);
            public:
                InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory(
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
                bool createStageDependentData(
                    const MemoryInfo& memory_info,
                    const RayTracingStepCPUInfo& step_info,
                    const PipelineConfiguration& configuration,
                    const ExperimentalSetup& setup,
                    const InterfaceMapping& interface_mapping,
                    std::shared_ptr<void>& stage_dependent_data) override;
            private:
                double m_num_rays_per_world_space_unit;
                std::vector<std::shared_ptr<PolarizationData>>
                    m_polarization_data_pointer;
            };
        }
    }
}

#endif