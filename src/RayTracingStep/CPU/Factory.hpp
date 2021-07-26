#ifndef WWU_LDPLAB_RTSCPU_FACTORY_HPP
#define WWU_LDPLAB_RTSCPU_FACTORY_HPP

#include <set>

#include <LDPLAB/RayTracingStep/CPU/PipelineConfiguration.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCPUInfo.hpp>

#include "RayTracingStepCPU.hpp"

namespace ldplab
{
    namespace rtscpu
    {
        class Factory
        {
        public:
            std::shared_ptr<RayTracingStepCPU> createRTS(
                const RayTracingStepCPUInfo& info,
                ExperimentalSetup&& setup);
            std::shared_ptr<RayTracingStepCPU> createRTS(
                const RayTracingStepCPUInfo& info,
                ExperimentalSetup&& setup,
                PipelineConfiguration& user_configuration,
                bool allow_default_stage_overwrite_on_compability_error);
        private:
            struct PipelineConfigurationBooleanState
            {
                bool bounding_volume_intersection_state;
                bool initial_stage_state;
                bool inner_particle_propagation_state;
                bool particle_intersection_state;
                bool surface_interaction_state;
                std::map<IParticleGeometry::Type, bool> generic_geometry_state;
            };
        private:
            void createDefaultConfiguration(
                const RayTracingStepCPUInfo& info,
                const ExperimentalSetup& setup,
                PipelineConfiguration& default_config);
            std::set<IParticleGeometry::Type> getPresentGeometryTypes(
                const ExperimentalSetup& setup);
            bool combineConfigurations(
                std::set<IParticleGeometry::Type>& geometry_types,
                PipelineConfiguration& default_config,
                PipelineConfiguration& user_config,
                PipelineConfiguration& combination);
            InterfaceMapping createInterfaceMapping(
                const ExperimentalSetup& setup);
            bool createViableConfiguration(
                const RayTracingStepCPUInfo& info,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping,
                std::set<IParticleGeometry::Type>& geometry_types,
                PipelineConfiguration& configuration,
                PipelineConfiguration& default_configuration,
                PipelineConfiguration& user_config,
                bool allow_default_stage_overwrite_on_compability_error);
            PipelineConfigurationBooleanState validateConfigurationCompability(
                const RayTracingStepCPUInfo& info,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping,
                PipelineConfiguration& configuration);
            PipelineConfigurationBooleanState checkConfigurationCast(
                std::set<IParticleGeometry::Type>& geometry_types,
                PipelineConfiguration& configuration);
            bool checkForConfigurationStateUniformity(
                const PipelineConfigurationBooleanState& configuration_state,
                bool desired_uniform_state);
            void logViableConfiguration(PipelineConfiguration& config);
            bool createPipeline(
                const RayTracingStepCPUInfo& info,
                InterfaceMapping&& interface_mapping,
                ExperimentalSetup&& setup,
                PipelineConfiguration& pipeline_config,
                std::shared_ptr<RayTracingStepCPU>& rts);
        };
    }
}

#endif