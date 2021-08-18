#ifndef WWU_LDPLAB_RTSCUDA_FACTORY_HPP
#define WWU_LDPLAB_RTSCUDA_FACTORY_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <memory>
#include <set>

#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <LDPLAB/RayTracingStep/CUDA/Factories.hpp>

#include "RayTracingStepCUDA.hpp"
#include "IPipeline.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class Factory
        {
        public:
            std::shared_ptr<RayTracingStepCUDA> createRTS(
                const RayTracingStepCUDAInfo& info,
                ExperimentalSetup&& setup);
            std::shared_ptr<RayTracingStepCUDA> createRTS(
                const RayTracingStepCUDAInfo& info,
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
                std::map<IParticleMaterial::Type, bool> generic_material_state;
            };
        private:
            void createDefaultConfiguration(
                const RayTracingStepCUDAInfo& info,
                const ExperimentalSetup& setup,
                PipelineConfiguration& default_config);
            std::set<IParticleGeometry::Type> getPresentGeometryTypes(
                const ExperimentalSetup& setup);
            std::set<IParticleMaterial::Type> getPresentMaterialTypes (
                const ExperimentalSetup& setup);
            bool combineConfigurations(
                std::set<IParticleGeometry::Type>& geometry_types,
                std::set<IParticleMaterial::Type>& material_types,
                PipelineConfiguration& default_config,
                PipelineConfiguration& user_config,
                PipelineConfiguration& combination);
            InterfaceMapping createInterfaceMapping(
                const ExperimentalSetup& setup);
            bool createViableConfiguration(
                const RayTracingStepCUDAInfo& info,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping,
                const GlobalData::DeviceProperties& device_properties,
                std::set<IParticleGeometry::Type>& geometry_types,
                std::set<IParticleMaterial::Type>& material_types,
                PipelineConfiguration& configuration,
                PipelineConfiguration& default_configuration,
                PipelineConfiguration& user_config,
                bool allow_default_stage_overwrite_on_compability_error);
            PipelineConfigurationBooleanState validateConfigurationCompability(
                const RayTracingStepCUDAInfo& info,
                const ExperimentalSetup& setup,
                const InterfaceMapping& interface_mapping,
                const GlobalData::DeviceProperties& device_properties,
                PipelineConfiguration& configuration);
            PipelineConfigurationBooleanState checkConfigurationCast(
                std::set<IParticleGeometry::Type>& geometry_types,
                std::set<IParticleMaterial::Type>& material_types,
                PipelineConfiguration& configuration);
            bool checkForConfigurationStateUniformity(
                const PipelineConfigurationBooleanState& configuration_state,
                bool desired_uniform_state);
            void logViableConfiguration(PipelineConfiguration& config);
            bool getDeviceData(GlobalData::DeviceProperties& device_props);
            bool createGlobalData(
                const RayTracingStepCUDAInfo& info,
                PipelineConfiguration& pipeline_config,
                ExperimentalSetup&& setup,
                InterfaceMapping&& interface_mapping,
                GlobalData::DeviceProperties&& device_properties,
                std::unique_ptr<GlobalData>& global_data);
            bool createPipeline(
                const RayTracingStepCUDAInfo& info,
                GlobalData::DeviceProperties&& device_properties,
                InterfaceMapping&& interface_mapping,
                ExperimentalSetup&& setup,
                PipelineConfiguration& pipeline_config,
                std::shared_ptr<RayTracingStepCUDA>& rts);
        };
    }
}

#endif
#endif