#ifndef WWU_LDPLAB_RTSCPU_IMPL_SURFACE_INTERACTION_HPP
#define WWU_LDPLAB_RTSCPU_IMPL_SURFACE_INTERACTION_HPP

#include <LDPLAB/RayTracingStep/CPU/ISurfaceInteraction.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        class SurfaceInteraction1DLinearRefractionIndexGradient :
            public ISurfaceInteraction
        {
        public:
            virtual void stepSetup(
                const ExperimentalSetup& setup, 
                const SimulationState& simulation_state,
                const InterfaceMapping& interface_mapping) override { }
            virtual void execute(
                const RayBuffer& input_ray_data,
                const IntersectionBuffer& intersection_data,
                RayBuffer& output_ray_data,
                OutputBuffer& output_data,
                double intensity_cutoff,
                double medium_reflection_index,
                const std::vector<std::shared_ptr<IParticleMaterial>>& material_data,
                InteractionPassType pass_type,
                size_t pass_no,
                const SimulationParameter& simulation_parameter,
                void* stage_dependent_data) override;
        };
    }
} 

#endif