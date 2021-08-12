#ifndef WWU_LDPLAB_RTSCPU_IMPL_SURFACE_INTERACTION_HPP
#define WWU_LDPLAB_RTSCPU_IMPL_SURFACE_INTERACTION_HPP

#include <LDPLAB/RayTracingStep/CPU/ISurfaceInteraction.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        class SurfaceInteraction :
            public ISurfaceInteraction
        {
        public:
            virtual void stepSetup(
                const ExperimentalSetup& setup, 
                const SimulationState& simulation_state,
                const InterfaceMapping& interface_mapping,
                const std::vector<ParticleTransformation>& particle_transformation) override { }
            virtual void execute(
                const RayBuffer& input_ray_data,
                const IntersectionBuffer& intersection_data,
                RayBuffer& output_ray_data,
                OutputBuffer& output_data,
                double intensity_cutoff,
                double medium_reflection_index,
                const std::vector<std::shared_ptr<IParticleMaterial>>& material_data,
                const std::vector<Vec3>& center_of_mass,
                InteractionPassType pass_type,
                size_t pass_no,
                const SimulationParameter& simulation_parameter,
                void* stage_dependent_data) override;
        private:
            double reflectance(
                double cos_alpha, double cos_beta, double n_r) const;
        };
    }
} 

#endif