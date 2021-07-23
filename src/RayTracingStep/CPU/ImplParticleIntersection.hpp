#ifndef WWU_LDPLAB_RTSCPU_IMPL_PARTICLE_INTERSECTION_HPP
#define WWU_LDPLAB_RTSCPU_IMPL_PARTICLE_INTERSECTION_HPP

#include <LDPLAB/RayTracingStep/CPU/IParticleIntersection.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        class ParticleIntersection : public IParticleIntersection
        {
        public:
            virtual void stepSetup(
                const ExperimentalSetup& setup, 
                const SimulationState& simulation_state, 
                const InterfaceMapping& interface_mapping) override { }
            virtual void execute(
                RayBuffer& ray_data, 
                IntersectionBuffer& intersection_data, 
                const std::vector<ParticleTransformation>& transformation_data, 
                const std::vector<std::shared_ptr<IGenericGeometry>>& geometry_data, 
                const SimulationParameter& simulation_parameter, 
                void* stage_dependent_data) override;
        };
    }
}

#endif