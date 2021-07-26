#ifndef WWU_LDPLAB_RTSCPU_I_INNER_PARTICLE_PROPAGATION_HPP
#define WWU_LDPLAB_RTSCPU_I_INNER_PARTICLE_PROPAGATION_HPP

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/RayTracingStep/CPU/Data.hpp>
#include <LDPLAB/RayTracingStep/CPU/IPipelineStage.hpp>
#include <LDPLAB/SimulationState.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        // Prototype
        class IGenericGeometry;

        /** @brief Abstract baseclass for the inner particle propagation. */
        class IInnerParticlePropagation : public IPipelineStage
        {
        public:
            virtual ~IInnerParticlePropagation() { }
            /**
             * @brief Computes the ray route through the particle and the 
             *        resulting influence on force and torque.
             * @param[in, out] Buffer containing the ray data.
             * @param[out] intersection_data The buffer that needs to be filled
             *                               with the intersection data when 
             *                               the ray leaves the particle.
             * @param[in, out] output_data Output buffer that is used to 
             *                             collect the force and torque data
             *                             generated during the inner particle
             *                             ray propagation.
             * @param[in] geometry_data Generic geometry instances per particle
             *                          used for intersection tests.
             * @param[in] material_data Generic material instances per particle
             *                          used for ray particle interaction 
             *                          behavior.
             * @param[in] center_of_mass Center of mass per particle given in 
             *                           the respective particle coordinate 
             *                           system.
             * @param[in] simulation_parameter Parameters of the simulation.
             * @param[in] stage_dependent_data A trhead-local pointer to a data
             *                                 structure created by the stage
             *                                 factory. Can be null, if the 
             *                                 stage factory did not provide 
             *                                 any data on stage creation.
             */
            virtual void execute(
                RayBuffer& ray_data,
                IntersectionBuffer& intersection_data,
                OutputBuffer& output_data,
                const std::vector<std::shared_ptr<IGenericGeometry>>& geometry_data, 
                const std::vector<std::shared_ptr<IParticleMaterial>>& material_data,
                const std::vector<Vec3>& center_of_mass,
                const SimulationParameter& simulation_parameter,
                void* stage_dependent_data) = 0;
        };
    }
}

#endif
