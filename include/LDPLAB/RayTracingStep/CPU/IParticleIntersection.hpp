#ifndef WWU_LDPLAB_RTSCPU_I_PARTICLE_INTERSECTION_HPP
#define WWU_LDPLAB_RTSCPU_I_PARTICLE_INTERSECTION_HPP

#include <memory>

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



        /**
         * @brief Abstract baseclass for the geometry intersection pipeline
         *        stage.
         */
        class IParticleIntersection : public IPipelineStage
        {
        public:
            virtual ~IParticleIntersection() { }
            /**
             * @brief Performs the intersection test between the particles and
             *        a batch of rays.
             * @details The stage needs to test each ray that is in particle
             *          space for an intersection with the corresponding 
             *          particle (given by the rays particle index).
             *          The intersection test itself should be performed by the 
             *          generic geometry instance representing the particle 
             *          geometry.
             *          If a ray intersects a particle, the intersection data
             *          consisting of the point of intersection and the surface 
             *          normal, both in particle space, as well as the index of
             *          the intersected particle needs to be stored in the 
             *          intersection buffer.
             *          If a ray does not intersect the particle, the particle
             *          intersection stage needs to transform that ray back 
             *          into world space (and update its index accordingly).
             * @param[in, out] ray_data Buffer containing the ray data.
             * @param[out] intersection_data The buffer that has to be filled
             *                               with intersection data.
             * @param[in] transformation_data Transformation data per particle
             *                                to transform rays into particle
             *                                space.
             * @param[in] geometry_data Generic geometry instances per particle
             *                          used for intersection tests.
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
                const std::vector<ParticleTransformation>& transformation_data,
                const std::vector<std::shared_ptr<IGenericGeometry>>& geometry_data,
                const SimulationParameter& simulation_parameter,
                void* stage_dependent_data) = 0;
        };
    }
}

#endif
