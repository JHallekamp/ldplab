#ifndef WWU_LDPLAB_RTSCPU_I_BOUNDING_VOLUME_INTERSECTION_HPP
#define WWU_LDPLAB_RTSCPU_I_BOUNDING_VOLUME_INTERSECTION_HPP

#include <memory>

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/RayTracingStep/CPU/Data.hpp>
#include <LDPLAB/RayTracingStep/CPU/IPipelineStage.hpp>
#include <LDPLAB/SimulationState.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        /**
         * @brief Abstract baseclass for the bounding volume intersection
         *        pipeline stage.
         * @note Since particle bounding volumes are only processed by this
         *       stage (in general), it needs to track the bounding volumes 
         *       on its own. The pipeline therefore does not provide any 
         *       bounding volume representations to the stage execute method.
         */
        class IBoundingVolumeIntersection : public IPipelineStage
        {
        public:
            virtual ~IBoundingVolumeIntersection() { }
            /**
             * @brief Performs an intersection test of all world space rays
             *        with the bounding spheres.
             * @details Each ray inside the buffer that is in world space (when
             *          particle index is greater than the number of particles)
             *          needs to be checked against the bounding volumes of the 
             *          particles inside the experimental setup. If a ray
             *          intersects a bounding volume, it has to be transformed
             *          into the corresponding particle space. Otherwise, if
             *          the ray intersects no particle bounding volume, it has
             *          to be set to invalid by settings its corresponding 
             *          particle index to a negative value.
             * @param[in, out] ray_data The ray buffer that contains the ray 
             *                          data.
             * @param[in] transformation_data Transformation data per particle
             *                                to transform rays into particle
             *                                space.
             * @param[in] simulation_parameter Parameters of the simulation.
             * @param[in] stage_dependent_data A trhead-local pointer to a data
             *                                 structure created by the stage
             *                                 factory. Can be null, if the 
             *                                 stage factory did not provide 
             *                                 any data on stage creation.
             * @returns The number of rays that hit a particle bounding volume.
             */
            virtual size_t execute(
                RayBuffer& ray_data,
                const std::vector<ParticleTransformation>& transformation_data,
                const SimulationParameter& simulation_parameter,
                void* stage_dependent_data) = 0;
        };
    }
}

#endif
