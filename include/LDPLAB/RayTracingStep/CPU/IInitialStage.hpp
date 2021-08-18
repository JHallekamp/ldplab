#ifndef WWU_LDPLAB_RTSCPU_I_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_I_INITIAL_STAGE_HPP

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/RayTracingStep/CPU/Data.hpp>
#include <LDPLAB/RayTracingStep/CPU/IPipelineStage.hpp>
#include <LDPLAB/SimulationState.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        /** 
         * @brief Abstract baseclass for the initial pipeline stage. 
         * @note Since the initial stage is the only stage interacting with the
         *       light sources (in general), it is instructed to keep its 
         *       required light source data on its own.
         */
        class IInitialStage : public IPipelineStage
        {
        public:
            virtual ~IInitialStage() { }
            /**
             * @brief Creates a batch of rays.
             * @details The initial stage abstracts the light sources and 
             *          creates the rays that traverse the scene. The pipeline
             *          assumes the created rays in world space, however if 
             *          they are already in particle space (which is possible
             *          for example if they are created using bounding volume
             *          projections), the pipeline is also able to handle 
             *          them, which will probably result in a very tiny 
             *          performance gain.
             * @param[out] initial_batch_buffer The ray buffer where created 
             *                                 rays are stored.
             * @param[in] simulation_parameter Parameters of the simulation.
             * @param[in] stage_dependent_data A trhead-local pointer to a data
             *                                 structure created by the stage
             *                                 factory. Can be null, if the 
             *                                 stage factory did not provide 
             *                                 any data on stage creation.
             * @returns true, if there are still batches left to be created
             *          after the current batch is executed. 
             *          Otherwise returns false.
             */
            virtual bool execute(
                RayBuffer& initial_batch_buffer,
                const SimulationParameter& simulation_parameter,
                void* stage_dependent_data) = 0;
        };
    }
}

#endif
