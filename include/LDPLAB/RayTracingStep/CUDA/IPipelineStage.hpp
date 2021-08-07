#ifndef WWU_LDPLAB_RTSCUDA_I_PIPELINE_STAGE_HPP
#define WWU_LDPLAB_RTSCUDA_I_PIPELINE_STAGE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/RayTracingStep/IRayTracingStep.hpp>
#include <LDPLAB/SimulationState.hpp>

#include "Data.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        // Prototypes
        class Factory;

        /** @brief Abstract baseclass for pipeline stages. */
        class IPipelineStage
        {
            friend Factory;
        public:
            virtual ~IPipelineStage() { }
            /**
             * @brief Called each time before the ray tracing step pipeline
             *        is executed.
             * @param[in] simulation_state The current simulation state with
             *                             which the step execution has been
             *                             started.
             * @param[in] global_data The global pipeline data.
             */
            virtual void stepSetup(
                const SimulationState& simulation_state,
                const GlobalData& global_data) { };
            /**
             * @brief Called once each time before a ray batch is created.
             * @param[in] global_data The global pipeline data.
             * @param[in] batch_data All data associated with the current 
             *                       batch.
             */
            virtual void batchSetup(
                const GlobalData& global_data,
                BatchData& batch_data) { };
        protected:
            UID<IRayTracingStep> getParentRayTracingStepUID() const;
        private:
            UID<IRayTracingStep> m_parent_rts_uid;
        };
    }
}

#endif
#endif