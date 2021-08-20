#ifndef WWU_LDPLAB_RTSCUDA_I_PIPELINE_STAGE_HPP
#define WWU_LDPLAB_RTSCUDA_I_PIPELINE_STAGE_HPP

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
             * @brief Called once for each device context per execution.
             * @param[in] simulation_state The current simulation state with
             *                             which the step execution has been
             *                             started.
             * @param[in] shared_data The shared data instance.
             * @param[in] device_context The device context for which the setup
             *                           is called.
             * @note The step setup is always called in sequential order, 
             *       starting with device group 0.
             */
            virtual void stepSetup(
                const SimulationState& simulation_state,
                SharedStepData& shared_data,
                DeviceContext& device_context) { };
            /**
             * @brief Called once each time before a ray batch is created.
             * @param[in] shared_data The global pipeline data.
             * @param[in] stream_context The context of the stream.
             */
            virtual void batchSetup(
                const SharedStepData& shared_data,
                StreamContext& stream_context,
                size_t batch_no) { };
        protected:
            UID<IRayTracingStep> getParentRayTracingStepUID() const;
        private:
            UID<IRayTracingStep> m_parent_rts_uid;
        };
    }
}

#endif