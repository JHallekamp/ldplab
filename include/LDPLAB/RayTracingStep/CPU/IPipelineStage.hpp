#ifndef WWU_LDPLAB_RTSCPU_I_PIPELINE_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_I_PIPELINE_STAGE_HPP

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/RayTracingStep/CPU/Data.hpp>
#include <LDPLAB/RayTracingStep/IRayTracingStep.hpp>
#include <LDPLAB/SimulationState.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        // Prototypes
        class Factory;

        /** @brief Abstract baseclass for pipeline stages. */
        class IPipelineStage
        {
            friend Factory;
        public:
            /**
             * @brief Called each time before the ray tracing step pipeline
             *        is executed.
             * @param[in] setup The experimental setup with which the ray
             *                  tracing step was created.
             * @param[in] simulation_state The current simulation state with
             *                             which the step execution has been
             *                             started.
             * @param[in] interface_mapping The ray tracing step interface
             *                              mapping to relate experimental
             *                              setup and simulation state
             *                              components given by their uid to
             *                              the internal index based structure.
             */
            virtual void stepSetup(
                const ExperimentalSetup& setup,
                const SimulationState& simulation_state,
                const InterfaceMapping& interface_mapping) = 0;
            /**
             * @brief Called once each time before a ray batch is created.
             * @param[in] stage_dependent_data A trhead-local pointer to a data
             *                                 structure created by the stage
             *                                 factory. Can be null, if the
             *                                 stage factory did not provide
             *                                 any data on stage creation.
             */
            virtual void batchSetup(void* stage_dependent_data) { };
        protected:
            UID<IRayTracingStep> getParentRayTracingStepUID() const;
        private:
            UID<IRayTracingStep> m_parent_rts_uid;
        };
    }
}

#endif