#ifndef WWU_LDPLAB_RTSCPU_RAY_TRACING_STEP_HPP
#define WWU_LDPLAB_RTSCPU_RAY_TRACING_STEP_HPP

#include <LDPLAB/RayTracingStep/IRayTracingStep.hpp>
#include <LDPLAB/UID.hpp>

#include "Pipeline.hpp"

namespace ldplab
{
    namespace rtscpu
    {
        // Prototype
        class Factory;

        class RayTracingStepCPU : public IRayTracingStep
        {
            friend Factory;
        public:
            RayTracingStepCPU() { }
            /**
             * @brief Inherited via IRayTracingStep. Starts the ray tracing
             *        simulation.
             * @param[in] input The current simulation state.
             * @param[out] output Results of the ray tracing step execution.
             */
            void execute(
                const SimulationState& input,
                RayTracingStepOutput& output) override;
        private:
            std::shared_ptr<Pipeline> m_pipeline;
            std::shared_ptr<utils::ThreadPool> m_thread_pool;
        };
    }
}

#endif