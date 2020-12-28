#ifndef WWU_LDPLAB_RTSCPU_RAY_TRACING_STEP_CPU_HPP
#define WWU_LDPLAB_RTSCPU_RAY_TRACING_STEP_CPU_HPP

#include "..\IRayTracingStep.hpp"

#include <memory>
#include <vector>

namespace ldplab
{
    // Protoype
    class RayTracingStepFactory;

    namespace rtscpu
    {
        // Prototype
        struct Context;

        /**
         * @brief Class implementing IRaytracingStep on the CPU. It is non
         *        distributed.
         */
        class RayTracingStepCPU : public IRayTracingStep
        {
            friend RayTracingStepFactory;
        public:
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
            /**
            * @brief Constructor
            */
            RayTracingStepCPU(std::shared_ptr<Context> context);
        private:
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif
