#ifndef WWU_LDPLAB_RTSCPU_RAY_TRACING_STEP_CPU_HPP
#define WWU_LDPLAB_RTSCPU_RAY_TRACING_STEP_CPU_HPP

#include "..\IRayTracingStep.hpp"

#include <memory>
#include <vector>

namespace ldplab
{
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
            friend class RayTracingStepFactory;
        public:
            /**
             * @brief Inherited via IRayTracingStep. Starts the ray tracing
             *        simulation.
             * @param[in] input The ray tracing step input.
             * @param[out] output Results of the ray tracing step execution.
             */
            void execute(
                const RayTracingStepInput& input,
                RayTracingStepOutput& output) override;
        private:
            /**
            * @brief Constructor
            */
            RayTracingStepCPU(std::shared_ptr<Context> context);

        };
    }
}

#endif
