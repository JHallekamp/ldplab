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
        class Context;

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
             */
            void start() override;
        private:
            /**
            * @brief Constructor
            */
            RayTracingStepCPU(std::shared_ptr<Context> context);

        };
    }
}

#endif
