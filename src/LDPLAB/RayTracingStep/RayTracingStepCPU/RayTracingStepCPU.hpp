#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CPU_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CPU_HPP

#include "..\IRayTracingStep.hpp"

#include <memory>
#include <vector>

namespace ldplab
{
    // Prototype
    class RayTracingStepCPUContext;

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
        RayTracingStepCPU(std::shared_ptr<RayTracingStepCPUContext> context);

    };
}

#endif
