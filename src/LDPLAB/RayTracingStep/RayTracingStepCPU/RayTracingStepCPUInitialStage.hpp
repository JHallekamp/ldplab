#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CPU_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CPU_INITIAL_STAGE_HPP

#include "RayTracingStepCPU.hpp"
#include "RayTracingStepCPUContext.hpp"

namespace ldplab
{
    class RayTracingStepCPUInitialStage
    {
    public:
        RayTracingStepCPUInitialStage(
            std::shared_ptr<RayTracingStepCPUContext> context);
        void executeSetup();
        void execute(size_t particle);
    private:
        std::shared_ptr<RayTracingStepCPUContext> m_context;
    };
}

#endif