#ifndef WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP

#include "RayTracingStepCPU/RayTracingStepCPU.hpp"
#include "RayTracingStepCPU/RayTracingStepCPUInfo.hpp"

#include <memory>

namespace ldplab
{
    /**
     * @brief Class building a RayTracingStep.  
     */
    class RayTracingStepFactory
    {
    public:
        static std::shared_ptr<RayTracingStepCPU> 
            createRayTracingStepCPU(RayTracingStepCPUInfo& info);
    };
}

#endif