#ifndef WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP

#include "RayTracingStepCPU/RayTracingStepCPU.hpp"
#include "RayTracingStepCPUInfo.hpp"
#include "RayTracingStepCPU/Context.hpp"

#include <memory>



namespace ldplab
{
    /**
     * @brief Class building a RayTracingStep.  
     */
    class RayTracingStepFactory
    {
    public:
        static std::shared_ptr<rtscpu::RayTracingStepCPU> 
            createRayTracingStepCPU(RayTracingStepCPUInfo& info);
    private:
        static void initGeometry(
            RayTracingStepCPUInfo& info, std::shared_ptr<rtscpu::Context> context);
    };
}

#endif