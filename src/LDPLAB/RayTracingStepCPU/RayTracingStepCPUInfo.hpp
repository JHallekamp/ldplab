#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP

#include "..\ExperimentalSetup.hpp"

namespace ldplab
{
    /**
     * @brief Structure containing all information to create an 
     *        RayTracingStepCPU 
     */
    struct RayTracingStepCPUInfo
    {
        ExperimentalSetup setup;
        // Threadpool
    };
}

#endif
