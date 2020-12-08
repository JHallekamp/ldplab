#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP

#include "..\..\ExperimentalSetup\ExperimentalSetup.hpp"
#include "..\..\ThreadPool.hpp"

#include <memory>

namespace ldplab
{
    /**
     * @brief Structure containing all information to create an 
     *        RayTracingStepCPU 
     */
    struct RayTracingStepCPUInfo
    {
        std::shared_ptr<ExperimentalSetup> setup;
        std::shared_ptr<ThreadPool> thread_pool;
    };
}

#endif
