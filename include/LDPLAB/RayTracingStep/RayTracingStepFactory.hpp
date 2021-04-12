#ifndef WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP

#include "../ExperimentalSetup/ExperimentalSetup.hpp"
#include "IRayTracingStep.hpp"
#include "RayTracingStepCPUInfo.hpp"
#include "RayTracingStepGPUOpenGLInfo.hpp"

#include <memory>

namespace ldplab
{
    // prototype
    namespace rtscpu { struct Context; }
    namespace rtsgpu_ogl { struct Context; }

    /**
     * @brief Class building a RayTracingStep.  
     */
    class RayTracingStepFactory
    {
    public:
        /** @brief No instances of this class allowed. */
        RayTracingStepFactory() = delete;
        /** 
         * @brief Creates a ray tracing step that runs on CPU only.
         * @param[in] setup The setup of the experiment.
         * @param[in] info Contains the information needed to create the stage.
         * @returns Pointer to the ray tracing step instance.
         */
        static std::shared_ptr<IRayTracingStep> 
            createRayTracingStepCPU(
                const ExperimentalSetup& setup,
                const RayTracingStepCPUInfo& info);
        /**
         * @brief Creates a ray tracing step that utilizes the GPU.
         * @param[in] setup The setup of the experiment.
         * @param[in] info Contains the information needed to create the stage.
         * @returns Pointer to the ray tracing step instance.
         */
        static std::shared_ptr<IRayTracingStep>
            createRayTracingStepGPUOpenGL(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info);
    };
}

#endif