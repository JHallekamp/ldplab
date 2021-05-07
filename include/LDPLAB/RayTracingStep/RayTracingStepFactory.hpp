#ifndef WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP

#include "../ExperimentalSetup/ExperimentalSetup.hpp"
#include "IRayTracingStep.hpp"
#include "RayTracingStepCPUInfo.hpp"
#include "RayTracingStepCUDAInfo.hpp"
#include "RayTracingStepOpenGLInfo.hpp"

#include <memory>

namespace ldplab
{
    // prototype
    namespace rtscpu { struct Context; }
    namespace rtsogl { struct Context; }

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
         * @brief Creates a ray tracing step that utilizes CUDA.
         * @param[in] setup The setup of the experiment.
         * @param[in] info Contains the information needed to create the stage.
         * @returns Pointer to the ray tracing step instance.
         */
        static std::shared_ptr<IRayTracingStep>
            createRayTracingStepCUDA(
                const ExperimentalSetup& setup,
                const RayTracingStepCUDAInfo& info);
        /**
         * @brief Creates a ray tracing step that utilizes OpenGL to achieve
         *        GPU acceleration.
         * @param[in] setup The setup of the experiment.
         * @param[in] info Contains the information needed to create the stage.
         * @returns Pointer to the ray tracing step instance.
         */
        static std::shared_ptr<IRayTracingStep>
            createRayTracingStepOpenGL(
                const ExperimentalSetup& setup,
                const RayTracingStepOpenGLInfo& info);
    };
}

#endif