#ifndef WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP

#include "../ExperimentalSetup/ExperimentalSetup.hpp"
#include "IRayTracingStep.hpp"
#include "RayTracingStepCPUInfo.hpp"
#include "RayTracingStepCUDAInfo.hpp"
#include "RayTracingStepOpenGLInfo.hpp"

#include "CPU/PipelineConfiguration.hpp"

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
         * @param[in] info Contains the information needed to create the stage.
         * @param[in] setup The setup of the experiment.
         * @returns Pointer to the ray tracing step instance.
         * @note Forfeits ownership of the experimental setup. If you want to
         *       keep the ownership, you should create a complete copy of the
         *       setup and all its contents.
         */
        static std::shared_ptr<IRayTracingStep>
            createRayTracingStepCPU(
                const RayTracingStepCPUInfo& info,
                ExperimentalSetup&& setup);
        /**
         * @brief Creates a ray tracing step that runs on CPU only.
         * @param[in] info Contains the information needed to create the stage.
         * @param[in] setup The setup of the experiment.
         * @param[in] user_defined_configuration A pipeline configuration 
         *                                       containing stage factories 
         *                                       that the ray tracing step 
         *                                       factory favours over the 
         *                                       automatic generated default
         *                                       configuration.
         * @param[in] allow_default_stage_overwrites If set to true, allows the
         *                                           ray tracing step factory 
         *                                           to overwrite stages in the
         *                                           user defined pipeline 
         *                                           configuration with default
         *                                           stages, if the user 
         *                                           provided stage factory 
         *                                           reports incompatibility
         *                                           to the given setup and
         *                                           configuration.
         * @returns Pointer to the ray tracing step instance.
         * @note Forfeits ownership of the experimental setup. If you want to
         *       keep the ownership, you should create a complete copy of the
         *       setup and all its contents.
         */
        static std::shared_ptr<IRayTracingStep>
            createRayTracingStepCPU(
                const RayTracingStepCPUInfo& info,
                ExperimentalSetup&& setup,
                rtscpu::PipelineConfiguration& user_defined_configuration,
                bool allow_default_stage_overwrites = false);
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