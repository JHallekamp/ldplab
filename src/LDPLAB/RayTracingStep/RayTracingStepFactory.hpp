#ifndef WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP

#include "..\ExperimentalSetup\ExperimentalSetup.hpp"
#include "RayTracingStepCPUInfo.hpp"
#include "RayTracingStepCPU/RayTracingStep.hpp"
#include "RayTracingStepGPUOpenGLInfo.hpp"
#include "RayTracingStepGPU/OpenGL/RayTracingStep.hpp"

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
        /** 
         * @brief Creates a ray tracing step that runs on CPU only.
         * @param[in] setup The setup of the experiment.
         * @param[in] info Contains the information needed to create the stage.
         * @returns Pointer to the ray tracing step instance.
         */
        static std::shared_ptr<rtscpu::RayTracingStep> 
            createRayTracingStepCPU(
                const ExperimentalSetup& setup,
                const RayTracingStepCPUInfo& info);
        /**
         * @brief Creates a ray tracing step and provides pointers to the 
         *        context and individual stages to debug them individually.
         * @param[in] setup The setup of the experiment.
         * @param[in] info Contains the information needed to create the stage.
         * @param[out] debug Stores the pointer to context and ray tracing
         *                   pipeline stages.
         * @returns Pointer to the ray tracing step instance.
         * @note This does NOT create a pipeline and therefore executing the
         *       ray tracing step will fail with a null pointer exception. You
         *       will have to do the pipeline work (e.g. updating particle data
         *       or ray buffer management) on your own.
         */
        static std::shared_ptr<rtscpu::RayTracingStep>
            createRayTracingStepCPUDebug(
                const ExperimentalSetup& setup,
                const RayTracingStepCPUInfo& info,
                RayTracingStepCPUDebugInfo& debug);
        /**
         * @brief Creates a ray tracing step that utilizes the GPU.
         * @param[in] setup The setup of the experiment.
         * @param[in] info Contains the information needed to create the stage.
         * @returns Pointer to the ray tracing step instance.
         */
        static std::shared_ptr<rtsgpu_ogl::RayTracingStep>
            createRayTracingStepGPUOpenGL(
                const ExperimentalSetup& setup,
                const RayTracingStepGPUOpenGLInfo& info);
    private:
        static void initRodParticleGeometryCPU(
            const ExperimentalSetup& setup,
            std::shared_ptr<rtscpu::Context> context);
        static void initRodParticleGeometryGPUOpenGL(
            const ExperimentalSetup& setup,
            std::shared_ptr<rtsgpu_ogl::Context> context);
        /**
         * @brief Checking if all types in the experimental setup are the same.
         * @param[in] info Structure containing all information about the setup.
         * @returns true if all types are equal and false if not.
         */
        static bool checkTypeUniformity(const ExperimentalSetup& setup);
    };
}

#endif