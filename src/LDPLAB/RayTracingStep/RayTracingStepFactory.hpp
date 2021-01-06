#ifndef WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP

#include "..\ExperimentalSetup\ExperimentalSetup.hpp"
#include "RayTracingStepCPUInfo.hpp"
#include "RayTracingStepCPU/Context.hpp"
#include "RayTracingStepCPU/RayTracingStep.hpp"
#include "RayTracingStepGPUOpenGLInfo.hpp"
#include "RayTracingStepGPU/OpenGL/Context.hpp"
#include "RayTracingStepGPU/OpenGL/RayTracingStep.hpp"

#include <memory>

namespace ldplab
{
    /**
     * @brief Class building a RayTracingStep.  
     */
    class RayTracingStepFactory
    {
    public:
        static std::shared_ptr<rtscpu::RayTracingStep> 
            createRayTracingStepCPU(
                const ExperimentalSetup& setup,
                const RayTracingStepCPUInfo& info);
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