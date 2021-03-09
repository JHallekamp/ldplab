#include "RayTracingStepFactory.hpp"

#include "RayTracingStepCPU/Factory.hpp"
#include "RayTracingStepGPU/OpenGL/Factory.hpp"
#include "../Utils/Profiler.hpp"

std::shared_ptr<ldplab::IRayTracingStep> ldplab::RayTracingStepFactory::
    createRayTracingStepCPU(
        const ExperimentalSetup& setup,
        const RayTracingStepCPUInfo& info)
{
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtscpu);
    return rtscpu::Factory::createRTS(setup, info);
}

std::shared_ptr<ldplab::IRayTracingStep> 
    ldplab::RayTracingStepFactory::createRayTracingStepCPUDebug(
        const ExperimentalSetup& setup, 
        const RayTracingStepCPUInfo& info, 
        RayTracingStepCPUDebugInfo& debug)
{
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtscpu);
    return rtscpu::Factory::createRTSDebug(setup, info, debug);
}

std::shared_ptr<ldplab::IRayTracingStep>
    ldplab::RayTracingStepFactory::createRayTracingStepGPUOpenGL(
        const ExperimentalSetup& setup, 
        const RayTracingStepGPUOpenGLInfo& info)
{
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtsgpu_ogl);
    return rtsgpu_ogl::Factory::createRTS(setup, info);
}
