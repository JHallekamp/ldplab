#include <LDPLAB/RayTracingStep/RayTracingStepFactory.hpp>

#include "RayTracingStepCPU/Factory.hpp"
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSOGL
#   include "RayTracingStepCUDA/Factory.hpp"
#endif
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSOGL
#   include "RayTracingStepOpenGL/Factory.hpp"
#endif
#include "../Utils/Log.hpp"
#include "../Utils/Profiler.hpp"

std::shared_ptr<ldplab::IRayTracingStep> ldplab::RayTracingStepFactory::
    createRayTracingStepCPU(
        const ExperimentalSetup& setup,
        const RayTracingStepCPUInfo& info)
{
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtscpu);
    return std::move(rtscpu::Factory::createRTS(setup, info));
}

std::shared_ptr<ldplab::IRayTracingStep> ldplab::RayTracingStepFactory::
    createRayTracingStepCUDA(
        const ExperimentalSetup& setup, 
        const RayTracingStepCUDAInfo& info)
{
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtscuda);
    return std::move(rtscuda::Factory::createRTS(setup, info));
#else
    LDPLAB_LOG_ERROR("RTS factory: Could not create ray tracing step CUDA "\
        "because library has been build without "\
        "LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA");
    return nullptr;
#endif
    return std::shared_ptr<IRayTracingStep>();
}

std::shared_ptr<ldplab::IRayTracingStep>
    ldplab::RayTracingStepFactory::createRayTracingStepOpenGL(
        const ExperimentalSetup& setup, 
        const RayTracingStepOpenGLInfo& info)
{
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSOGL
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtsogl);
    return std::move(rtsogl::Factory::createRTS(setup, info));
#else
    LDPLAB_LOG_ERROR("RTS factory: Could not create ray tracing step OpenGL "\
        "because library has been build without "\
        "LDPLAB_BUILD_OPTION_ENABLE_RTSOGL");
    return nullptr;
#endif
}
