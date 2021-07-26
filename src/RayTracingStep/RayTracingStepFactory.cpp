#include <LDPLAB/RayTracingStep/RayTracingStepFactory.hpp>

#include "CPU/Factory.hpp"
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#   include "CUDA/Factory.hpp"
#endif
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSOGL
#   include "OpenGL/Factory.hpp"
#endif
#include "../Utils/Log.hpp"
#include "../Utils/Profiler.hpp"

std::shared_ptr<ldplab::IRayTracingStep> 
    ldplab::RayTracingStepFactory::createRayTracingStepCPU(
        const RayTracingStepCPUInfo& info, 
        ExperimentalSetup&& setup)
{
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtscpu);
    rtscpu::Factory rts_factory;
    return std::move(rts_factory.createRTS(info, std::move(setup)));
}

std::shared_ptr<ldplab::IRayTracingStep> 
    ldplab::RayTracingStepFactory::createRayTracingStepCPU(
        const RayTracingStepCPUInfo& info, 
        ExperimentalSetup&& setup, 
        rtscpu::PipelineConfiguration& user_defined_configuration, 
        bool allow_default_stage_overwrites)
{
    LDPLAB_PROFILING_START(ray_tracing_step_factory_create_rtscpu);
    rtscpu::Factory rts_factory;
    return std::move(rts_factory.createRTS(
        info, 
        std::move(setup), 
        user_defined_configuration, 
        allow_default_stage_overwrites));
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
