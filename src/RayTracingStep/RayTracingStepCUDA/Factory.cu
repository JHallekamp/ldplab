#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "Factory.hpp"

std::shared_ptr<ldplab::IRayTracingStep> ldplab::rtscuda::Factory::createRTS(
    const ExperimentalSetup& setup, 
    const RayTracingStepCUDAInfo& info)
{
    return std::shared_ptr<IRayTracingStep>();
}

#endif