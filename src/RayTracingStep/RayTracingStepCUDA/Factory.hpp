#ifndef WWU_LDPLAB_RTSCUDA_FACTORY_HPP
#define WWU_LDPLAB_RTSCUDA_FACTORY_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/ExperimentalSetup/ExperimentalSetup.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <LDPLAB/RayTracingStep/IRayTracingStep.hpp>
#include <memory>

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Factory class to create the ray tracing step. */
        class Factory
        {
        public:
            /** @brief Creates the ray tracing step instance */
            static std::shared_ptr<IRayTracingStep> createRTS(
                const ExperimentalSetup& setup,
                const RayTracingStepCUDAInfo& info);
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_FACTORY_HPP