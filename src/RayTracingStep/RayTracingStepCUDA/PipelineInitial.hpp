#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_INITIAL_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_INITIAL_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <memory>

namespace ldplab
{
    namespace rtscuda
    {
        // Prototype
        struct Context;

        /** @brief Typedefinition of initial stage. */
        typedef bool (*pipelineInitialStageKernel_t)(
            /** @todo Declare parameters */);

        /** @brief Abstract baseclass for the initial stage. */
        class IPipelineInitialStage
        {
        public:
            virtual ~IPipelineInitialStage() { }
            /** @brief Creates an instance of a pipeline stage implementation. */
            static std::shared_ptr<IPipelineInitialStage> createInstance(
                    const RayTracingStepCUDAInfo& info, Context& context);
            /** @brief Gets called before the pipeline enters execution. */
            virtual void setup() { }
            /** @brief Provides the caller with a pointer to the kernel. */
            virtual pipelineInitialStageKernel_t getKernel() = 0;
            /** @brief Fills the initial batch buffer with rays. */
            virtual bool execute(size_t initial_ray_buffer_index) = 0;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELINE_INITIAL_HPP