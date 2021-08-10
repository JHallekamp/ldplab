#ifndef WWU_LDPLAB_RTSCUDA_RAY_TRACING_STEP_CUDA_HPP
#define WWU_LDPLAB_RTSCUDA_RAY_TRACING_STEP_CUDA_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/IRayTracingStep.hpp>
#include <LDPLAB/UID.hpp>
#include <LDPLAB/UID.hpp>

#include "IPipeline.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class RayTracingStepCUDA : public IRayTracingStep
        {
            friend class Factory;
        public:
            /**
             * @brief Inherited via IRayTracingStep. Starts the ray tracing
             *        simulation.
             * @param[in] input The current simulation state.
             * @param[out] output Results of the ray tracing step execution.
             */
            void execute(
                const SimulationState& input,
                RayTracingStepOutput& output) override;
        private:
            std::unique_ptr<IPipeline> m_pipeline;
        };
    }
}

#endif
#endif