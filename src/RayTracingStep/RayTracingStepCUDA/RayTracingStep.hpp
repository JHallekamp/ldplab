#ifndef WWU_LDPLAB_RTSCUDA_RAY_TRACING_STEP_HPP
#define WWU_LDPLAB_RTSCUDA_RAY_TRACING_STEP_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include "Data.hpp"
#include <LDPLAB/RayTracingStep/IRayTracingStep.hpp>
#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        // Prototypes
        class Factory;
        struct Context;

        /** @brief Class implementing IRaytracingStep using CUDA. */
        class RayTracingStep : public IRayTracingStep
        {
            friend Factory;
        public:
            /** @brief Inherited via ldplab::IRayTracingStep */
            virtual void execute(
                const SimulationState& input,
                RayTracingStepOutput& output) override;
        private:
            bool updateContext(const SimulationState& input);
            ldplab::Mat3 getRotationMatrix(
                double rx, double ry, double rz, RotationOrder order);
        private:
            std::unique_ptr<Context> m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_RAY_TRACING_STEP_HPP