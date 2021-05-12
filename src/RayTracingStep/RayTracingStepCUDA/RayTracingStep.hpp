#ifndef WWU_LDPLAB_RTSCUDA_RAY_TRACING_STEP_HPP
#define WWU_LDPLAB_RTSCUDA_RAY_TRACING_STEP_HPP

#include "Data.hpp"
#include <LDPLAB/RayTracingStep/IRayTracingStep.hpp>
#include <LDPLAB/Geometry.hpp>

#include <memory>
#include <vector>

namespace ldplab
{
    namespace rtscuda
    {
        // Prototypes
        class Factory;
        struct Context;

        /**
         * @brief Class implementing IRaytracingStep on the CPU. It is non
         *        distributed.
         */
        class RayTracingStep : public IRayTracingStep
        {
            friend Factory;
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
            /** 
             * @brief Updates the internal data buffers. 
             * @note This is called automatically on execute so you usually
             *       do not have to call it manually.
             */
            void updateContext(const SimulationState& input);
        private:
            ldplab::Mat3 getRotationMatrix(
                real_t rx, real_t ry, real_t rz, RotationOrder order);
            /**
            * @brief Constructor
            */
            RayTracingStep(std::unique_ptr<Context> context);
        private:
            std::unique_ptr<Context> m_context;
        };
    }
}

#endif
