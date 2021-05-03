#ifndef WWU_LDPLAB_RTSOGL_RAY_TRACING_STEP_CPU_HPP
#define WWU_LDPLAB_RTSOGL_RAY_TRACING_STEP_CPU_HPP

#include <LDPLAB/RayTracingStep/IRayTracingStep.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepOpenGLInfo.hpp>
#include <LDPLAB/Geometry.hpp>

#include <memory>
#include <vector>

namespace ldplab
{
    namespace rtsogl
    {
        // Prototype
        struct Context;

        /**
         * @brief Class implementing IRaytracingStep on the CPU. It is non
         *        distributed.
         */
        class RayTracingStep : public IRayTracingStep
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
            /**
             * @brief Updates the internal data buffers.
             * @note This is called automatically on execute so you usually
             *       do not have to call it manually.
             */
            void updateContext(const SimulationState& input);
            /** @brief Initializes the GPU part of the ray tracing step. */
            bool initGPU(const RayTracingStepOpenGLInfo& info);
        private:
            Mat3 getRotationMatrix(
                double rx, double ry, double rz, RotationOrder order);
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
