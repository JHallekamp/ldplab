#ifndef WWU_LDPLAB_RTSGPU_OGL_RAY_TRACING_STEP_CPU_HPP
#define WWU_LDPLAB_RTSGPU_OGL_RAY_TRACING_STEP_CPU_HPP

#include "Data.hpp"
#include "..\..\IRayTracingStep.hpp"
#include "..\..\..\Geometry.hpp"

#include <memory>
#include <vector>

namespace ldplab
{
    // Protoype
    class RayTracingStepFactory;

    namespace rtsgpu_ogl
    {
        // Prototype
        struct Context;

        /**
         * @brief Class implementing IRaytracingStep on the CPU. It is non
         *        distributed.
         */
        class RayTracingStep : public IRayTracingStep
        {
            friend RayTracingStepFactory;
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
            Mat4 getRotationMatrix(
                double rx, double ry, double rz, RotationOrder order);
            /**
            * @brief Constructor
            */
            RayTracingStep(std::shared_ptr<Context> context);
        private:
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif
