#ifndef WWU_LDPLAB_RAY_TRACING_STEP_RAY_TRACING_STEP_INTERFACE_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_RAY_TRACING_STEP_INTERFACE_HPP

#include "RayTracingStepOutput.hpp"
#include "../SimulationState.hpp"

namespace ldplab
{
    /**
     * @brief Generic interface class which provides the simulation a 
     *        calculation of the light induced force on the particles via 
     *        dynamic ray tracing. 
     */
    class IRayTracingStep
    {
    public:
        /**
         * @brief The destructor is virtual since class inhered from 
         *        IRayTracingStep.
         */
        virtual ~IRayTracingStep() { };
        /**
         * @brief Begin ray tracing simulation.
         * @param[in] input The current simulation state.
         * @param[out] output Results of the ray tracing step execution.
         */
        virtual void execute(
            const SimulationState& input,
            RayTracingStepOutput& output) = 0;
    };
}

#endif 
