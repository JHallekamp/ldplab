#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CPU_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CPU_HPP

#include "..\IRayTracingStep.hpp"
#include "..\Geometry.hpp"
#include "RayBuffer.hpp"

#include <memory>

namespace ldplab
{
    /**
     * @brief Class implementing IRaytracingStep on the CPU. It is non deferred.
     */
    class RayTracingStepCPU : IRayTracingStep
    {
    public:
        /**
         * @brief Constructor
         */
        RayTracingStepCPU();
        /**
         * @brief Inherited via IRayTracingStep. Starts the ray tracing 
         *        simulation.
         */
        void start();
    private:
        std::shared_ptr<RayBuffer> m_inital_ray_buffer;
        std::shared_ptr<RayBuffer[]> m_ray_batches;
        size_t m_max_iteration_deep;
    };
}

#endif
