#ifndef WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_FACTORY_HPP

#include "..\ExperimentalSetup\ExperimentalSetup.hpp"
#include "RayTracingStepCPU/RayTracingStepCPU.hpp"
#include "RayTracingStepCPUInfo.hpp"
#include "RayTracingStepCPU/Context.hpp"

#include <memory>



namespace ldplab
{
    /**
     * @brief Class building a RayTracingStep.  
     */
    class RayTracingStepFactory
    {
    public:
        static std::shared_ptr<rtscpu::RayTracingStepCPU> 
            createRayTracingStepCPU(
                const ExperimentalSetup& setup,
                const RayTracingStepCPUInfo& info);
    private:
        static void initRodParticleGeometry(
            const ExperimentalSetup& setup,
            std::shared_ptr<rtscpu::Context> context);
        /**
         * @brief Checking if all types in the experimental setup are the same.
         * @param[in] info Structure containing all information about the setup.
         * @returns true if all types are equal and false if not.
         */
        static bool checkTypeUniformity(const ExperimentalSetup& setup);
    };
}

#endif