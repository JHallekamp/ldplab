#ifndef WWU_LDPLAB_RTSCUDA_I_PIPELINE_HPP
#define WWU_LDPLAB_RTSCUDA_I_PIPELINE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepOutput.hpp>
#include <LDPLAB/SimulationState.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        class Factory;
        class IPipeline
        {
        public:
            bool stepSetup(const SimulationState& sim_state);
            bool finalizeOutput(RayTracingStepOutput& output);
            virtual void execute() = 0;
        private:
            std::unique_ptr<GlobalData> m_context;
            ldplab::Mat3 getRotationMatrix(
                double rx, double ry, double rz, RotationOrder order);
        };
    }
}

#endif
#endif