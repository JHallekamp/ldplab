#ifndef WWU_LDPLAB_RTSCUDA_I_PIPELINE_HPP
#define WWU_LDPLAB_RTSCUDA_I_PIPELINE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepOutput.hpp>
#include <LDPLAB/SimulationState.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IBoundingVolumeIntersection.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IInitialStage.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IInnerParticlePropagation.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IParticleIntersection.hpp>
#include <LDPLAB/RayTracingStep/CUDA/ISurfaceInteraction.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        class Factory;
        class IPipeline
        {
            friend Factory;
        public:
            bool stepSetup(const SimulationState& sim_state);
            bool finalizeOutput(RayTracingStepOutput& output);
            virtual void execute() = 0;
        private:
            ldplab::Mat3 getRotationMatrix(
                double rx, double ry, double rz, RotationOrder order);
        protected:
            std::unique_ptr<GlobalData> m_context;
            std::shared_ptr<IBoundingVolumeIntersection> m_stage_bvi;
            std::shared_ptr<IInitialStage> m_stage_is;
            std::shared_ptr<IInnerParticlePropagation> m_stage_ipp;
            std::shared_ptr<IParticleIntersection> m_stage_pi;
            std::shared_ptr<ISurfaceInteraction> m_stage_si;
        };
    }
}

#endif
#endif