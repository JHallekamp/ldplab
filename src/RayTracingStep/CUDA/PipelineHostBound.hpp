#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_HOST_BOUND_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_HOST_BOUND_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include "IPipeline.hpp"

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IBoundingVolumeIntersection.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IInitialStage.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IInnerParticlePropagation.hpp>
#include <LDPLAB/RayTracingStep/CUDA/IParticleIntersection.hpp>
#include <LDPLAB/RayTracingStep/CUDA/ISurfaceInteraction.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        class PipelineHostBound : public IPipeline
        {
        public:
            void execute() override;
        };
    }
}

#endif
#endif