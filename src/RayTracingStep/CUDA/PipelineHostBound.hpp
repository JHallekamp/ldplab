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

#include "../../Utils/ThreadPool.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class PipelineHostBound : public IPipeline
        {
        public:
            PipelineHostBound(std::shared_ptr<utils::ThreadPool> thread_pool);
            void execute() override;
        private:
            void createBatchJob(size_t process_id);
            void setupBatch(BatchData& batch_data);
            void executeBatch(
                BatchData& batch_data,
                size_t batch_no,
                size_t depth,
                size_t ray_buffer_index,
                bool inside_particle);
        private:
            std::shared_ptr<utils::ThreadPool> m_thread_pool;
        };
    }
}

#endif
#endif