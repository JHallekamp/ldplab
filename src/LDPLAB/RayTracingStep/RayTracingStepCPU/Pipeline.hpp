#ifndef WWU_LDPLAB_RTSCPU_PIPELINE_HPP
#define WWU_LDPLAB_RTSCPU_PIPELINE_HPP

#include "BufferControl.hpp"
#include "InitialStage.hpp"
#include "InnerParticlePropagationStage.hpp"
#include "RayBoundingVolumeIntersectionTestStage.hpp"
#include "RayParticleInteractionStage.hpp"
#include "RayParticleIntersectionTestStage.hpp"

#include "Context.hpp"
#include "../../ThreadPool.hpp"

#include <memory>
#include <vector>

namespace ldplab
{
    namespace rtscpu
    {
        /**
         * @brief Contains the stages of the ray tracing CPU pipeline.
         * @details Pipelines are meant to be executed in parallel. 
         *          Each individual pipeline has its own state and its own
         *          batch buffers. However, the stages are shared between
         *          all parallel executed pipelines (at least within one
         *          instance of the Pipeline class).
         * @note Creates ldplab::rtscpu::BufferControl instances for each
         *       parallel execution of the pipeline. This is likely to heavily
         *       increase the memory footprint of a Pipeline instance.
         */
        class Pipeline : public ThreadPool::IJob
        {
        public:
            Pipeline(
                std::unique_ptr<IInitialStage> initial,
                std::unique_ptr<IRayBoundingVolumeIntersectionTestStage> rbvit,
                std::unique_ptr<IRayParticleIntersectionTestStage> rpit,
                std::unique_ptr<IRayParticleInteractionStage> rpi,
                std::unique_ptr<IInnerParticlePropagationStage> ipp,
                std::shared_ptr<Context> context);
            /**
             * @brief Sets up the pipeline stages.
             * @note Only called once per ray tracing step execution.
             */
            void setup();
            /** @brief Inherited via IJob */
            void execute(size_t job_id) override;
        private:
            void processBatch(
                RayBuffer& buffer,
                BufferControl& buffer_control);
        private:
            std::unique_ptr<IInitialStage> 
                m_initial_stage;
            std::unique_ptr<IRayBoundingVolumeIntersectionTestStage>
                m_ray_bounding_volume_intersection_test_stage;
            std::unique_ptr<IRayParticleIntersectionTestStage>
                m_ray_particle_intersection_test_stage;
            std::unique_ptr<IRayParticleInteractionStage>
                m_ray_particle_interaction_stage;
            std::unique_ptr<IInnerParticlePropagationStage>
                m_inner_particle_propagation_stage;
            std::vector<BufferControl>
                m_buffer_controls;
        };
    }
}

#endif