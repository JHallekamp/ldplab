#ifndef WWU_LDPLAB_RTSGPU_OGL_PIPELINE_HPP
#define WWU_LDPLAB_RTSGPU_OGL_PIPELINE_HPP

#include "BufferControl.hpp"
#include "InitialStage.hpp"
#include "InnerParticlePropagationStage.hpp"
#include "RayBoundingVolumeIntersectionTestStage.hpp"
#include "RayParticleInteractionStage.hpp"
#include "RayParticleIntersectionTestStage.hpp"

#include <LDPLAB/RayTracingStep/RayTracingStepOutput.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepGPUOpenGLInfo.hpp>
#include "../../../Utils/ThreadPool.hpp"

#include <memory>
#include <vector>

namespace ldplab
{
    namespace rtsgpu_ogl
    {
        // Prototype
        struct Context;

        /**
         * @brief Contains the stages of the ray tracing CPU pipeline.
         * @details Pipelines are meant to be executed in parallel. 
         *          Each individual pipeline has its own state and its own
         *          batch buffers. However, the stages are shared between
         *          all parallel executed pipelines (at least within one
         *          instance of the Pipeline class).
         * @note Creates ldplab::rtsgpu_ogl::BufferControl instances for each
         *       parallel execution of the pipeline. This is likely to heavily
         *       increase the memory footprint of a Pipeline instance.
         */
        class Pipeline : public utils::ThreadPool::IJob
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
             * @brief Initializes the shader.
             * @returns true, if the initialization succeeds.
             */
            bool initShaders();
            /**
             * @brief Sets up the pipeline stages.
             * @note Only called once per ray tracing step execution.
             */
            void setup();
            /**
             * @brief Adding output form all batches together and writing it 
             *        to the ldplab::RayTracingStepOutput.
             */
            void finalizeOutput(RayTracingStepOutput& output);
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
            std::shared_ptr<Context> m_context;
            std::vector<BufferControl>
                m_buffer_controls;
        };
    }
}

#endif