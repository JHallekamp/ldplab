#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/RayTracingStepOutput.hpp>

#include "PipelineBoundingVolumeIntersection.hpp"
#include "PipelineBufferSetup.hpp"
#include "PipelineGatherOutput.hpp"
#include "PipelineInitial.hpp"
#include "PipelineInnerParticlePropagation.hpp"
#include "PipelineParticleInteraction.hpp"
#include "PipelineParticleIntersection.hpp"
#include "PipelineRayBufferReduce.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Baseclass for a pipeline. */
        class IPipeline
        {
        public:
            /** @brief Type of the pipeline. */
            enum class Type { host_bound, device_bound };
        public:
            /** @brief Creates the pipeline instance. */
            static std::shared_ptr<IPipeline> create(
                const Type pipeline_type,
                const RayTracingStepCUDAInfo& info,
                Context& context,
                std::shared_ptr<PipelineBufferSetup> buffer_setup,
                std::shared_ptr<PipelineGatherOutput> gather_output,
                std::shared_ptr<PipelineRayBufferReduceStage> rbr,
                std::shared_ptr<IPipelineBoundingVolumeIntersectionStage> bvi,
                std::shared_ptr<IPipelineInitialStage> initial,
                std::shared_ptr<IPipelineInnerParticlePropagation> ipp,
                std::shared_ptr<IPipelineParticleInteractionStage> interaction,
                std::shared_ptr<IPipelineParticleIntersectionStage> intersection);
            /** @brief Gets called before execution. */
            virtual void setup();
            /** @brief Executes the pipeline. */
            virtual void execute() = 0;
            /** @brief Returns the exectuion output. */
            virtual bool getOutput(RayTracingStepOutput& output);
        protected:
            IPipeline(Context& ctx) : m_context{ ctx } { }
            /** 
             * @brief Called on pipeline creation after pipeline stages have
             *        been set. 
             * @returns true, if no error occured.
             */
            virtual bool allocate(const RayTracingStepCUDAInfo& info) = 0;
        protected:
            Context& m_context;
            std::shared_ptr<PipelineBufferSetup>
                m_buffer_setup_stage;
            std::shared_ptr<PipelineGatherOutput>
                m_gather_output_stage;
            std::shared_ptr<PipelineRayBufferReduceStage>
                m_ray_buffer_reduce_stage;
            std::shared_ptr<IPipelineBoundingVolumeIntersectionStage> 
                m_bounding_volume_intersection_stage;
            std::shared_ptr<IPipelineInitialStage> 
                m_initial_stage;
            std::shared_ptr<IPipelineInnerParticlePropagation> 
                m_inner_particle_propagation_stage;
            std::shared_ptr<IPipelineParticleInteractionStage> 
                m_particle_interaction_stage;
            std::shared_ptr<IPipelineParticleIntersectionStage> 
                m_particle_intersection_stage;
        };

        /** @brief Host-bound pipeline. */
        class HostPipeline : public IPipeline
        {
        public:
            HostPipeline(Context& ctx);
            /** @brief Inherited via ldplab::rtscuda::IPipeline */
            void execute() override;
        protected:
            /** @brief Inherited via ldplab::rtscuda::IPipeline */
            bool allocate(const RayTracingStepCUDAInfo& info) override;
        private:
            /** @brief Recursive pipeline execution. */
            void executeBatch(
                size_t depth,
                size_t ray_buffer_index,
                bool inside_particle);
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELEINE_HPP