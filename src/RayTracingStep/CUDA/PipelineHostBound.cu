#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineHostBound.hpp"

#include <array>
#include <functional>

#include "../../Utils/Log.hpp"
#include "../../Utils/Assert.hpp"
#include "../../Utils/Profiler.hpp"
#include "StageBufferSetup.hpp"
#include "StageBufferReorder.hpp"
#include "StageBufferSort.hpp"
#include "StageGatherOutput.hpp"
#include "StageRayBufferReduce.hpp"

class JobWrapper : public ldplab::utils::ThreadPool::IJob
{
public:
	JobWrapper(std::function<void(size_t)> job) : m_job{ job } { }
	void execute(
        size_t job_id, 
        size_t batch_size, 
        size_t thread_id) override 
    { m_job(job_id); }
private:
	std::function<void(size_t)> m_job;
};

ldplab::rtscuda::PipelineHostBound::PipelineHostBound(
	std::shared_ptr<utils::ThreadPool> thread_pool)
	:
	m_thread_pool{ thread_pool }
{ }

void ldplab::rtscuda::PipelineHostBound::execute()
{
    using namespace std::placeholders;
    std::atomic_size_t batch_ctr{ 0 };
	const std::shared_ptr<JobWrapper> job = 
		std::make_shared<JobWrapper>(
			std::bind(&PipelineHostBound::createBatchJob, this, _1, &batch_ctr));
	m_thread_pool->executeJobBatch(
		job, 
		m_context->execution_model.stream_contexts.size());
}

void ldplab::rtscuda::PipelineHostBound::createBatchJob(
    size_t job_id, 
    std::atomic_size_t* batch_no)
{
	// Get batch data
	StreamContext& stream_context = m_context->execution_model.stream_contexts[job_id];
    PipelineData& pipeline_data = m_pipeline_data[job_id];

    // Set device id
    stream_context.deviceContext().activateDevice();

    // Initial buffer setup
    BufferSetup::executeStepSetup(stream_context, pipeline_data);

    constexpr size_t initial_batch_buffer_index = 0;
    bool batches_left = false;
    do
    {
        const size_t current_batch = (*batch_no)++;
        batches_left = m_stage_is->execute(
            stream_context,
            current_batch,
            initial_batch_buffer_index);
        if (batches_left)
        {
            setupBatch(
                stream_context,
                current_batch);
            executeBatch(
                stream_context,
                pipeline_data,
                current_batch,
                0,
                initial_batch_buffer_index,
                stream_context.simulationParameter().num_rays_per_batch,
                false);
        }
    } while (batches_left);
    stream_context.synchronizeOnStream();
    stream_context.deviceContext().synchronizeOnDevice();
}

void ldplab::rtscuda::PipelineHostBound::setupBatch(
    StreamContext& stream_context, size_t batch_no)
{
	m_stage_bvi->batchSetup(*m_context, stream_context, batch_no);
	m_stage_is->batchSetup(*m_context, stream_context, batch_no);
	m_stage_ipp->batchSetup(*m_context, stream_context, batch_no);
	m_stage_pi->batchSetup(*m_context, stream_context, batch_no);
	m_stage_si->batchSetup(*m_context, stream_context, batch_no);
}

void ldplab::rtscuda::PipelineHostBound::executeBatch(
	StreamContext& stream_context, 
    PipelineData& pipeline_data,
	size_t batch_no, 
	size_t depth, 
	size_t ray_buffer_index, 
    size_t num_rays,
	bool inside_particle)
{
    // Check if buffer contains rays
    PipelineData::RayStateCountingResult ray_state_count;
    ray_state_count = RayStateCounting::execute(
        stream_context,
        pipeline_data, 
        ray_buffer_index,
        num_rays);

    if (ray_state_count.num_active_rays == 0)
        return;

    size_t threshold = static_cast<size_t>(
        static_cast<double>(num_rays) * 
        stream_context.simulationParameter().buffer_reorder_threshold);
    if (ray_state_count.num_active_rays < threshold &&
        num_rays > stream_context.simulationParameter().buffer_min_size)
    {
        num_rays = BufferReorder::execute(
            stream_context,
            pipeline_data,
            ray_buffer_index,
            ray_state_count.num_active_rays,
            num_rays,
            false);
    }

    // Prepare buffer
    BufferSetup::executeLayerSetup(
        stream_context,
        pipeline_data, 
        ray_buffer_index,
        ray_buffer_index,
        num_rays);

    // Switch between inside and outside particle
    if (inside_particle)
    {
        m_stage_ipp->execute(
            stream_context,
            ray_buffer_index, 
            ray_buffer_index, 
            ray_buffer_index,
            num_rays);
    }
    else
    {
        do
        {
            m_stage_bvi->execute(
                stream_context,
                ray_buffer_index,
                num_rays);
            m_stage_pi->execute(
                stream_context,
                ray_buffer_index, 
                ray_buffer_index,
                num_rays);
            ray_state_count = RayStateCounting::execute(
                stream_context,
                pipeline_data,
                ray_buffer_index,
                num_rays);
            if (ray_state_count.num_active_rays == 0)
                return;
        } while (ray_state_count.num_world_space_rays > 0);

        threshold = static_cast<size_t>(
            static_cast<double>(num_rays) *
            stream_context.simulationParameter().buffer_reorder_threshold);
        if (ray_state_count.num_active_rays < threshold &&
            num_rays > stream_context.simulationParameter().buffer_min_size)
        {
            num_rays = BufferReorder::execute(
                stream_context,
                pipeline_data,
                ray_buffer_index,
                ray_state_count.num_active_rays,
                num_rays,
                true);
        }
        
        if (stream_context.simulationParameter().sort_ray_buffer)
        {
            BufferSort::execute(
                stream_context,
                pipeline_data,
                ray_buffer_index,
                num_rays,
                true);
        }
    }

    // Perform surface interaction and ray branching
    constexpr std::array<bool, 2> passes{ true, false };
    for (size_t i = 0; i < passes.size(); ++i)
    {
        const bool reflection_pass = passes[i];
        const size_t pass_lim = reflection_pass ?
            m_context->simulation_parameter.num_surface_interaction_reflection_passes :
            m_context->simulation_parameter.num_surface_interaction_transmission_passes;
        for (size_t j = 0; j < pass_lim; ++j)
        {
            m_stage_si->execute(
                stream_context,
                ray_buffer_index,
                ray_buffer_index + 1,
                ray_buffer_index,
                ray_buffer_index,
                m_context->simulation_parameter.intensity_cutoff,
                m_context->experimental_setup.medium_reflection_index,
                inside_particle,
                reflection_pass,
                j,
                num_rays);
            if (depth < m_context->simulation_parameter.max_branching_depth)
            {
                executeBatch(
                    stream_context,
                    pipeline_data,
                    batch_no,
                    depth + 1, 
                    ray_buffer_index + 1, 
                    num_rays,
                    reflection_pass ? inside_particle : !inside_particle);
            }
        }
    }

    GatherOutput::execute(
        stream_context,
        pipeline_data,
        ray_buffer_index,
        ray_buffer_index,
        num_rays);
}

#endif
