#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineHostBound.hpp"

#include <array>
#include <functional>

#include "../../Utils/Log.hpp"
#include "../../Utils/Assert.hpp"
#include "../../Utils/Profiler.hpp"
#include "StageBufferSetup.hpp"
#include "StageGatherOutput.hpp"
#include "StageRayBufferReduce.hpp"

class JobWrapper : public ldplab::utils::ThreadPool::IJob
{
public:
	JobWrapper(std::function<void(size_t)> job) : m_job{ job } { }
	void execute(size_t job_id, size_t batch_size) override { m_job(job_id); }
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
	const static std::shared_ptr<JobWrapper> job = 
		std::make_shared<JobWrapper>(
			std::bind(&createBatchJob, this, std::placeholders::_1));
	m_thread_pool->executeJobBatch(
		job, 
		m_context->simulation_parameter.num_parallel_batches);
}

void ldplab::rtscuda::PipelineHostBound::createBatchJob(size_t process_id)
{
	// Get batch data
	BatchData& batch_data = m_context->batch_data[process_id];
    PipelineData& pipeline_data = m_pipeline_data[process_id];

    // Initial buffer setup
	BufferSetup::executeStepSetup(*m_context, batch_data, pipeline_data);

    constexpr size_t initial_batch_buffer_index = 0;
    bool batches_left = false;
    size_t num_batches = 0;
    do
    {
		batches_left = m_stage_is->execute(
			*m_context, 
			batch_data, 
			initial_batch_buffer_index);
		setupBatch(batch_data);
        executeBatch(
            batch_data,
            pipeline_data,
			num_batches, 
			0, 
			initial_batch_buffer_index, 
			false);
        ++num_batches;
    } while (batches_left);
}

void ldplab::rtscuda::PipelineHostBound::setupBatch(BatchData& batch_data)
{
	m_stage_bvi->batchSetup(*m_context, batch_data);
	m_stage_is->batchSetup(*m_context, batch_data);
	m_stage_ipp->batchSetup(*m_context, batch_data);
	m_stage_pi->batchSetup(*m_context, batch_data);
	m_stage_si->batchSetup(*m_context, batch_data);
}

void ldplab::rtscuda::PipelineHostBound::executeBatch(
	BatchData& batch_data, 
    PipelineData& pipeline_data,
	size_t batch_no, 
	size_t depth, 
	size_t ray_buffer_index, 
	bool inside_particle)
{
    // Prepare buffer
    BufferSetup::executeLayerSetup(
        *m_context,
        batch_data, 
        pipeline_data, 
        ray_buffer_index);

    // Check if buffer contains rays
    PipelineData::RayBufferReductionResult ray_state_count;
    ray_state_count = RayBufferReduce::execute(
        *m_context,
        batch_data, 
        pipeline_data, 
        ray_buffer_index);

    if (ray_state_count.num_active_rays == 0)
        return;

    // Switch between inside and outside particle
    if (inside_particle)
        m_stage_ipp->execute(*m_context, batch_data, ray_buffer_index, ray_buffer_index);
    else
    {
        do
        {
            m_stage_bvi->execute(*m_context, batch_data, ray_buffer_index);
            m_stage_pi->execute(*m_context, batch_data, ray_buffer_index, ray_buffer_index);
            ray_state_count = RayBufferReduce::execute(
                *m_context,
                batch_data, 
                pipeline_data,
                ray_buffer_index);
        } while (ray_state_count.num_world_space_rays > 0);
    }
    // Perform surface interaction and ray branching
    constexpr size_t num_passes = 2;
    for (size_t i = 0; i < num_passes; ++i)
    {
        const bool reflection_pass = (i == 0);
        const bool result_inside_particle = reflection_pass ?
            inside_particle :
            !inside_particle;
        const size_t pass_lim = reflection_pass ?
            m_context->simulation_parameter.num_surface_interaction_reflection_passes :
            m_context->simulation_parameter.num_surface_interaction_transmission_passes;
        for (size_t j = 0; j < pass_lim; ++j)
        {
            m_stage_si->execute(
                *m_context,
                batch_data,
                ray_buffer_index,
                ray_buffer_index + 1,
                ray_buffer_index,
                m_context->simulation_parameter.intensity_cutoff,
                m_context->experimental_setup.medium_reflection_index,
                inside_particle,
                reflection_pass,
                j);
            GatherOutput::execute(
                *m_context, 
                batch_data, 
                pipeline_data, 
                ray_buffer_index);
            if (depth < m_context->simulation_parameter.max_branching_depth)
            {
                executeBatch(
                    batch_data, 
                    pipeline_data,
                    batch_no,
                    depth + 1, 
                    ray_buffer_index + 1, 
                    result_inside_particle);
            }
        }
    }
}

#endif
