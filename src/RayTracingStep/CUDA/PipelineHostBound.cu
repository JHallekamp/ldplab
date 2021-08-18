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
	void execute(
        size_t job_id, 
        size_t batch_size, 
        size_t thread_id) override 
    { m_job(thread_id); }
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
	const static std::shared_ptr<JobWrapper> job = 
		std::make_shared<JobWrapper>(
			std::bind(&PipelineHostBound::createBatchJob, this, _1));
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
    LDPLAB_PROFILING_START(pipeline_execute_step_buffer_setup);
    BufferSetup::executeStepSetup(*m_context, batch_data, pipeline_data);
    LDPLAB_PROFILING_STOP(pipeline_execute_step_buffer_setup);

    constexpr size_t initial_batch_buffer_index = 0;
    bool batches_left = false;
    size_t num_batches = 0;
    do
    {
        LDPLAB_PROFILING_START(pipeline_create_batch);
		batches_left = m_stage_is->execute(
			*m_context, 
			batch_data, 
			initial_batch_buffer_index);
        LDPLAB_PROFILING_STOP(pipeline_create_batch);
        LDPLAB_PROFILING_START(pipeline_setup_batch);
        setupBatch(batch_data);
        LDPLAB_PROFILING_STOP(pipeline_setup_batch);
        LDPLAB_PROFILING_START(pipeline_execute_batch);
        executeBatch(
            batch_data,
            pipeline_data,
			num_batches, 
			0, 
			initial_batch_buffer_index, 
			false);
        LDPLAB_PROFILING_STOP(pipeline_execute_batch);
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
    // Check if buffer contains rays
    LDPLAB_PROFILING_START(pipeline_ray_buffer_reduce);
    PipelineData::RayBufferReductionResult ray_state_count;
    ray_state_count = RayBufferReduce::execute(
        *m_context,
        batch_data, 
        pipeline_data, 
        ray_buffer_index);
    LDPLAB_PROFILING_STOP(pipeline_ray_buffer_reduce);

    if (ray_state_count.num_active_rays == 0)
        return;

    // Prepare buffer
    LDPLAB_PROFILING_START(pipeline_execute_layer_buffer_setup);
    BufferSetup::executeLayerSetup(
        *m_context,
        batch_data, 
        pipeline_data, 
        ray_buffer_index,
        ray_buffer_index);
    LDPLAB_PROFILING_STOP(pipeline_execute_layer_buffer_setup);

    // Switch between inside and outside particle
    if (inside_particle)
    {
        LDPLAB_PROFILING_START(pipeline_inner_particle_propagation);
        m_stage_ipp->execute(
            *m_context, 
            batch_data, 
            ray_buffer_index, 
            ray_buffer_index, 
            ray_buffer_index);
        LDPLAB_PROFILING_STOP(pipeline_inner_particle_propagation);
    }
    else
    {
        do
        {
            LDPLAB_PROFILING_START(pipeline_bounding_volume_intersection);
            m_stage_bvi->execute(
                *m_context, 
                batch_data, 
                ray_buffer_index);
            LDPLAB_PROFILING_STOP(pipeline_bounding_volume_intersection);
            LDPLAB_PROFILING_START(pipeline_particle_intersection);
            m_stage_pi->execute(
                *m_context, 
                batch_data, 
                ray_buffer_index, 
                ray_buffer_index);
            LDPLAB_PROFILING_STOP(pipeline_particle_intersection);
            LDPLAB_PROFILING_START(pipeline_ray_buffer_reduce);
            ray_state_count = RayBufferReduce::execute(
                *m_context,
                batch_data, 
                pipeline_data,
                ray_buffer_index);
            LDPLAB_PROFILING_STOP(pipeline_ray_buffer_reduce);
            if (ray_state_count.num_active_rays == 0)
                return;
        } while (ray_state_count.num_world_space_rays > 0);
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
            LDPLAB_PROFILING_START(pipeline_surface_interaction);
            m_stage_si->execute(
                *m_context,
                batch_data,
                ray_buffer_index,
                ray_buffer_index + 1,
                ray_buffer_index,
                ray_buffer_index,
                m_context->simulation_parameter.intensity_cutoff,
                m_context->experimental_setup.medium_reflection_index,
                inside_particle,
                reflection_pass,
                j);
            LDPLAB_PROFILING_STOP(pipeline_surface_interaction);
            if (depth < m_context->simulation_parameter.max_branching_depth)
            {
                executeBatch(
                    batch_data, 
                    pipeline_data,
                    batch_no,
                    depth + 1, 
                    ray_buffer_index + 1, 
                    reflection_pass ? inside_particle : !inside_particle);
            }
        }
    }

    LDPLAB_PROFILING_START(pipeline_gather_output);
    GatherOutput::execute(
        *m_context,
        batch_data,
        pipeline_data,
        ray_buffer_index,
        ray_buffer_index);
    LDPLAB_PROFILING_STOP(pipeline_gather_output);
}

#endif
