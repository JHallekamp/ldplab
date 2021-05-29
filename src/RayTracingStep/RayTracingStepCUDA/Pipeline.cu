#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "Pipeline.hpp"

#include "Context.hpp"
#include "../../Utils/Log.hpp"
#include "../../Utils/Assert.hpp"
#include "../../Utils/Profiler.hpp"

std::shared_ptr<ldplab::rtscuda::IPipeline> ldplab::rtscuda::IPipeline::create(
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
    std::shared_ptr<IPipelineParticleIntersectionStage> intersection)
{
    std::shared_ptr<IPipeline> pipeline;
    if (pipeline_type == Type::host_bound)
    {
        LDPLAB_LOG_ERROR("RTSCUDA context %i: Pipeline creation failed, "\
            "unsupported pipeline type",
            context.uid);
        return nullptr;
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCUDA context %i: Pipeline creation failed, "\
            "unsupported pipeline type",
            context.uid);
        return nullptr;
    }
    // Write stages
    pipeline->m_buffer_setup_stage = std::move(buffer_setup);
    pipeline->m_gather_output_stage = std::move(gather_output);
    pipeline->m_ray_buffer_reduce_stage = std::move(rbr);
    pipeline->m_bounding_volume_intersection_stage = std::move(bvi);
    pipeline->m_initial_stage = std::move(initial);
    pipeline->m_inner_particle_propagation_stage = std::move(ipp);
    pipeline->m_particle_interaction_stage = std::move(interaction);
    pipeline->m_particle_intersection_stage = std::move(intersection);
    // Allocate stage
    if (!pipeline->allocate(info))
        return nullptr;
    return pipeline;
}

void ldplab::rtscuda::IPipeline::setup()
{
    m_bounding_volume_intersection_stage->setup();
    m_initial_stage->setup();
    m_inner_particle_propagation_stage->setup();
    m_particle_interaction_stage->setup();
    m_particle_intersection_stage->setup();
}

bool ldplab::rtscuda::IPipeline::getOutput(RayTracingStepOutput& output)
{
    // Download output data
    if (!m_context.resources.output_buffer.force_per_particle.download(
        m_context.resources.output_buffer.host_force_per_particle.data()))
    {
        LDPLAB_LOG_ERROR("RTSCUDA context %i: Failed to download force "\
            "output from device",
            m_context.uid);
        return false;
    }
    if (!m_context.resources.output_buffer.torque_per_particle.download(
        m_context.resources.output_buffer.host_torque_per_particle.data()))
    {
        LDPLAB_LOG_ERROR("RTSCUDA context %i: Failed to download torque "\
            "output from device",
            m_context.uid);
        return false;
    }
    // Map from internal index to uid
    for (size_t p = 0; p < m_context.parameters.num_particles; ++p)
    {
        UID<Particle> puid =
            m_context.interface_mapping.particle_index_to_uid[p];
        output.force_per_particle[puid] =
            m_context.resources.output_buffer.host_force_per_particle[p];
        output.torque_per_particle[puid] =
            m_context.resources.output_buffer.host_torque_per_particle[p];
    }
    return true;
}

ldplab::rtscuda::HostPipeline::HostPipeline(Context& ctx)
    :
    IPipeline{ ctx }
{ }

void ldplab::rtscuda::HostPipeline::execute()
{
    LDPLAB_LOG_DEBUG("RTSCUDA context %i: Ray tracing step executes pipeline",
        m_context.uid);

    constexpr size_t initial_batch_buffer_index = 0;
    bool batches_left = false;
    size_t num_batches = 0;
    do
    {
        LDPLAB_PROFILING_START(rtscuda_pipeline_initial_batch_creation);
        batches_left = m_initial_stage->execute(initial_batch_buffer_index);
        LDPLAB_PROFILING_STOP(rtscuda_pipeline_initial_batch_creation);
        LDPLAB_LOG_TRACE("RTSCUDA context %i: Pipeline executes batch %i",
            m_context.uid, num_batches);
        executeBatch(0, initial_batch_buffer_index, false);
        LDPLAB_LOG_TRACE("RTSCUDA context %i: Pipeline finished batch execution %i",
            m_context.uid, num_batches);
        ++num_batches;
    } while(batches_left);

    LDPLAB_LOG_DEBUG("RTSCUDA context %i: Ray tracing step pipeline "\
        "execution finished",
        m_context.uid);
}

bool ldplab::rtscuda::HostPipeline::allocate(const RayTracingStepCUDAInfo& info)
{
    return true;
}

void ldplab::rtscuda::HostPipeline::executeBatch(
    size_t depth, 
    size_t ray_buffer_index, 
    bool inside_particle)
{
    // Prepare buffer
    m_buffer_setup_stage->execute();
    RayBufferReduceResult ray_state_count;
    // Check if buffer contains rays
    ray_state_count = m_ray_buffer_reduce_stage->execute(ray_buffer_index);
    if (ray_state_count.num_active_rays == 0)
        return;
    // Switch between inside and outside particle
    if (inside_particle)
        m_inner_particle_propagation_stage->execute(ray_buffer_index);
    else
    {
        do
        {
            m_bounding_volume_intersection_stage->execute(ray_buffer_index);
            m_particle_intersection_stage->execute(ray_buffer_index);
            ray_state_count = m_ray_buffer_reduce_stage->execute(ray_buffer_index);
        } while (ray_state_count.num_world_space_rays > 0);
    }
    // Perform particle interaction
    size_t reflection_buffer_index = depth * 2 + 1;
    size_t transmission_buffer_index = depth * 2 + 2;
    m_particle_interaction_stage->execute(
        inside_particle, 
        ray_buffer_index, 
        reflection_buffer_index, 
        transmission_buffer_index);
    // Gather output
    m_gather_output_stage->execute();

    // Branch
    if (depth < m_context.parameters.max_branching_depth)
    {
        executeBatch(depth + 1, reflection_buffer_index, inside_particle);
        executeBatch(depth + 1, transmission_buffer_index, !inside_particle);
    }
}

#endif