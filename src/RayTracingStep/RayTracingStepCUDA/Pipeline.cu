#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "Pipeline.hpp"

#include "Context.hpp"
#include "../../Utils/Log.hpp"
#include "../../Utils/Assert.hpp"
#include "../../Utils/Profiler.hpp"

std::unique_ptr<ldplab::rtscuda::IPipeline> ldplab::rtscuda::IPipeline::create(
    const Type pipeline_type, 
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info, 
    Context& context)
{
    std::unique_ptr<IPipeline> pipeline;
    if (pipeline_type == Type::host_bound)
        pipeline = std::make_unique<HostPipeline>(context);
    else
    {
        LDPLAB_LOG_ERROR("RTSCUDA context %i: Pipeline creation failed, "\
            "unsupported pipeline type",
            context.uid);
        return nullptr;
    }
    // Write stages
    pipeline->m_buffer_setup_stage = 
        std::make_shared<PipelineBufferSetup>(context);
    pipeline->m_gather_output_stage = 
        std::make_shared<PipelineGatherOutput>(context);
    pipeline->m_ray_buffer_reduce_stage =
        std::make_shared<PipelineRayBufferReduceStage>(context);
    pipeline->m_bounding_volume_intersection_stage =
        IPipelineBoundingVolumeIntersectionStage::createInstance(info, context);
    pipeline->m_initial_stage =
        IPipelineInitialStage::createInstance(setup, info, context);
    pipeline->m_inner_particle_propagation_stage =
        IPipelineInnerParticlePropagation::createInstance(info, context);
    pipeline->m_particle_interaction_stage =
        IPipelineParticleInteractionStage::createInstance(info, context);
    pipeline->m_particle_intersection_stage =
        IPipelineParticleIntersectionStage::createInstance(info, context);
    // Allocate stage
    if (!pipeline->allocate(info))
        return nullptr;
    return std::move(pipeline);
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

    // Initial buffer setup
    m_buffer_setup_stage->executeInitial();

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
        executeBatch(num_batches, 0, initial_batch_buffer_index, false);
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
    size_t batch_no,
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
    m_gather_output_stage->execute(ray_buffer_index);

    //std::vector<Vec3> output(m_context.parameters.num_rays_per_batch);
    //m_context.resources.output_buffer.force_per_ray.download(output.data());
    //std::vector<int32_t> index1(m_context.parameters.num_rays_per_batch);
    //std::vector<int32_t> index2(m_context.parameters.num_rays_per_batch);
    //m_context.resources.ray_buffer.index_buffers[reflection_buffer_index].download(index1.data());
    //m_context.resources.ray_buffer.index_buffers[ray_buffer_index].download(index2.data());

    // Branch
    if (depth < m_context.parameters.max_branching_depth)
    {
        executeBatch(batch_no, depth + 1, reflection_buffer_index, inside_particle);
        executeBatch(batch_no, depth + 1, transmission_buffer_index, !inside_particle);
    }
}

#endif