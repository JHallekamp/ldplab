#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "Pipeline.hpp"

#include "Context.hpp"
#include "../../Utils/Log.hpp"
#include "../../Utils/Assert.hpp"
#include "../../Utils/Profiler.hpp"

#include <cstdlib>

std::unique_ptr<ldplab::rtscuda::IPipeline> ldplab::rtscuda::IPipeline::create(
    const Type pipeline_type, 
    const ExperimentalSetup& setup,
    const RayTracingStepCUDAInfo& info, 
    Context& context)
{
    std::unique_ptr<IPipeline> pipeline;
    if (pipeline_type == Type::host_bound)
        pipeline = std::make_unique<HostPipeline>(context);
    else if (pipeline_type == Type::device_bound)
        pipeline = std::make_unique<DevicePipeline>(context);
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
    LDPLAB_PROFILING_START(pipeline_initial_buffer_setup);
    m_buffer_setup_stage->executeInitial();
    cudaDeviceSynchronize();
    LDPLAB_PROFILING_STOP(pipeline_initial_buffer_setup);

    constexpr size_t initial_batch_buffer_index = 0;
    bool batches_left = false;
    size_t num_batches = 0;
    do
    {
        LDPLAB_PROFILING_START(pipeline_initial_batch_creation);
        batches_left = m_initial_stage->execute(initial_batch_buffer_index);
        cudaDeviceSynchronize();
        LDPLAB_PROFILING_STOP(pipeline_initial_batch_creation);
        LDPLAB_LOG_TRACE("RTSCUDA context %i: Pipeline executes batch %i",
            m_context.uid, num_batches);

        LDPLAB_PROFILING_START(pipeline_process_batch);
        executeBatch(num_batches, 0, initial_batch_buffer_index, false);
        LDPLAB_PROFILING_STOP(pipeline_process_batch);

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
    LDPLAB_PROFILING_START(pipeline_buffer_setup);
    m_buffer_setup_stage->execute();
    cudaDeviceSynchronize();
    LDPLAB_PROFILING_STOP(pipeline_buffer_setup);

    // Check if buffer contains rays
    LDPLAB_PROFILING_START(pipeline_ray_index_reduction);
    RayBufferReduceResult ray_state_count;
    ray_state_count = m_ray_buffer_reduce_stage->execute(ray_buffer_index);
    cudaDeviceSynchronize();
    LDPLAB_PROFILING_STOP(pipeline_ray_index_reduction);

    if (ray_state_count.num_active_rays == 0)
        return;
    
    // Switch between inside and outside particle
    if (inside_particle)
    {
        LDPLAB_PROFILING_START(pipeline_inner_particle_propagation);
        m_inner_particle_propagation_stage->execute(ray_buffer_index);
        cudaDeviceSynchronize();
        LDPLAB_PROFILING_STOP(pipeline_inner_particle_propagation);
    }
    else
    {
        do
        {
            LDPLAB_PROFILING_START(pipeline_bounding_volume_intersection);
            m_bounding_volume_intersection_stage->execute(ray_buffer_index);
            cudaDeviceSynchronize();
            LDPLAB_PROFILING_STOP(pipeline_bounding_volume_intersection);

            LDPLAB_PROFILING_START(pipeline_particle_intersection);
            m_particle_intersection_stage->execute(ray_buffer_index);
            cudaDeviceSynchronize();
            LDPLAB_PROFILING_STOP(pipeline_particle_intersection);

            LDPLAB_PROFILING_START(pipeline_ray_index_reduction);
            ray_state_count = m_ray_buffer_reduce_stage->execute(ray_buffer_index);
            cudaDeviceSynchronize();
            LDPLAB_PROFILING_STOP(pipeline_ray_index_reduction);
        } while (ray_state_count.num_world_space_rays > 0);
    }
    // Perform particle interaction
    size_t reflection_buffer_index = depth * 2 + 1;
    size_t transmission_buffer_index = depth * 2 + 2;
    LDPLAB_PROFILING_START(pipeline_particle_interaction);
    m_particle_interaction_stage->execute(
        inside_particle, 
        ray_buffer_index, 
        reflection_buffer_index, 
        transmission_buffer_index);
    cudaDeviceSynchronize();
    LDPLAB_PROFILING_STOP(pipeline_particle_interaction);

    // Gather output
    LDPLAB_PROFILING_START(pipeline_gather_output);
    m_gather_output_stage->execute(ray_buffer_index);
    cudaDeviceSynchronize();
    LDPLAB_PROFILING_STOP(pipeline_gather_output);

    // Branch
    if (depth < m_context.parameters.max_branching_depth)
    {
        executeBatch(batch_no, depth + 1, reflection_buffer_index, inside_particle);
        executeBatch(batch_no, depth + 1, transmission_buffer_index, !inside_particle);
    }
}

// ============================================================================
__device__ void executePipelineBatch(
    ldplab::rtscuda::DevicePipelineResources& resources,
    ldplab::rtscuda::PipelineExecuteFunctions& execute_functions,
    char* stack,
    size_t stack_size);
__device__ bool executePipelineSingleStage(
    ldplab::rtscuda::DevicePipelineResources& resources,
    ldplab::rtscuda::PipelineExecuteFunctions& execute_functions,
    size_t ray_buffer_index,
    size_t reflection_buffer_index,
    size_t transmission_buffer_index, 
    bool inside_particle);
__global__ void executePipelineKernel(
    ldplab::rtscuda::DevicePipelineResources resources,
    ldplab::rtscuda::PipelineExecuteFunctions execute_functions);

ldplab::rtscuda::DevicePipeline::DevicePipeline(Context& ctx)
    :
    IPipeline{ ctx }
{ }

bool ldplab::rtscuda::DevicePipeline::allocate(
    const RayTracingStepCUDAInfo& info)
{
    return true;
}

void ldplab::rtscuda::DevicePipeline::execute()
{
    LDPLAB_PROFILING_START(pipeline_resource_setup);

    // Prepare resource structure
    DevicePipelineResources resources;
    
    resources.ray_buffer.indices =
        m_context.resources.ray_buffer.index_buffer_pointers.get();
    resources.ray_buffer.origins =
        m_context.resources.ray_buffer.origin_buffer_pointers.get();
    resources.ray_buffer.directions =
        m_context.resources.ray_buffer.direction_buffer_pointers.get();
    resources.ray_buffer.intensities =
        m_context.resources.ray_buffer.intensity_buffer_pointers.get();
    resources.ray_buffer.min_bv_dists =
        m_context.resources.ray_buffer.min_bv_dist_buffer_pointers.get();

    resources.intersection_buffer.isec_indices =
        m_context.resources.intersection_buffer.intersection_particle_index_buffer.get();
    resources.intersection_buffer.points =
        m_context.resources.intersection_buffer.intersection_point_buffer.get();
    resources.intersection_buffer.normals =
        m_context.resources.intersection_buffer.intersection_normal_buffer.get();

    resources.output_buffer.force_per_particle =
        m_context.resources.output_buffer.force_per_particle.get();
    resources.output_buffer.force_per_ray =
        m_context.resources.output_buffer.force_per_ray.get();
    resources.output_buffer.torque_per_particle =
        m_context.resources.output_buffer.torque_per_particle.get();
    resources.output_buffer.torque_per_ray =
        m_context.resources.output_buffer.torque_per_ray.get();

    resources.transformations.p2w_transformation =
        m_context.resources.transformations.p2w_transformation.get();
    resources.transformations.p2w_translation =
        m_context.resources.transformations.p2w_translation.get();
    resources.transformations.w2p_transformation =
        m_context.resources.transformations.w2p_transformation.get();
    resources.transformations.w2p_translation =
        m_context.resources.transformations.w2p_translation.get();

    resources.bounding_volumes.per_particle =
        m_context.resources.bounding_volumes.bounding_volume_per_particle.get();

    resources.particles.center_of_mass_per_particle =
        m_context.resources.particles.center_of_mass_per_particle.get();
    resources.particles.geometry_per_particle =
        m_context.resources.particles.geometry_per_particle.get();
    resources.particles.material_per_particle =
        m_context.resources.particles.material_per_particle.get();

    resources.reduction.reduction_result_buffer =
        m_context.resources.pipeline.reduction_result_buffer.get();

    resources.parameters.intensity_cutoff =
        m_context.parameters.intensity_cutoff;
    resources.parameters.light_source_resolution_per_world_unit =
        m_context.parameters.light_source_resolution_per_world_unit;
    resources.parameters.max_branching_depth =
        m_context.parameters.max_branching_depth;
    resources.parameters.medium_reflection_index =
        m_context.parameters.medium_reflection_index;
    resources.parameters.num_light_sources =
        m_context.parameters.num_light_sources;
    resources.parameters.num_particles =
        m_context.parameters.num_particles;
    resources.parameters.num_rays_per_batch =
        m_context.parameters.num_rays_per_batch;
    resources.parameters.output_in_particle_space =
        m_context.parameters.output_in_particle_space;

    resources.launch_params.boundingVolumeIntersection =
        m_bounding_volume_intersection_stage->getLaunchParameter();
    resources.launch_params.bufferSetup =
        m_buffer_setup_stage->getLaunchParameterBufferSetup();
    resources.launch_params.createBatch =
        m_initial_stage->getLaunchParameter();
    resources.launch_params.gatherOutput =
        m_gather_output_stage->getLaunchParameter();
    resources.launch_params.initialBufferSetup =
        m_buffer_setup_stage->getLaunchParameterInitialSetup();
    resources.launch_params.innerParticlePropagation =
        m_inner_particle_propagation_stage->getLaunchParameter();
    resources.launch_params.particleInteraction =
        m_particle_interaction_stage->getLaunchParameter();
    resources.launch_params.particleIntersection =
        m_particle_intersection_stage->getLaunchParameter();
    resources.launch_params.rayBufferReduceStep1 =
        m_ray_buffer_reduce_stage->getLaunchParameterStep1();
    resources.launch_params.rayBufferReduceStep2 =
        m_ray_buffer_reduce_stage->getLaunchParameterStep2();

    PipelineExecuteFunctions execute_functions;
    execute_functions.bounding_volume_intersection =
        m_bounding_volume_intersection_stage->getKernel();
    execute_functions.initial =
        m_initial_stage->getKernel();
    execute_functions.inner_particle_propagation =
        m_inner_particle_propagation_stage->getKernel();
    execute_functions.particle_interaction =
        m_particle_interaction_stage->getKernel();
    execute_functions.particle_intersection =
        m_particle_intersection_stage->getKernel();

    LDPLAB_PROFILING_STOP(pipeline_resource_setup);

    LDPLAB_PROFILING_START(pipeline_main_kernel_execution);
    executePipelineKernel<<<1, 1, m_context.parameters.max_branching_depth>>>(
        resources,
        execute_functions);
    cudaDeviceSynchronize();
    LDPLAB_PROFILING_STOP(pipeline_main_kernel_execution);
}

__global__ void executePipelineKernel(
    ldplab::rtscuda::DevicePipelineResources resources,
    ldplab::rtscuda::PipelineExecuteFunctions execute_functions)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;

    // Shared memory
    extern __shared__ char stack[];

    // Execute initial stage
    executeInitialSetupKernel(resources);

    // Execute while there are still batches.
    bool batches_left = false;
    size_t batch_no = 0;
    do
    {
        batches_left = execute_functions.initial(resources, 0, batch_no++);
        executePipelineBatch(
            resources,
            execute_functions,
            stack,
            resources.parameters.max_branching_depth);

    } while (batches_left);
}

__device__ void executePipelineBatch(
    ldplab::rtscuda::DevicePipelineResources& resources, 
    ldplab::rtscuda::PipelineExecuteFunctions& execute_functions,
    char* stack, 
    size_t stack_size)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;

    // Execute first stage
    if (!executePipelineSingleStage(resources, execute_functions, 0, 1, 2, false))
        return;

    // Simulate recursion using stack
    if (stack_size > 0)
    {
        stack[0] = 0;
        size_t depth = 0;
        while (stack[0] < 2)
        {
            // If we are done at this stage, look for parent subtree
            if (stack[depth] >= 2)
            {
                --depth;
                continue;
            }

            // Get the buffers
            const size_t ray_buffer_index = 2 * depth + stack[depth] + 1;
            const size_t reflection_buffer_index = 2 * (depth + 1) + 1;
            const size_t transmission_buffer_index = 2 * (depth + 1) + 2;
            bool inside_particle = false;
            for (size_t i = 0; i <= depth; ++i)
            {
                if (stack[i] > 0)
                    inside_particle = !inside_particle;
            }

            //printf("runs depth %i with stack[", depth + 1);
            //printf("%i] = ", depth);
            //printf("%i on buffer ", stack[depth]);
            //printf("%i\n", ray_buffer_index);

            // At this level, we will later look for another subtree
            ++stack[depth];

            // Execute subtree root
            if (!executePipelineSingleStage(
                resources,
                execute_functions,
                ray_buffer_index,
                reflection_buffer_index,
                transmission_buffer_index,
                inside_particle))
                continue;

            // Setup next subtree
            if (depth + 1 < stack_size)
            {
                ++depth;
                stack[depth] = 0;
            }
        }
    }
}

__device__ bool executePipelineSingleStage(
    ldplab::rtscuda::DevicePipelineResources& resources, 
    ldplab::rtscuda::PipelineExecuteFunctions& execute_functions,
    size_t ray_buffer_index,
    size_t reflection_buffer_index,
    size_t transmission_buffer_index,
    bool inside_particle)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;

    // Setup buffers
    executeBufferSetupKernel(resources);

    // Count rays
    if (executeRayBufferReduceKernel(resources, ray_buffer_index).num_active_rays == 0)
        return false;

    if (inside_particle)
        execute_functions.inner_particle_propagation(resources, ray_buffer_index);
    else
    {
        do
        {
            execute_functions.bounding_volume_intersection(resources, ray_buffer_index);
            execute_functions.particle_intersection(resources, ray_buffer_index);
        } while(executeRayBufferReduceKernel(resources, ray_buffer_index).num_world_space_rays > 0);
    }

    // Perform particle interaction
    execute_functions.particle_interaction(
        resources, 
        inside_particle, 
        ray_buffer_index, 
        reflection_buffer_index, 
        transmission_buffer_index);

    // Gather output
    executeGatherOutputKernel(resources, ray_buffer_index);
    return true;
}

#endif