#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "Factory.hpp"

#include "RayTracingStep.hpp"
#include "../../../Utils/Log.hpp"

std::shared_ptr<ldplab::IRayTracingStep> ldplab::rtscuda::Factory::createRTS(
    const ExperimentalSetup& setup, 
    const RayTracingStepCUDAInfo& info)
{
    std::shared_ptr<RayTracingStep> rts = std::make_shared<RayTracingStep>();
    rts->m_context = std::make_unique<Context>();
    createContext(setup, info, *rts->m_context);
    return rts;
}

bool ldplab::rtscuda::Factory::createContext(
    const ExperimentalSetup& setup, 
    const RayTracingStepCUDAInfo& info, 
    Context& context)
{
    // Write parameters
    context.parameters.intensity_cutoff = info.intensity_cutoff;
    context.parameters.light_source_resolution_per_world_unit = 
        info.light_source_resolution_per_world_unit;
    context.parameters.max_branching_depth = info.maximum_branching_depth;
    context.parameters.medium_reflection_index = setup.medium_reflection_index;
    context.parameters.num_light_sources = setup.light_sources.size();
    context.parameters.num_particles = setup.particles.size();
    context.parameters.num_rays_per_batch = info.number_rays_per_batch;
    context.parameters.num_threads_per_block = info.number_threads_per_block;
    context.parameters.output_in_particle_space = 
        info.return_force_in_particle_coordinate_system;
    if (!readCudaInfo(context))
        return false;

    // Create data
    if (!context.resources.bounding_volumes.allocateResource(setup.particles))
        return false;
    if (!context.resources.intersection_buffer.allocateResources(
        context.parameters.num_rays_per_batch))
        return false;
    if (!context.resources.output_buffer.allocateResources(
        context.parameters.num_particles,
        context.parameters.num_rays_per_batch))
        return false;
    if (!context.resources.particles.allocateResource(setup.particles))
        return false;
    if (!context.resources.pipeline.allocateResources(
        context.parameters.num_rays_per_batch,
        context.parameters.num_threads_per_block))
        return false;
    if (!context.resources.ray_buffer.allocateResources(
        context.parameters.max_branching_depth * 2 + 3,
        context.parameters.num_rays_per_batch))
        return false;
    if (!context.resources.transformations.allocateResource(
        context.parameters.num_particles))
        return false;

    // Create interface
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        context.interface_mapping.particle_index_to_uid.emplace(
            i, setup.particles[i].uid);
        context.interface_mapping.particle_uid_to_index.emplace(
            setup.particles[i].uid, i);
    }
    for (size_t i = 0; i < setup.light_sources.size(); ++i)
    {
        context.interface_mapping.light_source_index_to_uid.emplace(
            i, setup.light_sources[i].uid);
        context.interface_mapping.light_source_uid_to_index.emplace(
            setup.light_sources[i].uid, i);
    }

    // Create the pipeline
    IPipeline::Type pipeline_type = IPipeline::Type::device_bound;
    if (info.host_bound_pipeline)
        pipeline_type = IPipeline::Type::host_bound;
    context.pipeline = IPipeline::create(
        pipeline_type,
        setup,
        info, 
        context);
    if (context.pipeline == nullptr)
        return false;

    return true;
}

bool ldplab::rtscuda::Factory::readCudaInfo(Context& context)
{
    int device_count = 0;
    cudaError_t error_id = cudaGetDeviceCount(&device_count);

    if (error_id != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to get cuda device count, "\
            "cudaGetDeviceCount returned error code %i: %s",
            error_id,
            cudaGetErrorString(error_id));
        return false;
    }

    if (device_count == 0)
    {
        LDPLAB_LOG_ERROR("RTSCUDA factory: Failed to find any cuda devices.");
        return false;
    }

    cudaSetDevice(0);
    cudaDeviceProp device_prop;
    cudaGetDeviceProperties(&device_prop, 0);
    LDPLAB_LOG_INFO("RTSCUDA factory: Using cuda device %s", device_prop.name);
    
    auto &props = context.device_properties;
    props.max_block_size.x = device_prop.maxThreadsDim[0];
    props.max_block_size.y = device_prop.maxThreadsDim[1];
    props.max_block_size.z = device_prop.maxThreadsDim[2];
    props.max_grid_size.x = device_prop.maxGridSize[0];
    props.max_grid_size.y = device_prop.maxGridSize[1];
    props.max_grid_size.z = device_prop.maxGridSize[2];
    props.max_num_threads_per_block = device_prop.maxThreadsPerBlock;
    props.max_num_threads_per_mp = device_prop.maxThreadsPerMultiProcessor;
    props.num_mps = device_prop.multiProcessorCount;
    props.registers_per_block = device_prop.regsPerBlock;
    props.shared_mem_per_block = device_prop.sharedMemPerBlock;
    props.shared_mem_per_mp = device_prop.sharedMemPerMultiprocessor;
    props.warp_size = device_prop.warpSize;
    
    return true;
}

#endif