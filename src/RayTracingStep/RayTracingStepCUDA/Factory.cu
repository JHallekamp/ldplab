#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "Factory.hpp"

#include "RayTracingStep.hpp"

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
    context.pipeline = IPipeline::create(
        IPipeline::Type::host_bound, 
        setup,
        info, 
        context);
    if (context.pipeline == nullptr)
        return false;

    return true;
}

#endif