#include "Pipeline.hpp"

#include "../../Utils/Assert.hpp"
#include "../../Utils/Log.hpp"
#include "../../Utils/Profiler.hpp"
#include "RayTracingStepCPU.hpp"
#include "MemoryControl.hpp"

#include <glm/ext.hpp>

ldplab::rtscpu::Pipeline::Pipeline(
    const RayTracingStepCPU& owner, 
    const RayTracingStepCPUInfo& info, 
    const SimulationParameter& simulation_parameter,
    InterfaceMapping&& interface_mapping, 
    std::unique_ptr<ExperimentalSetup>&& setup, 
    std::vector<MemoryControl>&& memory_controls, 
    std::vector<std::shared_ptr<IGenericGeometry>>&& geometries,
    std::unique_ptr<IBoundingVolumeIntersection>&& bvi, 
    std::unique_ptr<IInitialStage>&& is, 
    std::unique_ptr<IInnerParticlePropagation>&& ipp, 
    std::unique_ptr<IParticleIntersection>&& pi, 
    std::unique_ptr<ISurfaceInteraction>&& si)
    :
    m_owner{ owner },
    m_info{ info },
    m_sim_params{ simulation_parameter },
    m_interface_mapping{ std::move(interface_mapping) },
    m_setup{ std::move(setup) },
    m_memory_controls{ std::move(memory_controls) },
    m_generic_geometries{ std::move(geometries) },
    m_particle_transformations{ simulation_parameter.num_particles },
    m_stage_bvi{ std::move(bvi) },
    m_stage_is{ std::move(is) },
    m_stage_ipp{ std::move(ipp) },
    m_stage_pi{ std::move(pi) },
    m_stage_si{ std::move(si) }
{ }

void ldplab::rtscpu::Pipeline::stepSetup(const SimulationState& sim_state)
{
    // Update particle transformations
    for (size_t i = 0; i < m_sim_params.num_particles; ++i)
    {
        UID<Particle> particle_uid{ m_interface_mapping.particle_index_to_uid[i] };
        auto particle_it = sim_state.particle_instances.find(particle_uid);
        if (particle_it == sim_state.particle_instances.end())
        {
            LDPLAB_LOG_ERROR("RTSCPU context %i: Could not update particle "\
                "transformations, particle %i is not present in the given "\
                "simulation state, abort RTSCPU execution",
                m_owner.uid(),
                particle_uid);
        }
        const ParticleInstance& particle_instance = particle_it->second;

        // Set particle current transformation
        m_particle_transformations[i].w2p_translation =
            -particle_instance.position;
        m_particle_transformations[i].p2w_translation =
            particle_instance.position;
        m_particle_transformations[i].w2p_rotation_scale =
            getRotationMatrix(
                -particle_instance.orientation.x,
                -particle_instance.orientation.y,
                -particle_instance.orientation.z,
                invertRotationOrder(particle_instance.rotation_order));
        m_particle_transformations[i].p2w_scale_rotation =
            getRotationMatrix(
                particle_instance.orientation.x,
                particle_instance.orientation.y,
                particle_instance.orientation.z,
                particle_instance.rotation_order);
    }

    // Call stage setups
    m_stage_bvi->stepSetup(*m_setup, sim_state, m_interface_mapping);
    m_stage_is->stepSetup(*m_setup, sim_state, m_interface_mapping);
    m_stage_ipp->stepSetup(*m_setup, sim_state, m_interface_mapping);
    m_stage_pi->stepSetup(*m_setup, sim_state, m_interface_mapping);
    m_stage_si->stepSetup(*m_setup, sim_state, m_interface_mapping);
}

void ldplab::rtscpu::Pipeline::execute(size_t job_id, size_t batch_size)
{
    LDPLAB_ASSERT(job_id < m_memory_controls.size());
    LDPLAB_LOG_DEBUG("RTSCPU context %i: Ray tracing pipeline executes "\
        "pipeline instance %i",
        m_owner.uid(), job_id);

    MemoryControl& memory_control = m_memory_controls[job_id];
    memory_control.resetOutputBuffer();
    RayBuffer& initial_batch_buffer = memory_control.initialBuffer();
    MemoryControl::StageDependentData& sdd = 
        memory_control.getStageDependentData();

    bool batches_left;
    size_t num_batches = 0;
    do
    {
        // Setup batch
        LDPLAB_PROFILING_START(pipeline_stage_batch_setup);
        m_stage_bvi->batchSetup(sdd.bounding_volume_intersection_data.get());
        m_stage_ipp->batchSetup(sdd.inner_particle_propagation_data.get());
        m_stage_is->batchSetup(sdd.initial_stage_data.get());
        m_stage_pi->batchSetup(sdd.particle_intersection_data.get());
        m_stage_si->batchSetup(sdd.surface_interaction_data.get());
        LDPLAB_PROFILING_STOP(pipeline_stage_batch_setup);

        LDPLAB_PROFILING_START(pipeline_initial_batch_creation);
        batches_left = m_stage_is->execute(
            initial_batch_buffer,
            m_sim_params,
            sdd.initial_stage_data.get());
        LDPLAB_PROFILING_STOP(pipeline_initial_batch_creation);
        LDPLAB_LOG_TRACE("RTSCPU context %i: Filled batch buffer %i with %i "\
            "initial rays",
            m_owner.uid(),
            initial_batch_buffer.uid,
            initial_batch_buffer.active_rays);

        if (initial_batch_buffer.active_rays > 0)
        {
            LDPLAB_LOG_TRACE("RTSCPU context %i: Pipeline instance %i executes "\
                "batch %i on initial buffer %i",
                m_owner.uid(), job_id, num_batches, initial_batch_buffer.uid);
            ++num_batches;

            LDPLAB_PROFILING_START(pipeline_process_batch);
            processBatch(initial_batch_buffer, memory_control);
            LDPLAB_PROFILING_STOP(pipeline_process_batch);

            LDPLAB_LOG_TRACE("RTSCPU context %i: Pipeline instance %i "\
                "finished batch execution",
                m_owner.uid(), job_id);
        }
    } while (batches_left);

    LDPLAB_LOG_DEBUG("RTSCPU context %i: Ray tracing pipeline finished  "\
        "execution of pipeline instance %i",
        m_owner.uid(), job_id);
}

ldplab::Mat3 ldplab::rtscpu::Pipeline::getRotationMatrix(
    double rx,
    double ry,
    double rz,
    RotationOrder order)
{
    ldplab::Mat3 rotx(0), roty(0), rotz(0);

    rotx[0][0] = 1;
    rotx[1][1] = cos(rx);
    rotx[1][2] = -sin(rx);
    rotx[2][1] = sin(rx);
    rotx[2][2] = cos(rx);

    roty[0][0] = cos(ry);
    roty[0][2] = sin(ry);
    roty[1][1] = 1;
    roty[2][0] = -sin(ry);
    roty[2][2] = cos(ry);

    rotz[0][0] = cos(rz);
    rotz[0][1] = -sin(rz);
    rotz[1][0] = sin(rz);
    rotz[1][1] = cos(rz);
    rotz[2][2] = 1;

    switch (order)
    {
    case (RotationOrder::xyz): return rotz * roty * rotx;
    case (RotationOrder::xzy): return roty * rotz * rotx;
    case (RotationOrder::yxz): return rotz * rotx * roty;
    case (RotationOrder::yzx): return rotx * rotz * roty;
    case (RotationOrder::zxy): return roty * rotx * rotz;
    case (RotationOrder::zyx): return rotx * roty * rotz;
    }

    // To avoid compiler warnings
    LDPLAB_LOG_WARNING("RTSCPU context %i: Encountered unknown rotation "\
        "order, assumes xyz instead.",
        m_owner.uid());
    return rotz * roty * rotx;
}

void ldplab::rtscpu::Pipeline::processBatch(RayBuffer& buffer, MemoryControl& mem_control)
{
    if (buffer.active_rays <= 0)
        return;

    LDPLAB_PROFILING_START(pipeline_buffer_setup);
    IntersectionBuffer& intersection_buffer = 
        mem_control.getIntersectionBuffer(buffer.depth);
    OutputBuffer& output_buffer = mem_control.getOutputBuffer();
    RayBuffer& output_ray_buffer = mem_control.getRayBuffer(buffer.depth + 1);
    MemoryControl::StageDependentData& sdd = 
        mem_control.getStageDependentData();
    // Reset intersection buffer
    for (size_t i = 0; i < intersection_buffer.size; ++i)
        intersection_buffer.particle_index[i] = -1;
    LDPLAB_PROFILING_STOP(pipeline_buffer_setup);

    if (buffer.inner_particle_rays)
    {
        LDPLAB_PROFILING_START(pipeline_inner_particle_propagation);
        m_stage_ipp->execute(
            buffer,
            intersection_buffer,
            output_buffer,
            m_generic_geometries,
            m_sim_params,
            sdd.inner_particle_propagation_data.get());
        LDPLAB_PROFILING_STOP(pipeline_inner_particle_propagation);
    }
    else
    {
        if (buffer.active_rays != buffer.world_space_rays)
        {
            LDPLAB_PROFILING_START(
                pipeline_world_space_particle_intersection_test);
            m_stage_pi->execute(
                buffer,
                intersection_buffer,
                m_particle_transformations,
                m_generic_geometries,
                m_sim_params,
                sdd.particle_intersection_data.get());
            LDPLAB_PROFILING_STOP(
                pipeline_world_space_particle_intersection_test);
        }

        while (buffer.world_space_rays > 0)
        {
            LDPLAB_PROFILING_START(
                pipeline_world_space_bounding_volume_intersection_test);
            const size_t intersects_bv =
                m_stage_bvi->execute(
                    buffer,
                    m_particle_transformations,
                    m_sim_params,
                    sdd.bounding_volume_intersection_data.get());
            LDPLAB_PROFILING_STOP(
                pipeline_world_space_bounding_volume_intersection_test);

            if (intersects_bv > 0)
            {
                LDPLAB_PROFILING_START(
                    pipeline_world_space_particle_intersection_test);
                m_stage_pi->execute(
                    buffer,
                    intersection_buffer,
                    m_particle_transformations,
                    m_generic_geometries,
                    m_sim_params,
                    sdd.particle_intersection_data.get());
                LDPLAB_PROFILING_STOP(
                    pipeline_world_space_particle_intersection_test);
            }
            else if (buffer.active_rays == 0)
                return;
        }
    }

    if (buffer.depth < m_sim_params.max_branching_depth)
    {
        for (size_t r = 0; 
            r < m_sim_params.num_surface_interaction_reflection_passes; 
            ++r)
        {
            LDPLAB_PROFILING_START(pipeline_surface_interaction);
            m_stage_si->execute(
                buffer,
                intersection_buffer,
                output_ray_buffer,
                output_buffer,
                m_info.intensity_cutoff,
                m_setup->medium_reflection_index,
                ISurfaceInteraction::InteractionPassType::reflection,
                r,
                m_sim_params,
                sdd.surface_interaction_data.get());
            LDPLAB_PROFILING_STOP(pipeline_surface_interaction);
            processBatch(output_ray_buffer, mem_control);
        }

        for (size_t t = 0;
            t < m_sim_params.num_surface_interaction_reflection_passes;
            ++t)
        {
            LDPLAB_PROFILING_START(pipeline_surface_interaction);
            m_stage_si->execute(
                buffer,
                intersection_buffer,
                output_ray_buffer,
                output_buffer,
                m_info.intensity_cutoff,
                m_setup->medium_reflection_index,
                ISurfaceInteraction::InteractionPassType::transmission,
                t,
                m_sim_params,
                sdd.surface_interaction_data.get());
            LDPLAB_PROFILING_STOP(pipeline_surface_interaction);
            processBatch(output_ray_buffer, mem_control);
        }
    }
    else if (buffer.active_rays > 0 &&
        m_info.emit_warning_on_maximum_branching_depth_discardment)
    {
        // Compute max and average length
        double max_intensity = 0.0, avg_intensity = 0.0;
        for (size_t i = 0; i < buffer.size; ++i)
        {
            if (buffer.index_data[i] < 0)
                continue;

            double intensity = buffer.ray_data[i].intensity;
            avg_intensity += intensity;
            if (intensity > max_intensity)
                max_intensity = intensity;
        }
        avg_intensity /= static_cast<double>(buffer.active_rays);
        LDPLAB_LOG_WARNING("RTSCPU context %i: Pipeline reached max branching "\
            "depth %i with a total of %i still active rays, which include a "\
            "max intensity of %f and average intensity of %f",
            m_owner.uid(),
            m_sim_params.max_branching_depth,
            buffer.active_rays,
            max_intensity,
            avg_intensity);
    }
}
