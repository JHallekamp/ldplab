#include "Pipeline.hpp"

#include <array>
#include <LDPLAB/RayTracingStep/CPU/ISurfaceInteraction.hpp>

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
    ExperimentalSetup&& setup, 
    std::vector<MemoryControl>&& memory_controls, 
    std::vector<std::shared_ptr<IGenericGeometry>>&& geometries,
    std::vector<std::shared_ptr<IParticleMaterial>>&& materials,
    std::vector<Vec3>&& particle_center_of_mass,
    std::shared_ptr<IBoundingVolumeIntersection>&& bvi,
    std::shared_ptr<IInitialStage>&& is,
    std::shared_ptr<IInnerParticlePropagation>&& ipp,
    std::shared_ptr<IParticleIntersection>&& pi,
    std::shared_ptr<ISurfaceInteraction>&& si)
    :
    m_owner{ owner },
    m_info{ info },
    m_sim_params{ simulation_parameter },
    m_interface_mapping{ std::move(interface_mapping) },
    m_setup{ std::move(setup) },
    m_memory_controls{ std::move(memory_controls) },
    m_generic_geometries{ std::move(geometries) },
    m_particle_materials{ std::move(materials) },
    m_particle_center_of_mass{ std::move(particle_center_of_mass) },
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
            LDPLAB_LOG_ERROR("RTSCPU %i: Could not update particle "\
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
                -particle_instance.orientation.z,
                -particle_instance.orientation.y,
                -particle_instance.orientation.x,
                invertRotationOrder(particle_instance.rotation_order));
        m_particle_transformations[i].p2w_scale_rotation =
            getRotationMatrix(
                particle_instance.orientation.x,
                particle_instance.orientation.y,
                particle_instance.orientation.z,
                particle_instance.rotation_order);
    }

    // Call stage setups
    m_stage_bvi->stepSetup(
        m_setup, sim_state, m_interface_mapping, m_particle_transformations);
    m_stage_is->stepSetup(
        m_setup, sim_state, m_interface_mapping, m_particle_transformations);
    m_stage_ipp->stepSetup(
        m_setup, sim_state, m_interface_mapping, m_particle_transformations);
    m_stage_pi->stepSetup(
        m_setup, sim_state, m_interface_mapping, m_particle_transformations);
    m_stage_si->stepSetup(
        m_setup, sim_state, m_interface_mapping, m_particle_transformations);
}

void ldplab::rtscpu::Pipeline::finalizeOutput(RayTracingStepOutput& output)
{
    for (size_t p = 0; p < m_setup.particles.size(); ++p)
    {
        UID<Particle> puid = m_interface_mapping.particle_index_to_uid[p];
        output.force_per_particle[puid] = Vec3{ 0, 0, 0 };
        output.torque_per_particle[puid] = Vec3{ 0, 0, 0 };
        for (size_t bc = 0; bc < m_memory_controls.size(); ++bc)
        {
            output.force_per_particle[puid] +=
                m_memory_controls[bc].getOutputBuffer().force[p];
            output.torque_per_particle[puid] +=
                m_memory_controls[bc].getOutputBuffer().torque[p];
        }
        
        if (!m_info.return_force_in_particle_coordinate_system)
        {
            // Transform output from particle into world space
            const ParticleTransformation& trans = m_particle_transformations[p];
            output.force_per_particle[puid] = trans.p2w_scale_rotation *
                output.force_per_particle[puid];
            output.torque_per_particle[puid] = trans.p2w_scale_rotation *
                output.torque_per_particle[puid];
        }
    }
}

void ldplab::rtscpu::Pipeline::execute(
    size_t job_id, 
    size_t batch_size,
    size_t thread_id)
{
    LDPLAB_ASSERT(job_id < m_memory_controls.size());
    LDPLAB_LOG_DEBUG("RTSCPU %i: Ray tracing pipeline executes "\
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
        LDPLAB_LOG_TRACE("RTSCPU %i: Filled batch buffer %i with %i "\
            "initial rays",
            m_owner.uid(),
            initial_batch_buffer.uid,
            initial_batch_buffer.active_rays);

        if (initial_batch_buffer.active_rays > 0)
        {
            LDPLAB_LOG_TRACE("RTSCPU %i: Pipeline instance %i executes "\
                "batch %i on initial buffer %i",
                m_owner.uid(), job_id, num_batches, initial_batch_buffer.uid);
            ++num_batches;

            LDPLAB_PROFILING_START(pipeline_process_batch);
            processBatch(initial_batch_buffer, memory_control);
            LDPLAB_PROFILING_STOP(pipeline_process_batch);

            LDPLAB_LOG_TRACE("RTSCPU %i: Pipeline instance %i "\
                "finished batch execution",
                m_owner.uid(), job_id);
        }
    } while (batches_left);

    LDPLAB_LOG_DEBUG("RTSCPU %i: Ray tracing pipeline finished  "\
        "execution of pipeline instance %i",
        m_owner.uid(), job_id);
}

namespace
{
    ldplab::Mat3 rotationX(const double angle)
    {
        ldplab::Mat3 rot(0);

        rot[0][0] = 1;
        rot[1][1] = cos(angle);
        rot[1][2] = sin(angle);
        rot[2][1] = -sin(angle);
        rot[2][2] = cos(angle);

        return rot;
    }
    ldplab::Mat3 rotationY(const double angle)
    {
        ldplab::Mat3 rot(0);

        rot[0][0] = cos(angle);
        rot[0][2] = -sin(angle);
        rot[1][1] = 1;
        rot[2][0] = sin(angle);
        rot[2][2] = cos(angle);

        return rot;
    }
    ldplab::Mat3 rotationZ(const double angle)
    {
        ldplab::Mat3 rot(0);

        rot[0][0] = cos(angle);
        rot[0][1] = sin(angle);
        rot[1][0] = -sin(angle);
        rot[1][1] = cos(angle);
        rot[2][2] = 1;

        return rot;
    }
}

ldplab::Mat3 ldplab::rtscpu::Pipeline::getRotationMatrix(
    double a,
    double b,
    double c,
    RotationOrder order)
{
    /*
        ldplab::Mat3 rotx(0), roty(0), rotz(0);
    
        rotx[0][0] = 1;
        rotx[1][1] = cos(rx);
        rotx[1][2] = sin(rx);
        rotx[2][1] = -sin(rx);
        rotx[2][2] = cos(rx);

        roty[0][0] = cos(ry);
        roty[0][2] = -sin(ry);
        roty[1][1] = 1;
        roty[2][0] = sin(ry);
        roty[2][2] = cos(ry);

        rotz[0][0] = cos(rz);
        rotz[0][1] = sin(rz);
        rotz[1][0] = -sin(rz);
        rotz[1][1] = cos(rz);
        rotz[2][2] = 1;
    */

    switch (order)
    {
    case (RotationOrder::xyz): return rotationZ(c) * rotationY(b) * rotationX(a);
    case (RotationOrder::xzy): return rotationY(c) * rotationZ(b) * rotationX(a);
    case (RotationOrder::yxz): return rotationZ(c) * rotationX(b) * rotationY(a);
    case (RotationOrder::yzx): return rotationX(c) * rotationZ(b) * rotationY(a);
    case (RotationOrder::zxy): return rotationY(c) * rotationX(b) * rotationZ(a);
    case (RotationOrder::zyx): return rotationX(c) * rotationY(b) * rotationZ(a);
    case (RotationOrder::zyz): return rotationZ(c) * rotationY(b) * rotationZ(a);
    case (RotationOrder::zxz): return rotationZ(c) * rotationX(b) * rotationZ(a);
    }

    // To avoid compiler warnings
    LDPLAB_LOG_WARNING("RTSCPU %i: Encountered unknown rotation "\
        "order, assumes xyz instead.",
        m_owner.uid());
    return rotationZ(c) * rotationY(b) * rotationX(a);
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
            m_particle_materials,
            m_particle_center_of_mass,
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

    struct SurfaceInteractionPass {
        ISurfaceInteraction::InteractionPassType type;
        size_t iterations;
    };
    std::array<SurfaceInteractionPass, 2> si_passes = {
        SurfaceInteractionPass{ 
            ISurfaceInteraction::InteractionPassType::reflection, 
            m_sim_params.num_surface_interaction_reflection_passes },
        SurfaceInteractionPass{
            ISurfaceInteraction::InteractionPassType::transmission,
            m_sim_params.num_surface_interaction_transmission_passes }
    };
    for (size_t pass = 0; pass < si_passes.size(); ++pass)
    {
        for (size_t i = 0; i < si_passes[pass].iterations; ++i)
        {
            LDPLAB_PROFILING_START(pipeline_surface_interaction);
            m_stage_si->execute(
                buffer,
                intersection_buffer,
                output_ray_buffer,
                output_buffer,
                m_info.intensity_cutoff,
                m_setup.medium_refraction_index,
                m_particle_materials,
                m_particle_center_of_mass,
                si_passes[pass].type,
                i,
                m_sim_params,
                sdd.surface_interaction_data.get());
            LDPLAB_PROFILING_STOP(pipeline_surface_interaction);
            if (buffer.depth < m_sim_params.max_branching_depth)
                processBatch(output_ray_buffer, mem_control);
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
                LDPLAB_LOG_WARNING("RTSCPU %i: Pipeline reached max branching "\
                    "depth %i with a total of %i still active rays, which include a "\
                    "max intensity of %f and average intensity of %f",
                    m_owner.uid(),
                    m_sim_params.max_branching_depth,
                    buffer.active_rays,
                    max_intensity,
                    avg_intensity);
            }
        }
    }
}
