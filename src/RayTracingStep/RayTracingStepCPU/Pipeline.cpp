#include "Pipeline.hpp"

#include "Context.hpp"
#include "../../Utils/Log.hpp"
#include "../../Utils/Assert.hpp"
#include "../../Utils/Profiler.hpp"


ldplab::rtscpu::Pipeline::Pipeline(
    std::unique_ptr<IInitialStage> initial, 
    std::unique_ptr<IRayBoundingVolumeIntersectionTestStage> rbvit,
    std::unique_ptr<IRayParticleIntersectionTestStage> rpit, 
    std::unique_ptr<IRayParticleInteractionStage> rpi,
    std::unique_ptr<IInnerParticlePropagationStage> ipp, 
    std::shared_ptr<Context> context)
    :
    m_initial_stage{ std::move(initial) },
    m_ray_bounding_volume_intersection_test_stage{ std::move(rbvit) },
    m_ray_particle_intersection_test_stage{ std::move(rpit) },
    m_ray_particle_interaction_stage{ std::move(rpi) },
    m_inner_particle_propagation_stage{ std::move(ipp) },
    m_context { context }
{
    for (size_t i = 0; i < m_context->parameters.number_parallel_pipelines; ++i)
        m_buffer_controls.emplace_back(m_context);

    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "Pipeline instance created",
        m_context->uid);
}

void ldplab::rtscpu::Pipeline::setup()
{
    LDPLAB_LOG_DEBUG("RTSCPU context %i: Setup ray tracing pipeline",
        m_context->uid);
    m_initial_stage->setup();
    m_ray_bounding_volume_intersection_test_stage->setup();
}

void ldplab::rtscpu::Pipeline::finalizeOutput(RayTracingStepOutput& output)
{
    for (size_t p = 0; p < m_context->particles.size(); ++p)
    {
        UID<Particle> puid = m_context->particle_index_to_uid_map[p];
        output.force_per_particle[puid] = Vec3{ 0, 0, 0 };
        output.torque_per_particle[puid] = Vec3{ 0, 0, 0 };
        for (size_t bc = 0; bc < m_buffer_controls.size(); ++bc)
        {
            output.force_per_particle[puid] += 
                m_buffer_controls[bc].getOutputBuffer().force[p];
            output.torque_per_particle[puid] += 
                m_buffer_controls[bc].getOutputBuffer().torque[p];
        }

        //// Transform output from particle into world space
        //const ParticleTransformation& trans = m_context->
        //    particle_transformations[p];
        //output.force_per_particle[puid] = trans.p2w_scale_rotation *
        //    output.force_per_particle[puid];
        //output.torque_per_particle[puid] = trans.p2w_scale_rotation *
        //    output.torque_per_particle[puid];
    }
}

void ldplab::rtscpu::Pipeline::execute(size_t job_id, size_t batch_size)
{
    LDPLAB_ASSERT(job_id < m_buffer_controls.size());
    LDPLAB_LOG_DEBUG("RTSCPU context %i: Ray tracing pipeline executes "\
        "pipeline instance %i",
        m_context->uid, job_id);

    BufferControl& buffer_control = m_buffer_controls[job_id];
    buffer_control.resetOutputBuffer();
    RayBuffer& initial_batch_buffer = buffer_control.initialBuffer();

    bool batches_left;
    size_t num_batches = 0;
    do
    {
        LDPLAB_PROFILING_START(pipeline_initial_batch_creation);
        batches_left = m_initial_stage->createBatch(initial_batch_buffer);
        LDPLAB_PROFILING_STOP(pipeline_initial_batch_creation);
        LDPLAB_LOG_TRACE("RTSCPU context %i: Filled batch buffer %i with %i "\
            "initial rays",
            m_context->uid, 
            initial_batch_buffer.uid, 
            initial_batch_buffer.active_rays);

        if (initial_batch_buffer.active_rays > 0)
        {
            LDPLAB_LOG_TRACE("RTSCPU context %i: Pipeline instance %i executes "\
                "batch %i on initial buffer %i",
                m_context->uid, job_id, num_batches, initial_batch_buffer.uid);
            ++num_batches;

            LDPLAB_PROFILING_START(pipeline_process_batch);
            processBatch(initial_batch_buffer, buffer_control);
            LDPLAB_PROFILING_STOP(pipeline_process_batch);

            LDPLAB_LOG_TRACE("RTSCPU context %i: Pipeline instance %i "\
                "finished batch execution",
                m_context->uid, job_id);
        }
    } while(batches_left);

    LDPLAB_LOG_DEBUG("RTSCPU context %i: Pipeline instance %i executed "\
        "successfully in a total of %i batches",
        m_context->uid, job_id, num_batches);
}

void ldplab::rtscpu::Pipeline::processBatch(
    RayBuffer& buffer, BufferControl& buffer_control)
{
    if (buffer.active_rays <= 0)
        return;

    LDPLAB_PROFILING_START(pipeline_buffer_setup);
    IntersectionBuffer& intersection_buffer =
        buffer_control.getIntersectionBuffer();
    OutputBuffer& output_buffer = buffer_control.getOutputBuffer();
    RayBuffer& reflection_buffer =
        buffer_control.getReflectionBuffer(buffer);
    RayBuffer& transmission_buffer =
        buffer_control.getTransmissionBuffer(buffer);
    // Reset intersection buffer
    for (size_t i = 0; i < intersection_buffer.size; ++i)
        intersection_buffer.particle_index[i] = -1;
    LDPLAB_PROFILING_STOP(pipeline_buffer_setup);

    if (buffer.inner_particle_rays)
    {
        LDPLAB_PROFILING_START(pipeline_inner_particle_propagation);
        m_inner_particle_propagation_stage->execute(
            buffer, intersection_buffer, output_buffer);
        LDPLAB_PROFILING_STOP(pipeline_inner_particle_propagation);

        LDPLAB_PROFILING_START(pipeline_inner_particle_interaction);
        m_ray_particle_interaction_stage->execute(
            intersection_buffer,
            buffer,
            reflection_buffer,
            transmission_buffer,
            output_buffer);
        LDPLAB_PROFILING_STOP(pipeline_inner_particle_interaction);
    }
    else
    {
        if (buffer.world_space_rays == 0)
        {
            LDPLAB_PROFILING_START(
                pipeline_world_space_particle_intersection_test);
            m_ray_particle_intersection_test_stage->execute(
                buffer,
                intersection_buffer);
            LDPLAB_PROFILING_STOP(
                pipeline_world_space_particle_intersection_test);
        }

        while (buffer.world_space_rays > 0)
        {
            LDPLAB_PROFILING_START(
                pipeline_world_space_bounding_volume_intersection_test);
            const size_t intersects_bv =
                m_ray_bounding_volume_intersection_test_stage->execute(
                    buffer);
            LDPLAB_PROFILING_STOP(
                pipeline_world_space_bounding_volume_intersection_test);

            if (intersects_bv > 0)
            {
                LDPLAB_PROFILING_START(
                    pipeline_world_space_particle_intersection_test);
                m_ray_particle_intersection_test_stage->execute(
                    buffer,
                    intersection_buffer);
                LDPLAB_PROFILING_STOP(
                    pipeline_world_space_particle_intersection_test);
            }
            else if (buffer.active_rays == 0)
                return;
        }

        LDPLAB_PROFILING_START(pipeline_world_space_particle_interaction);
        m_ray_particle_interaction_stage->execute(
            intersection_buffer,
            buffer,
            reflection_buffer,
            transmission_buffer,
            output_buffer);
        LDPLAB_PROFILING_STOP(pipeline_world_space_particle_interaction);
    }

    // Check if there are still active rays left and print a warning to the log
    if (reflection_buffer.uid != buffer_control.dummyBufferUID() &&
        transmission_buffer.uid != buffer_control.dummyBufferUID())
    {
        processBatch(reflection_buffer, buffer_control);
        processBatch(transmission_buffer, buffer_control);
    }
    else if (buffer.active_rays > 0 &&
        m_context->flags.emit_warning_on_maximum_branching_depth_discardment)
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
            m_context->uid,
            m_context->parameters.maximum_branching_depth,
            buffer.active_rays,
            max_intensity,
            avg_intensity);
    }
}
