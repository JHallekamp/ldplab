#include "Pipeline.hpp"

#include "Context.hpp"
#include "../../../Log.hpp"
#include "../../../Utils/Assert.hpp"

ldplab::rtsgpu_ogl::Pipeline::Pipeline(
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

    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "Pipeline instance created",
        m_context->uid);
}

void ldplab::rtsgpu_ogl::Pipeline::setup()
{
    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Setup ray tracing pipeline",
        m_context->uid);
    m_initial_stage->setup();
    m_ray_bounding_volume_intersection_test_stage->setup();
}

void ldplab::rtsgpu_ogl::Pipeline::finalizeOutput(RayTracingStepOutput& output)
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
    }
}

void ldplab::rtsgpu_ogl::Pipeline::execute(size_t job_id)
{
    LDPLAB_ASSERT(job_id < m_buffer_controls.size());
    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Ray tracing pipeline executes "\
        "pipeline instance %i",
        m_context->uid, job_id);

    BufferControl& buffer_control = m_buffer_controls[job_id];
    buffer_control.resetOutputBuffer();
    RayBuffer& initial_batch_buffer = buffer_control.initialBuffer();

    bool batches_left;
    size_t num_batches = 0;
    do
    {
        batches_left = m_initial_stage->createBatch(initial_batch_buffer);
        LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Filled batch buffer %i with %i "\
            "initial rays",
            m_context->uid, 
            initial_batch_buffer.uid, 
            initial_batch_buffer.active_rays);

        if (initial_batch_buffer.active_rays > 0)
        {
            ++num_batches;
            LDPLAB_LOG_TRACE("RTSCPU context %i: Pipeline instance %i executes "\
                "batch %i on initial buffer %i",
                m_context->uid, job_id, num_batches, initial_batch_buffer.uid);

            processBatch(initial_batch_buffer, buffer_control);

            LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Pipeline instance %i "\
                "finished batch execution",
                m_context->uid, job_id);
        }
    } while(batches_left);

    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Pipeline instance %i executed "\
        "successfully in a total of %i batches",
        m_context->uid, job_id, num_batches);
}

void ldplab::rtsgpu_ogl::Pipeline::processBatch(
    RayBuffer& buffer, BufferControl& buffer_control)
{
    if (buffer.active_rays <= 0)
        return;

    if (buffer.inner_particle_rays)
    {
        IntersectionBuffer& intersection_buffer =
            buffer_control.getIntersectionBuffer();
        OutputBuffer& output_buffer = buffer_control.getOutputBuffer();
        m_inner_particle_propagation_stage->execute(
            buffer, intersection_buffer, output_buffer);

        RayBuffer& reflection_buffer =
            buffer_control.getReflectionBuffer(buffer);
        RayBuffer& transmission_buffer =
            buffer_control.getTransmissionBuffer(buffer);

        m_ray_particle_interaction_stage->execute(
            intersection_buffer,
            buffer,
            reflection_buffer,
            transmission_buffer,
            output_buffer);

        if (reflection_buffer.uid != buffer_control.dummyBufferUID() &&
            transmission_buffer.uid != buffer_control.dummyBufferUID())
        {
            processBatch(reflection_buffer, buffer_control);
            processBatch(transmission_buffer, buffer_control);
        }
    }
    else
    {
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

        if (buffer.world_space_rays == 0)
        {
            m_ray_particle_intersection_test_stage->execute(
                buffer,
                intersection_buffer);
        }

        if (buffer.world_space_rays > 0)
        {
            do
            {
                if (m_ray_bounding_volume_intersection_test_stage->execute(
                    buffer))
                {
                    m_ray_particle_intersection_test_stage->execute(
                        buffer,
                        intersection_buffer);
                }
                else if (buffer.active_rays == 0)
                    return;
            } while (buffer.world_space_rays > 0);
        }

        m_ray_particle_interaction_stage->execute(
            intersection_buffer,
            buffer,
            reflection_buffer,
            transmission_buffer,
            output_buffer);

        if (reflection_buffer.uid != buffer_control.dummyBufferUID() &&
            transmission_buffer.uid != buffer_control.dummyBufferUID())
        {
            processBatch(reflection_buffer, buffer_control);
            processBatch(transmission_buffer, buffer_control);
        }
    }
}
