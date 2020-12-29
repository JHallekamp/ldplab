#include "Pipeline.hpp"
#include "../../Log.hpp"
#include "../../Utils/Assert.hpp"

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
    m_context { context },
    m_buffer_controls{ context->number_parallel_pipelines, context }
{
}

void ldplab::rtscpu::Pipeline::setup()
{
    LDPLAB_LOG_DEBUG("RTSCPU context %i: Setup ray tracing pipeline",
        m_context->uid);
    m_initial_stage->setup();
    m_ray_bounding_volume_intersection_test_stage->setup();
}

void ldplab::rtscpu::Pipeline::execute(size_t job_id)
{
    LDPLAB_ASSERT(job_id < m_buffer_controls.size());
    LDPLAB_LOG_DEBUG("RTSCPU context %i: Ray tracing pipeline executes "\
        "pipeline instance %i",
        m_context->uid, job_id);

    BufferControl& buffer_control = m_buffer_controls[job_id];
    RayBuffer& initial_batch_buffer = buffer_control.initialBuffer();
    
    // Setup has to be performed by the ray tracing step instance itself, since
    // it only has to be executed once!
    //m_initial_stage->setup();
    //m_ray_bounding_volume_intersection_test_stage->setup();
    
    bool batches_left;
    do
    {
        batches_left = m_initial_stage->createBatch(initial_batch_buffer);
        LDPLAB_LOG_TRACE("RTSCPU context %i: Filled batch buffer %i with %i"\
            "initial rays",
            m_context->uid, 
            initial_batch_buffer.uid, 
            initial_batch_buffer.active_rays);

        if (initial_batch_buffer.active_rays > 0)
        {
            LDPLAB_LOG_TRACE("RTSCPU context %i: Pipeline instance %i executes "\
                "new batch on initial buffer %i",
                m_context->uid, job_id, initial_batch_buffer.uid);

            processBatch(initial_batch_buffer, buffer_control);

            // The following part basically was a workaround so the bounding
            // volume intersection test would not be executed on freshly
            // created initial batch buffers. However, in the sake of better 
            // readability, we do not do that for now.
            /*
            IntersectionBuffer& intersection_buffer =
                buffer_control.getIntersectionBuffer();
            RayBuffer& reflection_buffer =
                buffer_control.getReflectionBuffer(initial_batch_buffer);
            RayBuffer& transmission_buffer =
                buffer_control.getTransmissionBuffer(initial_batch_buffer);

            m_ray_particle_intersection_test_stage->execute(
                initial_batch_buffer,
                intersection_buffer);

            m_ray_particle_interaction_stage->execute(
                intersection_buffer,
                initial_batch_buffer,
                reflection_buffer,
                transmission_buffer);

            while (initial_batch_buffer.active_rays > 0)
            {
                m_ray_bounding_volume_intersection_test_stage->execute(
                    initial_batch_buffer);

                m_ray_particle_intersection_test_stage->execute(
                    initial_batch_buffer,
                    intersection_buffer);

                m_ray_particle_interaction_stage->execute(
                    intersection_buffer,
                    initial_batch_buffer,
                    reflection_buffer,
                    transmission_buffer);
            }

            if (reflection_buffer.index != buffer_control.dummyBufferIndex() &&
                transmission_buffer.index != buffer_control.dummyBufferIndex())
            {
                processBatch(reflection_buffer, buffer_control);
                processBatch(transmission_buffer, buffer_control);
            }*/

            LDPLAB_LOG_TRACE("RTSCPU context %i: Pipeline instance %i "\
                "finished batch execution",
                m_context->uid, job_id);
        }
    } while(batches_left);

    LDPLAB_LOG_DEBUG("RTSCPU context %i: Pipeline instance %i executed "\
        "successfully",
        m_context->uid, job_id);
}

void ldplab::rtscpu::Pipeline::processBatch(
    RayBuffer& buffer, BufferControl& buffer_control)
{
    if (buffer.inner_particle_rays)
    {
        IntersectionBuffer& intersection_buffer =
            buffer_control.getIntersectionBuffer();
        m_inner_particle_propagation_stage->execute(
            buffer, intersection_buffer);

        RayBuffer& reflection_buffer =
            buffer_control.getReflectionBuffer(buffer);
        RayBuffer& transmission_buffer =
            buffer_control.getTransmissionBuffer(buffer);

        if (reflection_buffer.index != buffer_control.dummyBufferIndex() &&
            transmission_buffer.index != buffer_control.dummyBufferIndex())
        {
            m_ray_particle_interaction_stage->execute(
                intersection_buffer,
                buffer,
                reflection_buffer,
                transmission_buffer);

            processBatch(reflection_buffer, buffer_control);
            processBatch(transmission_buffer, buffer_control);
        }
    }
    else
    {
        IntersectionBuffer& intersection_buffer =
            buffer_control.getIntersectionBuffer();
        RayBuffer& reflection_buffer =
            buffer_control.getReflectionBuffer(buffer);
        RayBuffer& transmission_buffer =
            buffer_control.getTransmissionBuffer(buffer);

        if (reflection_buffer.index != buffer_control.dummyBufferIndex() &&
            transmission_buffer.index != buffer_control.dummyBufferIndex())
        {
            while (buffer.active_rays > 0)
            {
                m_ray_bounding_volume_intersection_test_stage->execute(
                    buffer);
                
                m_ray_particle_intersection_test_stage->execute(
                    buffer, 
                    intersection_buffer);
                
                m_ray_particle_interaction_stage->execute(
                    intersection_buffer, 
                    buffer, 
                    reflection_buffer, 
                    transmission_buffer);
            }

            processBatch(reflection_buffer, buffer_control);
            processBatch(transmission_buffer, buffer_control);
        }
    }
}
