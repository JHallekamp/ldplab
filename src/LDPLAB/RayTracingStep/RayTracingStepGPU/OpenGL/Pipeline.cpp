#include "Pipeline.hpp"

#include "Constants.hpp"
#include "Context.hpp"
#include "../../../Log.hpp"
#include "../../../Utils/Assert.hpp"
#include "../../../Utils/Profiler.hpp"

#include <sstream>

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

bool ldplab::rtsgpu_ogl::Pipeline::initShaders(
    const RayTracingStepGPUOpenGLInfo& info)
{
    std::string shader_path = info.shader_base_directory_path +
        "glsl/ldplab_cs_reset_output_and_intersection_buffer.glsl";

    // Load file
    std::ifstream file(shader_path);
    if (!file)
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Unable to open shader "\
            "source file \"%s\"", m_context->uid, shader_path.c_str());
        return false;
    }
    std::stringstream code;
    code << file.rdbuf();

    // Create shader
    std::mutex& gpu_mutex = m_context->ogl->getGPUMutex();
    std::unique_lock<std::mutex> gpu_lock{ gpu_mutex };
    m_context->ogl->bindGlContext();

    m_reset_buffer_cs =
        m_context->ogl->createComputeShader(
            "ResetOutputAndIntersectionBuffer", code.str());
    file.close();

    if (m_reset_buffer_cs != nullptr)
    {
        // Get uniform location
        m_reset_buffer_shader_uniform_location_num_rays_per_buffer =
            m_reset_buffer_cs->getUniformLocation("num_rays_per_buffer");

        // Set uniforms
        m_reset_buffer_cs->use();
        glUniform1ui(m_reset_buffer_shader_uniform_location_num_rays_per_buffer,
            m_context->parameters.number_rays_per_buffer);

        // Compute number of dispatched work groups
        const size_t num_rays_per_buffer =
            m_context->parameters.number_rays_per_buffer;
        m_shader_num_work_groups =
            (num_rays_per_buffer / constant::glsl_local_group_sizes::
                reset_output_and_intersection_buffer) +
            (num_rays_per_buffer % constant::glsl_local_group_sizes::
                reset_output_and_intersection_buffer ? 1 : 0);
    }

    // Finish
    m_context->ogl->unbindGlContext();
    return (m_reset_buffer_cs != nullptr);
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
        Vec3 force = Vec3{ 0, 0, 0 };
        Vec3 torque = Vec3{ 0, 0, 0 };
        for (size_t bc = 0; bc < m_buffer_controls.size(); ++bc)
        {
            force += m_buffer_controls[bc].getOutputBuffer().force_data[p];
            torque += m_buffer_controls[bc].getOutputBuffer().torque_data[p];
        }

        // Transform output from particle into world space
        const Mat3& p2w = 
            m_context->particle_transformation_data.p2w_scale_rotation[p];
        output.force_per_particle[puid] = p2w * force;
        output.torque_per_particle[puid] = p2w * torque;
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
        LDPLAB_PROFILING_START(pipeline_initial_batch_creation);
        batches_left = m_initial_stage->createBatch(initial_batch_buffer);
        LDPLAB_PROFILING_STOP(pipeline_initial_batch_creation);
        LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Filled batch buffer %i "\
            "with %i initial rays",
            m_context->uid, 
            initial_batch_buffer.uid, 
            initial_batch_buffer.active_rays);

        if (initial_batch_buffer.active_rays > 0)
        {
            LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Pipeline instance "\
                "%i executes batch %i on initial buffer %i",
                m_context->uid, job_id, num_batches, initial_batch_buffer.uid);
            ++num_batches;

            LDPLAB_PROFILING_START(pipeline_process_batch);
            uploadInitialBatch(initial_batch_buffer);
            processBatch(initial_batch_buffer, buffer_control);
            LDPLAB_PROFILING_STOP(pipeline_process_batch);

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

    LDPLAB_PROFILING_START(pipeline_buffer_setup);
    IntersectionBuffer& intersection_buffer =
        buffer_control.getIntersectionBuffer();
    OutputBuffer& output_buffer = buffer_control.getOutputBuffer();
    RayBuffer& reflection_buffer =
        buffer_control.getReflectionBuffer(buffer);
    RayBuffer& transmission_buffer =
        buffer_control.getTransmissionBuffer(buffer);
    resetBuffers(output_buffer, intersection_buffer);
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
    if (reflection_buffer.uid != buffer_control.dummyBufferDepth() &&
        transmission_buffer.uid != buffer_control.dummyBufferDepth())
    {
        processBatch(reflection_buffer, buffer_control);
        processBatch(transmission_buffer, buffer_control);
    }
    else if (buffer.active_rays > 0 &&
        m_context->flags.emit_warning_on_maximum_branching_depth_discardment)
    {
        // Download data
        buffer.ssbo.ray_properties->download(buffer.ray_properties_data);
        buffer.ssbo.particle_index->download(buffer.particle_index_data);
        // Compute max and average length
        double max_intensity = 0.0, avg_intensity = 0.0;
        for (size_t i = 0; i < buffer.size; ++i)
        {
            if (buffer.particle_index_data[i] < 0)
                continue;

            double intensity = buffer.ray_properties_data[i].intensity;
            avg_intensity += intensity;
            if (intensity > max_intensity)
                max_intensity = intensity;
        }
        avg_intensity /= static_cast<double>(buffer.active_rays);
        LDPLAB_LOG_WARNING("RTSGPU (OpenGL) context %i: Pipeline reached max "\
            "branching depth %i with a total of %i still active rays, which "\
            "include a max intensity of %f and average intensity of %f",
            m_context->uid,
            m_context->parameters.maximum_branching_depth,
            buffer.active_rays,
            max_intensity,
            avg_intensity);
    }
}

void ldplab::rtsgpu_ogl::Pipeline::resetBuffers(
    OutputBuffer& output, 
    IntersectionBuffer& intersection)
{
    // Bind GPU to this thread
    LDPLAB_PROFILING_START(pipeline_reset_buffers_gl_context_binding);
    std::mutex& gpu_mutex = m_context->ogl->getGPUMutex();
    std::unique_lock<std::mutex> gpu_lock{ gpu_mutex };
    m_context->ogl->bindGlContext();
    LDPLAB_PROFILING_STOP(pipeline_reset_buffers_gl_context_binding);

    // Bind shaders
    LDPLAB_PROFILING_START(pipeline_reset_buffers_shader_binding);
    m_reset_buffer_cs->use();
    LDPLAB_PROFILING_STOP(pipeline_reset_buffers_shader_binding);

    // Bind SSBOs
    LDPLAB_PROFILING_START(pipeline_reset_buffers_ssbo_binding);
    intersection.ssbo.particle_index->bindToIndex(0);
    output.ssbo.output_per_ray->bindToIndex(1);
    LDPLAB_PROFILING_STOP(pipeline_reset_buffers_ssbo_binding);

    // Execute shader
    LDPLAB_PROFILING_START(pipeline_reset_buffers_shader_execution);
    m_reset_buffer_cs->execute(m_shader_num_work_groups);
    LDPLAB_PROFILING_STOP(pipeline_reset_buffers_shader_execution);

    // Unbind gl context
    LDPLAB_PROFILING_START(pipeline_reset_buffers_gl_context_unbinding);
    m_context->ogl->unbindGlContext();
    gpu_lock.unlock();
    LDPLAB_PROFILING_STOP(pipeline_reset_buffers_gl_context_unbinding);
}

void ldplab::rtsgpu_ogl::Pipeline::uploadInitialBatch(RayBuffer& buffer)
{
    // Bind GPU to this thread
    LDPLAB_PROFILING_START(pipeline_upload_initial_batch_gl_context_binding);
    std::mutex& gpu_mutex = m_context->ogl->getGPUMutex();
    std::unique_lock<std::mutex> gpu_lock{ gpu_mutex };
    m_context->ogl->bindGlContext();
    LDPLAB_PROFILING_STOP(pipeline_upload_initial_batch_gl_context_binding);

    // Upload data
    LDPLAB_PROFILING_START(pipeline_upload_initial_batch_data_upload);
    buffer.ssbo.particle_index->upload(buffer.particle_index_data);
    buffer.ssbo.ray_properties->upload(buffer.ray_properties_data);
    LDPLAB_PROFILING_STOP(pipeline_upload_initial_batch_data_upload);

    // Unbind gl context
    LDPLAB_PROFILING_START(pipeline_upload_initial_batch_gl_context_unbinding);
    m_context->ogl->unbindGlContext();
    gpu_lock.unlock();
    LDPLAB_PROFILING_STOP(pipeline_upload_initial_batch_gl_context_unbinding);
}
