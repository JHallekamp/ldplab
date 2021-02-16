#include "RayBoundingVolumeIntersectionTestStage.hpp"

#include "Context.hpp"
#include "../../../Log.hpp"
#include "../../../Utils/Profiler.hpp"

#include <limits>
#include <sstream>

ldplab::rtsgpu_ogl::RayBoundingSphereIntersectionTestStageBruteForce::
    RayBoundingSphereIntersectionTestStageBruteForce(
        std::shared_ptr<Context> context)
    :
    m_context{ context },
    m_bounding_spheres{ ((BoundingSphereData*) 
        context->bounding_volume_data.get())->sphere_data }
{
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "RayBoundingSphereIntersectionTestStageBruteForce instance created",
        m_context->uid);
}

bool ldplab::rtsgpu_ogl::RayBoundingSphereIntersectionTestStageBruteForce::initShaders(
    const RayTracingStepGPUOpenGLInfo& info)
{
    std::string shader_path = info.shader_base_directory_path +
        "glsl/ldplab_cs_ray_bounding_sphere_intersection_test_brute_force.glsl";

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

    m_compute_shader =
        m_context->ogl->createComputeShader(
            "RayBoundingSphereIntersectionTestStageBruteForce", code.str());
    file.close();

    // Set uniforms
    if (m_compute_shader != nullptr)
    {
        m_shader_uniform_location_num_rays_per_buffer =
            m_compute_shader->getUniformLocation("num_rays_per_buffer");
        m_shader_uniform_location_num_particles =
            m_compute_shader->getUniformLocation("num_particles");

        m_compute_shader->use();
        glUniform1ui(m_shader_uniform_location_num_rays_per_buffer,
            m_context->parameters.number_rays_per_buffer);
        glUniform1ui(m_shader_uniform_location_num_particles,
            m_context->particles.size());
    }

    m_context->ogl->unbindGlContext();
    return (m_compute_shader != nullptr);
}

void ldplab::rtsgpu_ogl::RayBoundingSphereIntersectionTestStageBruteForce::setup()
{
    // For the brute force method, no setup is needed
}

size_t 
    ldplab::rtsgpu_ogl::RayBoundingSphereIntersectionTestStageBruteForce::execute(
        RayBuffer& buffer)
{
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Test bounding sphere "\
        "intersections for batch buffer %i",
        m_context->uid,
        buffer.uid);

    // Bind GPU to this thread
    LDPLAB_PROFILING_START(bounding_volume_intersection_test_gl_context_binding);
    std::mutex& gpu_mutex = m_context->ogl->getGPUMutex();
    std::unique_lock<std::mutex> gpu_lock{ gpu_mutex };
    m_context->ogl->bindGlContext();
    LDPLAB_PROFILING_STOP(bounding_volume_intersection_test_gl_context_binding);

    // Bind shaders
    LDPLAB_PROFILING_START(bounding_volume_intersection_test_shader_binding);
    m_compute_shader->use();
    LDPLAB_PROFILING_STOP(bounding_volume_intersection_test_shader_binding);

    // Bind SSBOs
    LDPLAB_PROFILING_START(bounding_volume_intersection_test_ssbo_binding);
    buffer.ray_origin_ssbo->bindToIndex(0);
    buffer.ray_direction_ssbo->bindToIndex(1);
    buffer.index_ssbo->bindToIndex(2);
    buffer.min_bounding_volume_distance_ssbo->bindToIndex(3);
    const BoundingSphereData* bsd = 
        (BoundingSphereData*)m_context->bounding_volume_data.get();
    bsd->ssbo.center->bindToIndex(4);
    bsd->ssbo.radius->bindToIndex(5);
    m_context->particle_transformation_data.ssbo.w2p_rotation_scale->bindToIndex(6);
    m_context->particle_transformation_data.ssbo.w2p_translation->bindToIndex(7);
    LDPLAB_PROFILING_STOP(bounding_volume_intersection_test_ssbo_binding);

    // Execute shader
    LDPLAB_PROFILING_START(bounding_volume_intersection_test_shader_execution);
    m_compute_shader->execute(buffer.size / 64);
    LDPLAB_PROFILING_STOP(bounding_volume_intersection_test_shader_execution);

    // Download index data
    LDPLAB_PROFILING_START(bounding_volume_intersection_test_data_download);
    buffer.index_ssbo->download(buffer.index_data);
    LDPLAB_PROFILING_STOP(bounding_volume_intersection_test_data_download);

    // Unbind gl context
    LDPLAB_PROFILING_START(bounding_volume_intersection_test_gl_context_unbinding);
    m_context->ogl->unbindGlContext();
    gpu_lock.unlock();
    LDPLAB_PROFILING_STOP(bounding_volume_intersection_test_gl_context_unbinding);

    // Calculate number of rays hitting bounding spheres
    size_t num_rays_hitting_boundary_sphere = 0;
    for (size_t i = 0; i < buffer.size; ++i)
    {
        if (buffer.index_data[i] >= 0)
            ++num_rays_hitting_boundary_sphere;
    }

    size_t num_rays_exiting_scene = 
        buffer.active_rays - num_rays_hitting_boundary_sphere;
    buffer.active_rays = num_rays_hitting_boundary_sphere;
    buffer.world_space_rays = 0;

    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Bounding sphere intersection "\
        "test on batch buffer %i completed, of %i tested rays %i hit "\
        "bounding spheres and %i rays exited the scene",
        m_context->uid,
        buffer.uid,
        num_rays_hitting_boundary_sphere + num_rays_exiting_scene,
        num_rays_hitting_boundary_sphere,
        num_rays_exiting_scene);

    return num_rays_hitting_boundary_sphere;
}
