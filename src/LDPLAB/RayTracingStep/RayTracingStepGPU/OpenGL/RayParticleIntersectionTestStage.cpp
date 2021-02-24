#include "RayParticleIntersectionTestStage.hpp"

#include "Constants.hpp"
#include "Context.hpp"
#include "Data.hpp"

#include "../../../SimulationState.hpp"
#include "../../../Log.hpp"
#include "../../../Utils/Profiler.hpp"

#include <glm/glm.hpp>
#include <cmath>
#include <sstream>

ldplab::rtsgpu_ogl::RodParticleIntersectionTest::
    RodParticleIntersectionTest(std::shared_ptr<Context> context)
    :
    m_context{ context }
{ }

bool ldplab::rtsgpu_ogl::RodParticleIntersectionTest::initShaders(
    const RayTracingStepGPUOpenGLInfo& info)
{
    std::string shader_path = info.shader_base_directory_path +
        "glsl/ldplab_cs_rod_particle_intersection_test.glsl";

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
            "RodParticleIntersectionTest", code.str());
    file.close();

    if (m_compute_shader != nullptr)
    {
        // Get uniform location
        m_shader_uniform_location_num_rays_per_buffer =
            m_compute_shader->getUniformLocation("num_rays_per_buffer");
        m_shader_uniform_location_num_particles =
            m_compute_shader->getUniformLocation("num_particles");

        // Set uniforms
        m_compute_shader->use();
        glUniform1ui(m_shader_uniform_location_num_rays_per_buffer,
            m_context->parameters.number_rays_per_buffer);
        glUniform1ui(m_shader_uniform_location_num_particles,
            m_context->particles.size());

        // Compute number of dispatched work groups
        const size_t num_rays_per_buffer =
            m_context->parameters.number_rays_per_buffer;
        m_shader_num_work_groups =
            (num_rays_per_buffer / constant::glsl_local_group_sizes::
                rod_particle_intersection_test) +
            (num_rays_per_buffer % constant::glsl_local_group_sizes::
                rod_particle_intersection_test ? 1 : 0);
    }

    m_context->ogl->unbindGlContext();
    return (m_compute_shader != nullptr);
}

void ldplab::rtsgpu_ogl::RodParticleIntersectionTest::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection)
{
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Execute ray particle intersection"\
        " test on batch buffer %i",
        m_context->uid, rays.uid);
    
    // Bind GPU to this thread
    LDPLAB_PROFILING_START(particle_intersection_test_gl_context_binding);
    std::mutex& gpu_mutex = m_context->ogl->getGPUMutex();
    std::unique_lock<std::mutex> gpu_lock{ gpu_mutex };
    m_context->ogl->bindGlContext();
    LDPLAB_PROFILING_STOP(particle_intersection_test_gl_context_binding);

    // Bind shaders
    LDPLAB_PROFILING_START(particle_intersection_test_shader_binding);
    m_compute_shader->use();
    LDPLAB_PROFILING_STOP(particle_intersection_test_shader_binding);

    // Bind SSBOs
    LDPLAB_PROFILING_START(particle_intersection_test_ssbo_binding);
    rays.ssbo.ray_properties->bindToIndex(0);
    rays.ssbo.particle_index->bindToIndex(1);
    intersection.ssbo.intersection_properties->bindToIndex(2);
    intersection.ssbo.particle_index->bindToIndex(3);
    RodParticleData* pd = (RodParticleData*)m_context->particle_data.get();
    pd->ssbo.rod_particles->bindToIndex(4);
    m_context->particle_transformation_data.ssbo.p2w->bindToIndex(5);
    LDPLAB_PROFILING_STOP(particle_intersection_test_ssbo_binding);

    // Execute shader
    LDPLAB_PROFILING_START(particle_intersection_test_shader_execution);
    m_compute_shader->execute(m_shader_num_work_groups);
    LDPLAB_PROFILING_STOP(particle_intersection_test_shader_execution);

    // Download index data
    LDPLAB_PROFILING_START(particle_intersection_test_data_download);
    rays.ssbo.particle_index->download(rays.particle_index_data);   

    //std::vector<RayBuffer::RayProperties> ray_properties(rays.size);
    //std::vector<int32_t> ray_index(rays.size);
    //std::vector<IntersectionBuffer::IntersectionProperties> intersection_properties(rays.size);
    //std::vector<int32_t> intersection_index(rays.size);
    //rays.ssbo.particle_index->download(ray_index.data());
    //rays.ssbo.ray_properties->download(ray_properties.data());
    //intersection.ssbo.particle_index->download(intersection_index.data());
    //intersection.ssbo.intersection_properties->download(intersection_properties.data());

    LDPLAB_PROFILING_STOP(particle_intersection_test_data_download);

    // Unbind gl context
    LDPLAB_PROFILING_START(particle_intersection_test_gl_context_unbinding);
    m_context->ogl->unbindGlContext();
    gpu_lock.unlock();
    LDPLAB_PROFILING_STOP(particle_intersection_test_gl_context_unbinding);
    
    size_t num_world_space_rays = 0;
    for (size_t i = 0; i < rays.size; ++i)
    {
        if (rays.particle_index_data[i] >= 0 &&
            rays.particle_index_data[i] >= m_context->particles.size())
            ++num_world_space_rays;
    }
    rays.world_space_rays = num_world_space_rays;

    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Ray particle intersection test on "\
        "batch buffer %i completed",
        m_context->uid, 
        rays.uid);
}