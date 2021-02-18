#include "InnerParticlePropagationStage.hpp"

#include "Constants.hpp"
#include "Data.hpp"
#include "Context.hpp"
#include "../../../ExperimentalSetup/Particle.hpp"
#include "../../../ExperimentalSetup/ParticleMaterial.hpp"
#include "../../../Log.hpp"
#include "../../../Utils/Profiler.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::
    LinearIndexGradientRodParticlePropagation(
        std::shared_ptr<Context> context,
        RK45 parameters)
    :
    m_context{ context },
    m_parameters{parameters},
    m_shader_num_work_groups{ 0 }
{
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "LinearIndexGradientRodParticlePropagation instance created",
        m_context->uid);
}

bool ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::initShaders(
    const ldplab::RayTracingStepGPUOpenGLInfo& info)
{
    std::string shader_path = info.shader_base_directory_path + 
        "glsl/ldplab_cs_linear_index_gradient_rod_particle_propagation.glsl";

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
            "LinearIndexGradientRodParticlePropagation", code.str());
    file.close();
    
    if (m_compute_shader != nullptr)
    {
        // Get uniform locations
        m_shader_uniform_location_num_rays_per_buffer =
            m_compute_shader->getUniformLocation("num_rays_per_buffer");
        m_shader_uniform_location_parameter_epsilon =
            m_compute_shader->getUniformLocation("parameter_epsilon");
        m_shader_uniform_location_parameter_initial_step_size =
            m_compute_shader->getUniformLocation("parameter_initial_step_size");
        m_shader_uniform_location_parameter_safety_factor =
            m_compute_shader->getUniformLocation("parameter_safety_factor");

        // Set uniforms
        m_compute_shader->use();
        glUniform1ui(m_shader_uniform_location_num_rays_per_buffer,
            m_context->parameters.number_rays_per_buffer);
        glUniform1d(m_shader_uniform_location_parameter_epsilon,
            m_parameters.epsilon);
        glUniform1d(m_shader_uniform_location_parameter_initial_step_size,
            m_parameters.initial_step_size);
        glUniform1d(m_shader_uniform_location_parameter_safety_factor,
            m_parameters.safety_factor);

        // Compute number of dispatched work groups
        const size_t num_rays_per_buffer = 
            m_context->parameters.number_rays_per_buffer;
        m_shader_num_work_groups =
            (num_rays_per_buffer / constant::glsl_local_group_sizes::
                linear_index_gradient_rod_particle_propagation) +
            (num_rays_per_buffer % constant::glsl_local_group_sizes::
                linear_index_gradient_rod_particle_propagation ? 1 : 0);
    }

    // Finish it
    m_context->ogl->unbindGlContext();
    return (m_compute_shader != nullptr);
}

void ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Execute inner particle ray "\
        "propagation on batch buffer %i",
        m_context->uid, rays.uid);
    
    // Bind GPU to this thread
    LDPLAB_PROFILING_START(inner_particle_propagation_gl_context_binding);
    std::mutex& gpu_mutex = m_context->ogl->getGPUMutex();
    std::unique_lock<std::mutex> gpu_lock{ gpu_mutex };
    m_context->ogl->bindGlContext();
    LDPLAB_PROFILING_STOP(inner_particle_propagation_gl_context_binding);
    
    // Bind shaders
    LDPLAB_PROFILING_START(inner_particle_propagation_shader_binding);
    m_compute_shader->use();
    LDPLAB_PROFILING_STOP(inner_particle_propagation_shader_binding);
    
    // Bind buffers
    LDPLAB_PROFILING_START(inner_particle_propagation_ssbo_binding);
    rays.ssbo.particle_index->bindToIndex(0);
    rays.ssbo.ray_properties->bindToIndex(1);
    intersection.ssbo.intersection_properties->bindToIndex(2);
    output.ssbo.output_per_ray->bindToIndex(3);
    RodParticleData* pd = (RodParticleData*)m_context->particle_data.get();
    pd->ssbo.rod_particles->bindToIndex(4);
    ParticleMaterialLinearOneDirectionalData* pmd =
        (ParticleMaterialLinearOneDirectionalData*)
        m_context->particle_material_data.get();
    pmd->ssbo.material->bindToIndex(5);
    LDPLAB_PROFILING_STOP(inner_particle_propagation_ssbo_binding);
    
    // Execute shader
    LDPLAB_PROFILING_START(inner_particle_propagation_shader_execution);
    m_compute_shader->execute(m_shader_num_work_groups);
    LDPLAB_PROFILING_STOP(inner_particle_propagation_shader_execution);
    
    // Unbind gl context
    LDPLAB_PROFILING_START(inner_particle_propagation_gl_context_unbinding);
    m_context->ogl->unbindGlContext();
    gpu_lock.unlock();
    LDPLAB_PROFILING_STOP(inner_particle_propagation_gl_context_unbinding);
   
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Inner particle ray propagation on "\
        "buffer %i completed",
        m_context->uid,
        rays.uid);
}
