#ifndef LDPLAB_BUILD_OPTION_DISABLE_RTSGPU_OGL

#include "InnerParticlePropagationStage.hpp"

#include "Constants.hpp"
#include "Data.hpp"
#include "Context.hpp"
#include "../../../ExperimentalSetup/Particle.hpp"
#include "../../../ExperimentalSetup/ParticleMaterial.hpp"
#include "../../../Utils/Log.hpp"
#include "../../../Utils/ComputeHelper.hpp"
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
    m_cs_inner_particle_propagation{ }
{
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "LinearIndexGradientRodParticlePropagation instance created",
        m_context->uid);
}

bool ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::initShaders()
{
    // Create shader
    if (!m_context->shared_shaders.createShaderByName(
            constant::glsl_shader_name::linear_index_gradient_rod_particle_propagation,
            m_cs_inner_particle_propagation.shader))
        return false;

    // Compute work group size
    m_cs_inner_particle_propagation.num_work_groups = 
        utils::ComputeHelper::getNumWorkGroups(
            m_context->parameters.number_rays_per_buffer,
            constant::glsl_local_group_size::linear_index_gradient_rod_particle_propagation);

    // Get uniform locations
    m_cs_inner_particle_propagation.uniform_num_rays_per_buffer =
        m_cs_inner_particle_propagation.shader->getUniformLocation("num_rays_per_buffer");
    m_cs_inner_particle_propagation.uniform_parameter_initial_step_size =
        m_cs_inner_particle_propagation.shader->getUniformLocation("parameter_initial_step_size");
    m_cs_inner_particle_propagation.uniform_parameter_epsilon =
        m_cs_inner_particle_propagation.shader->getUniformLocation("parameter_epsilon");
    m_cs_inner_particle_propagation.uniform_parameter_safety_factor =
        m_cs_inner_particle_propagation.shader->getUniformLocation("parameter_safety_factor");

    m_cs_inner_particle_propagation.shader->use();
    glUniform1ui(m_cs_inner_particle_propagation.uniform_num_rays_per_buffer,
        static_cast<GLuint>(m_context->parameters.number_rays_per_buffer));
    glUniform1d(m_cs_inner_particle_propagation.uniform_parameter_epsilon,
        m_parameters.epsilon);
    glUniform1d(m_cs_inner_particle_propagation.uniform_parameter_initial_step_size,
        m_parameters.initial_step_size);
    glUniform1d(m_cs_inner_particle_propagation.uniform_parameter_safety_factor,
        m_parameters.safety_factor);

    // Done
    return true;
}

void ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Execute inner particle ray "\
        "propagation on batch buffer %i",
        m_context->uid, rays.uid);
    
    // Bind shaders
    LDPLAB_PROFILING_START(inner_particle_propagation_shader_binding);
    m_cs_inner_particle_propagation.shader->use();
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
    m_cs_inner_particle_propagation.shader->execute(
        m_cs_inner_particle_propagation.num_work_groups);
    LDPLAB_PROFILING_STOP(inner_particle_propagation_shader_execution);
   
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Inner particle ray propagation on "\
        "buffer %i completed",
        m_context->uid,
        rays.uid);
}

#endif