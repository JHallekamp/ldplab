#include "RayParticleIntersectionTestStage.hpp"

#include "Constants.hpp"
#include "Context.hpp"
#include "Data.hpp"

#include "../../../SimulationState.hpp"
#include "../../../Log.hpp"
#include "../../../Utils/Profiler.hpp"
#include "../../../Utils/ComputeHelper.hpp"

#include <glm/glm.hpp>
#include <cmath>
#include <sstream>

ldplab::rtsgpu_ogl::RodParticleIntersectionTest::
    RodParticleIntersectionTest(std::shared_ptr<Context> context)
    :
    m_context{ context }
{ }

bool ldplab::rtsgpu_ogl::RodParticleIntersectionTest::initShaders()
{
    // Create shader
    if (!m_context->shared_shaders.createShaderByName(
            constant::glsl_shader_name::rod_particle_intersection_test,
            m_cs_intersection.shader))
        return false;

    // Compute work group size
    m_cs_intersection.num_work_groups = ComputeHelper::getNumWorkGroups(
        m_context->parameters.number_rays_per_buffer,
        constant::glsl_local_group_size::rod_particle_intersection_test);

    // Get uniform locations
    m_cs_intersection.uniform_num_particles =
        m_cs_intersection.shader->getUniformLocation("num_particles");
    m_cs_intersection.uniform_num_rays_per_buffer =
        m_cs_intersection.shader->getUniformLocation("num_rays_per_buffer");

    m_cs_intersection.shader->use();
    glUniform1ui(m_cs_intersection.uniform_num_particles,
        static_cast<GLuint>(m_context->particles.size()));
    glUniform1ui(m_cs_intersection.uniform_num_rays_per_buffer,
        static_cast<GLuint>(m_context->parameters.number_rays_per_buffer));

    // Done
    return true;
}

void ldplab::rtsgpu_ogl::RodParticleIntersectionTest::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection)
{
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Execute ray particle intersection"\
        " test on batch buffer %i",
        m_context->uid, rays.uid);
    
    // Bind shaders
    LDPLAB_PROFILING_START(particle_intersection_test_shader_binding);
    m_cs_intersection.shader->use();
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
    m_cs_intersection.shader->execute(m_cs_intersection.num_work_groups);
    LDPLAB_PROFILING_STOP(particle_intersection_test_shader_execution);

    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Ray particle intersection test on "\
        "batch buffer %i completed",
        m_context->uid, 
        rays.uid);
}