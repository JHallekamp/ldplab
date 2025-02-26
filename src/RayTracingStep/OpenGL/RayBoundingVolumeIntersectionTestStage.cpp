#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSOGL

#include "RayBoundingVolumeIntersectionTestStage.hpp"

#include "Constants.hpp"
#include "Context.hpp"
#include "../../Utils/Log.hpp"
#include "../../Utils/ComputeHelper.hpp"
#include "../../Utils/Profiler.hpp"

#include <limits>
#include <sstream>

ldplab::rtsogl::RayBoundingSphereIntersectionTestStageBruteForce::
    RayBoundingSphereIntersectionTestStageBruteForce(
        Context& context)
    :
    m_context{ context }
{
    LDPLAB_LOG_INFO("RTSOGL context %i: "\
        "RayBoundingSphereIntersectionTestStageBruteForce instance created",
        m_context.uid);
}

bool ldplab::rtsogl::RayBoundingSphereIntersectionTestStageBruteForce::
    initShaders()
{
    // Create shader
    if (!m_context.shared_shaders.createShaderByName(
            constant::glsl_shader_name::ray_bounding_sphere_intersection_test_brute_force,
            m_cs_bv_intersection.shader))
        return false;

    // Compute work group size
    m_cs_bv_intersection.num_work_groups = utils::ComputeHelper::getNumWorkGroups(
        m_context.parameters.number_rays_per_buffer,
        constant::glsl_local_group_size::ray_bounding_sphere_intersection_test_brute_force);

    // Get uniform locations
    m_cs_bv_intersection.uniform_num_particles =
        m_cs_bv_intersection.shader->getUniformLocation("num_particles");
    m_cs_bv_intersection.uniform_num_rays_per_buffer =
        m_cs_bv_intersection.shader->getUniformLocation("num_rays_per_buffer");

    m_cs_bv_intersection.shader->use();
    glUniform1ui(m_cs_bv_intersection.uniform_num_particles,
        static_cast<GLuint>(m_context.particles.size()));
    glUniform1ui(m_cs_bv_intersection.uniform_num_rays_per_buffer,
        static_cast<GLuint>(m_context.parameters.number_rays_per_buffer));

    // Done
    return true;
}

void ldplab::rtsogl::RayBoundingSphereIntersectionTestStageBruteForce::setup()
{
    // For the brute force method, no setup is needed
}

void 
    ldplab::rtsogl::RayBoundingSphereIntersectionTestStageBruteForce::execute(
        RayBuffer& buffer)
{
    LDPLAB_LOG_TRACE("RTSOGL context %i: Test bounding sphere "\
        "intersections for batch buffer %i",
        m_context.uid,
        buffer.uid);

    // Bind shaders
    LDPLAB_PROFILING_START(bounding_volume_intersection_test_shader_binding);
    m_cs_bv_intersection.shader->use();
    LDPLAB_PROFILING_STOP(bounding_volume_intersection_test_shader_binding);

    // Bind SSBOs
    LDPLAB_PROFILING_START(bounding_volume_intersection_test_ssbo_binding);
    buffer.ssbo.ray_properties->bindToIndex(0);
    buffer.ssbo.particle_index->bindToIndex(1);
    BoundingSphereData* bsd = 
        (BoundingSphereData*)m_context.bounding_volume_data.get();
    bsd->ssbo.sphere_properties->bindToIndex(2);
    m_context.particle_transformation_data.ssbo.w2p->bindToIndex(3);
    LDPLAB_PROFILING_STOP(bounding_volume_intersection_test_ssbo_binding);

    // Execute shader
    LDPLAB_PROFILING_START(bounding_volume_intersection_test_shader_execution);
    m_cs_bv_intersection.shader->execute(m_cs_bv_intersection.num_work_groups);
    LDPLAB_PROFILING_STOP(bounding_volume_intersection_test_shader_execution);

    LDPLAB_LOG_TRACE("RTSOGL context %i: Bounding sphere intersection "\
        "test on batch buffer %i completed",
        m_context.uid,
        buffer.uid);
}

#endif