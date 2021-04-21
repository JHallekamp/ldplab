#include "RayParticleIntersectionTestStage.hpp"

#include "Context.hpp"
#include "Data.hpp"

#include <LDPLAB/SimulationState.hpp>
#include "../../Utils/Log.hpp"

#include <glm/glm.hpp>

#include <cmath>

ldplab::rtscpu::RayParticleGenericGeometryIntersectionTest::
    RayParticleGenericGeometryIntersectionTest(
        Context& context)
    :
    m_context{ context }
{ }

void ldplab::rtscpu::RayParticleGenericGeometryIntersectionTest::execute(
    RayBuffer& rays, 
    IntersectionBuffer& intersection)
{
    LDPLAB_LOG_TRACE("RTSCPU context %i: Execute ray particle intersection"\
        " test on batch buffer %i",
        m_context.uid, rays.uid);
    size_t num_hit_rays = 0;
    size_t num_missed_rays = 0;

    for (size_t i = 0; i < rays.size; i++)
    {
        if (rays.index_data[i] < 0 ||
            rays.index_data[i] >= m_context.particles.size() ||
            rays.index_data[i] == intersection.particle_index[i])
            continue;

        Ray& ray = rays.ray_data[i];
        Vec3& inter_point = intersection.point[i];
        Vec3& inter_normal = intersection.normal[i];

        if (m_context.particle_data->geometries[i]->intersects(
            ray,
            inter_point,
            inter_normal))
        {
            // Transformation to world space
            ParticleTransformation& trans = m_context.
                particle_transformations[rays.index_data[i]];
            ray.origin = trans.p2w_scale_rotation * ray.origin +
                trans.p2w_translation;
            ray.direction =
                glm::normalize(trans.p2w_scale_rotation * ray.direction);
            // Ray missed particle
            rays.index_data[i] =
                static_cast<int32_t>(m_context.particles.size());
            num_missed_rays++;
        }
        else
        {
            intersection.particle_index[i] = rays.index_data[i];
            num_hit_rays++;
        }
    }

    rays.world_space_rays += num_missed_rays;

    LDPLAB_LOG_TRACE("RTSCPU context %i: Ray particle intersection test on "\
        "batch buffer %i completed, of %i tested rays %i rays hit particles "\
        "and %i rays missed",
        m_context.uid,
        rays.uid,
        num_hit_rays + num_missed_rays,
        num_hit_rays,
        num_missed_rays);
}