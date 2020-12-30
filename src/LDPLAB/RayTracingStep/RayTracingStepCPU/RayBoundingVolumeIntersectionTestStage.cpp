#include "RayBoundingVolumeIntersectionTestStage.hpp"
#include "Context.hpp"
#include "../../Log.hpp"

#include <limits>

ldplab::rtscpu::RayBoundingSphereIntersectionTestStageBruteForce::
    RayBoundingSphereIntersectionTestStageBruteForce(
        std::shared_ptr<Context> context)
    :
    m_context{ context }
{
    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "RayBoundingSphereIntersectionTestStageBruteForce instance created",
        m_context->uid);
}

void ldplab::rtscpu::RayBoundingSphereIntersectionTestStageBruteForce::setup()
{
    // For the brute force method, no setup is needed
}

void ldplab::rtscpu::RayBoundingSphereIntersectionTestStageBruteForce::execute(
    RayBuffer& buffer)
{
    LDPLAB_LOG_TRACE("RTSCPU context %i: Test bounding sphere intersections "\
        "for batch buffer %i",
        m_context->uid,
        buffer.uid);

    size_t num_rays_exiting_scene = 0;
    size_t num_rays_hitting_boundary_sphere = 0;

    const Vec3 unit = { 1, 1, 1 };
    for (size_t i = 0; i < buffer.size; ++i)
    {
        if (buffer.index_data[i] < m_context->particles.size() ||
            buffer.index_data[i] == -1)
            continue;

        double min_d = -1.0;
        size_t min_j = 0;

        const Vec3& o = buffer.ray_data[i].origin;
        for (size_t j = 0; j < m_context->particles.size(); ++j)
        {
            //const Vec3& oc = o - ((BoundingVolumeSphere*)
            //    m_context->particles[j].bounding_volume.get())->center;
            //const double rr = ((BoundingVolumeSphere*)
            //    m_context->particles[j].bounding_volume.get())->radius *
            //    ((BoundingVolumeSphere*)
            //        m_context->particles[j].bounding_volume.get())->radius;
            const Vec3& oc = o - 
                m_context->transformed_bounding_spheres[j].center;
            const double rr =
                m_context->transformed_bounding_spheres[j].radius *
                m_context->transformed_bounding_spheres[j].radius;

            const double q = glm::dot(oc, oc) - rr;
            if (q <= 1e-9) //std::numeric_limits<double>::epsilon())
                continue;

            const double p = glm::dot(buffer.ray_data[i].direction, oc);
            const double discriminant = p * p - q;
            if (discriminant < 1e-9) //0.0)
                continue;

            const double d_root = sqrt(discriminant);
            const double dist = -p - d_root;

            double t = buffer.min_bounding_volume_distance_data[i];
            if (dist <= buffer.min_bounding_volume_distance_data[i])
                continue;

            if (dist < min_d || min_d < 0)
            {
                min_d = dist;
                min_j = j;
            }
        }

        if (min_d < 0)
        {
            // ray exits scene
            buffer.index_data[i] = -1;
            ++num_rays_exiting_scene;
        }
        else
        {
            // ray hits particle min_j boundary volume
            buffer.index_data[i] = min_j;
            buffer.min_bounding_volume_distance_data[i] = min_d;
            transformRayFromWorldToParticleSpace(buffer.ray_data[i], min_j);
            ++num_rays_hitting_boundary_sphere;
        }
    }

    buffer.active_rays = num_rays_hitting_boundary_sphere;
    buffer.world_space_rays = 0;

    LDPLAB_LOG_TRACE("RTSCPU context %i: Bounding sphere intersection "\
        "test on batch buffer %i completed, of %i tested rays %i hit "\
        "bounding spheres and %i rays exited the scene",
        m_context->uid,
        buffer.uid,
        num_rays_hitting_boundary_sphere + num_rays_exiting_scene,
        num_rays_hitting_boundary_sphere,
        num_rays_exiting_scene);
}

inline void ldplab::rtscpu::RayBoundingSphereIntersectionTestStageBruteForce::
    transformRayFromWorldToParticleSpace(Ray& ray, size_t pidx) const
{
    ray.origin = m_context->particle_transformations[pidx].w2p_rotation_scale *
        (ray.origin + 
            m_context->particle_transformations[pidx].w2p_translation);
    ray.direction = glm::normalize(
        m_context->particle_transformations[pidx].w2p_rotation_scale *
            ray.direction);
}
