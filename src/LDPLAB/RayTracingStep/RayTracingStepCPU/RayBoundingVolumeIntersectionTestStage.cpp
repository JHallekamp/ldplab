#include "RayBoundingVolumeIntersectionTestStage.hpp"

ldplab::rtscpu::RayBoundingSphereIntersectionTestStageBruteForce::
    RayBoundingSphereIntersectionTestStageBruteForce(
        std::shared_ptr<Context> context)
    :
    m_context{ context }
{
}

void ldplab::rtscpu::RayBoundingSphereIntersectionTestStageBruteForce::setup()
{
    // For the brute force method, no setup is needed
}

void ldplab::rtscpu::RayBoundingSphereIntersectionTestStageBruteForce::execute(
    RayBuffer& buffer)
{
    Vec3 unit = { 1, 1, 1 };
    for (size_t i = 0; i < buffer.size; ++i)
    {
        if (buffer.index_data[i] < m_context->particles.size())
            continue;

        double min_d = -1.0;
        size_t min_j = 0;

        const Vec3 o = buffer.ray_data[i].origin;
        for (size_t j = 0; j < m_context->particles.size(); ++j)
        {
            const Vec3 oc = o - ((BoundingVolumeSphere*)
                m_context->particles[j].bounding_volume.get())->center;
            const double dot_uoc = glm::dot(unit, oc);
            
            if (dot_uoc < 0)
                continue;

            const double rr = ((BoundingVolumeSphere*)
                m_context->particles[j].bounding_volume.get())->radius *
                ((BoundingVolumeSphere*)
                    m_context->particles[j].bounding_volume.get())->radius;

            const double t = dot_uoc * dot_uoc - (glm::dot(oc, oc) - rr);
            double d = 0;

            if (t < 0)
                continue;
            else if (t == 0)
                d = dot_uoc;
            else
            {
                d = dot_uoc - sqrt(t);
                if (signbit(d))
                    continue;
            }

            if (d < min_d || min_d < 0)
            {
                min_d = d;
                min_j = j;
            }
        }

        if (min_d < 0)
        {
            // ray exits scene
            buffer.index_data[i] = -1;
            buffer.active_rays--;
        }
        else
        {
            // ray hits particle min_j boundary volume
            buffer.index_data[i] = min_j;
            transformRayFromWorldToParticleSpace(buffer.ray_data[i], min_j);
        }
    }
}

inline void ldplab::rtscpu::RayBoundingSphereIntersectionTestStageBruteForce::
    transformRayFromWorldToParticleSpace(Ray& ray, size_t pidx) const
{
    ray.origin += m_context->particle_transformations[pidx].w2p_translation;
    ray.origin = m_context->particle_transformations[pidx].w2p_rotation_scale *
        ray.origin;
}
