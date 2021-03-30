#include "ParticleAcceleratorStructures.hpp"

#include "Context.hpp"
#include "IntersectionTests.hpp"

#include <limits>

ldplab::rtscpu::ParticleMeshTriangleList::ParticleMeshTriangleList(
    const std::vector<Triangle>& mesh)
    :
    m_mesh{ mesh }
{
}

bool ldplab::rtscpu::ParticleMeshTriangleList::intersects(
    const Ray& ray,
    Vec3& intersection_point,
    Vec3& intersection_normal)
{
    Vec3 t_intersection_point, t_intersection_normal;
    double min_dist = std::numeric_limits<double>::max(), t_dist;
    for (size_t i = 0; i < m_mesh.size(); ++i)
    {
        t_dist = 0.0;
        if (IntersectionTest::rayTriangle(
            ray,
            m_mesh[i],
            t_intersection_point,
            t_intersection_normal,
            t_dist))
        {
            if (t_dist < min_dist)
            {
                t_dist = min_dist;
                intersection_point = t_intersection_point;
                intersection_normal = t_intersection_normal;
            }
        }
    }
    return min_dist != std::numeric_limits<double>::max();
}

bool ldplab::rtscpu::ParticleMeshTriangleList::intersectsLineSegment(
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    Vec3 t_intersection_point, t_intersection_normal;
    double min_dist = std::numeric_limits<double>::max(), t_dist;
    for (size_t i = 0; i < m_mesh.size(); ++i)
    {
        t_dist = 0.0;
        if (IntersectionTest::lineTriangle(
            segment_origin,
            segment_end,
            m_mesh[i],
            t_intersection_point,
            t_intersection_normal,
            t_dist))
        {
            if (t_dist < min_dist)
            {
                t_dist = min_dist;
                intersection_point = t_intersection_point;
                intersection_normal = t_intersection_normal;
            }
        }
    }
    return min_dist != std::numeric_limits<double>::max();
}
