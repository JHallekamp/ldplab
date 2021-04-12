#include "ParticleAcceleratorStructures.hpp"

#include "Context.hpp"
#include "IntersectionTests.hpp"

#include <limits>

ldplab::rtscpu::ParticleMeshList::ParticleMeshList(
    const std::vector<Triangle>& mesh)
    :
    m_mesh{ mesh }
{
}

bool ldplab::rtscpu::ParticleMeshList::intersects(
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

bool ldplab::rtscpu::ParticleMeshList::intersectsLineSegment(
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

ldplab::rtscpu::ParticleMeshOctree::ParticleMeshOctree(
    const std::vector<Triangle>& mesh, 
    size_t octree_depth)
{
    construct(mesh, octree_depth);
}

bool ldplab::rtscpu::ParticleMeshOctree::intersects(
    const Ray& ray, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    return false;
}

bool ldplab::rtscpu::ParticleMeshOctree::intersectsLineSegment(
    const Vec3& segment_origin, 
    const Vec3& segment_end,
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    return false;
}

void ldplab::rtscpu::ParticleMeshOctree::construct(
    const std::vector<Triangle>& mesh, 
    size_t octree_depth)
{
    // Get octree AABB min and max
    Vec3 min = Vec3(
        std::numeric_limits<double>::max(), 
        std::numeric_limits<double>::max(), 
        std::numeric_limits<double>::max());
    Vec3 max = Vec3(
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::min());
    double t;
    for (size_t i = 0; i < mesh.size(); ++i)
    {
        t = std::min(mesh[i].a.x, std::min(mesh[i].b.x, mesh[i].c.x));
        if (t < min.x)
            min.x = t;
        t = std::max(mesh[i].a.x, std::max(mesh[i].b.x, mesh[i].c.x));
        if (t > max.x)
            max.x = t;
        t = std::min(mesh[i].a.y, std::min(mesh[i].b.y, mesh[i].c.y));
        if (t < min.y)
            min.y = t;
        t = std::max(mesh[i].a.y, std::max(mesh[i].b.y, mesh[i].c.y));
        if (t > max.y)
            max.y = t;
        t = std::min(mesh[i].a.z, std::min(mesh[i].b.z, mesh[i].c.z));
        if (t < min.z)
            min.z = t;
        t = std::max(mesh[i].a.z, std::max(mesh[i].b.z, mesh[i].c.z));
        if (t > max.z)
            max.z = t;
    }

    m_octree_aabb.center = (min + max) * 0.5;
    m_octree_aabb.extents = max - m_octree_aabb.center;

    // Start creating the base nodes
}
