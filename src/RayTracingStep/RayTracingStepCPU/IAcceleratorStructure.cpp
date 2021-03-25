#include "IAcceleratorStructure.hpp"

#include "Context.hpp"

#include <limits>

ldplab::rtscpu::TriangleList::TriangleList(
    std::vector<Triangle> mesh)
    :
    m_mesh{ mesh }
{
}

double ldplab::rtscpu::TriangleList::
    Intersection(
        const Ray ray,
        Vec3& intersection_point,
        Vec3& intersection_normal)
{
    double min_distance = std::numeric_limits<double>::max();
    size_t mesh_index;
    bool intersected = false;
    for(size_t i = 0; i < m_mesh.size(); ++i)
    {
        double t = RayTriangleIntersection(ray, m_mesh[i]);
        if(t < min_distance || t >= 0)
        {
            min_distance = t;
            mesh_index = i;
            intersected = true;
        }
    }
    if(!intersected)
        return -1.0;
    intersection_point = ray.origin + ray.direction * min_distance;
    const Vec3 edge1 = m_mesh[mesh_index].b - m_mesh[mesh_index].a;
    const Vec3 edge2 = m_mesh[mesh_index].c - m_mesh[mesh_index].a;
    intersection_normal = glm::normalize(glm::cross(edge1,edge2));
    if(glm::dot(ray.direction,intersection_normal)>0)
        intersection_normal *= -1.0;
    return min_distance;
}

double ldplab::rtscpu::TriangleList::
    RayTriangleIntersection(const Ray& ray, 
        const Triangle& triangle)
{
    const double EPSILON = 1e-10;
    const Vec3 edge1 = triangle.b - triangle.a;
    const Vec3 edge2 = triangle.c - triangle.a;
    const Vec3 h = glm::cross(ray.direction, edge2);
    const double a = glm::dot(edge1, h);
    // Check parallelisim
    if (a > -EPSILON && a < EPSILON)
        return -1;
    const double f = 1.0/a;
    const Vec3 s = ray.origin - triangle.a;
    const double u = f * glm::dot(s,h);
    if (u < 0.0 || u > 1.0)
        return -1;
    const Vec3 q = glm::cross(s,edge1);
    const double v = f * glm::dot(ray.direction, q);
    if (v < 0.0 || u + v > 1.0)
        return -1;
    // Calculate distance to intersection point
    double t = f * glm::dot(edge2,q);
    if (t > EPSILON)
    {
        return t;
    }
    else
        return -1;
}
