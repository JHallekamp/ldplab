#include "ParticleAcceleratorStructures.hpp"

#include "Context.hpp"
#include "IntersectionTests.hpp"

#include <array>
#include <limits>
#include <utility>

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
    size_t min_index = m_mesh.size();
    double min_dist = std::numeric_limits<double>::max(), t_dist;
    for (size_t i = 0; i < m_mesh.size(); ++i)
    {
        if (IntersectionTest::rayTriangle(ray, m_mesh[i], t_dist))
        {
            if (t_dist < min_dist)
            {
                min_index = i;
                min_dist = t_dist;
            }
        }
    }
    if (min_index < m_mesh.size())
    {
        intersection_point = ray.origin + ray.direction * min_dist;
        const Vec3 edge1 = m_mesh[min_index].b - m_mesh[min_index].a;
        const Vec3 edge2 = m_mesh[min_index].c - m_mesh[min_index].a;
        intersection_normal = glm::normalize(glm::cross(edge1, edge2));
        if (glm::dot(intersection_normal, ray.direction) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }
    return false;
}

bool ldplab::rtscpu::ParticleMeshList::intersectsLineSegment(
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    size_t min_index = m_mesh.size();
    double min_dist = std::numeric_limits<double>::max(), t_dist;
    for (size_t i = 0; i < m_mesh.size(); ++i)
    {
        if (IntersectionTest::lineTriangle(
            segment_origin, segment_end, m_mesh[i], t_dist))
        {
            if (t_dist < min_dist)
            {
                min_index = i;
                min_dist = t_dist;
            }
        }
    }
    if (min_index < m_mesh.size())
    {
        const Vec3 dir = segment_end - segment_origin;
        intersection_point = segment_origin + dir * min_dist;
        const Vec3 edge1 = m_mesh[min_index].b - m_mesh[min_index].a;
        const Vec3 edge2 = m_mesh[min_index].c - m_mesh[min_index].a;
        intersection_normal = glm::normalize(glm::cross(edge1, edge2));
        if (glm::dot(intersection_normal, dir) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }
    return false;
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
    if (octree_depth == 0)
        return;
    m_octree_depth = 0;

    // Create root node
    m_nodes.emplace_back();
    OctreeNode root_node = m_nodes.back();

    // Get octree AABB min and max
    root_node.aabb.min = Vec3(
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max());
    root_node.aabb.max = Vec3(
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::min());

    for (size_t i = 0; i < mesh.size(); ++i)
    {
        root_node.aabb.min.x = std::min(root_node.aabb.min.x,
            std::min(mesh[i].a.x, std::min(mesh[i].b.x, mesh[i].c.x)));
        root_node.aabb.max.x = std::min(root_node.aabb.max.x,
            std::min(mesh[i].a.x, std::min(mesh[i].b.x, mesh[i].c.x)));

        root_node.aabb.min.y = std::min(root_node.aabb.min.y,
            std::min(mesh[i].a.y, std::min(mesh[i].b.y, mesh[i].c.y)));
        root_node.aabb.max.y = std::min(root_node.aabb.max.y,
            std::min(mesh[i].a.y, std::min(mesh[i].b.y, mesh[i].c.y)));

        root_node.aabb.min.z = std::min(root_node.aabb.min.z,
            std::min(mesh[i].a.z, std::min(mesh[i].b.z, mesh[i].c.z)));
        root_node.aabb.max.z = std::min(root_node.aabb.max.z,
            std::min(mesh[i].a.z, std::min(mesh[i].b.z, mesh[i].c.z)));
    }

    // Start creating the base nodes
    std::vector<std::vector<OctreeNode>> layers;
    layers.emplace_back();
    std::vector<OctreeNode>& leaf_nodes = layers.back();

    size_t num_branches = static_cast<size_t>(1) << octree_depth;
    const double side_length_x = (root_node.aabb.max.x - root_node.aabb.min.x) /
        static_cast<double>(num_branches);
    const double side_length_y = (root_node.aabb.max.y - root_node.aabb.min.y) /
        static_cast<double>(num_branches);
    const double side_length_z = (root_node.aabb.max.z - root_node.aabb.min.z) /
        static_cast<double>(num_branches);
    const size_t invalid_index = std::numeric_limits<size_t>::max();
    for (size_t x = 0; x < num_branches; ++x)
    {
        for (size_t y = 0; y < num_branches; ++y)
        {
            for (size_t z = 0; z < num_branches; ++z)
            {
                std::vector<Triangle> triangles;
                leaf_nodes.emplace_back();
                OctreeNode& node = leaf_nodes.back();
                node.aabb.min.x = static_cast<double>(x) * side_length_x;
                node.aabb.min.y = static_cast<double>(y) * side_length_y;
                node.aabb.min.z = static_cast<double>(z) * side_length_z;
                node.aabb.max.x = static_cast<double>(x + 1) * side_length_x;
                node.aabb.max.y = static_cast<double>(y + 1) * side_length_y;
                node.aabb.max.z = static_cast<double>(z + 1) * side_length_z;
                for (size_t i = 0; i < mesh.size(); ++i)
                {
                    if (IntersectionTest::triangleAABB(mesh[i], node.aabb))
                        triangles.push_back(mesh[i]);
                }
                if (triangles.size() > 0)
                {
                    node.num_children = 1;
                    node.children[0] = m_triangle_arrays.size();
                    m_triangle_arrays.emplace_back(triangles);
                }
            }
        }
    }

    // Create octree structure
    size_t sz_d, sz_d2;;
    for (size_t i = m_octree_depth; i > 0; --i)
    {
        sz_d2 = num_branches * num_branches;
        sz_d = num_branches;
        num_branches = 1 << (i - 1);
        layers.emplace_back();
        std::vector<OctreeNode>& prev_layer = layers[layers.size() - 2];
        std::vector<OctreeNode>& cur_layer = layers[layers.size() - 1];
        for (size_t x = 0; x < num_branches; ++x)
        {
            for (size_t y = 0; y < num_branches; ++y)
            {
                for (size_t z = 0; z < num_branches; ++z)
                {
                    cur_layer.emplace_back();
                    OctreeNode& cur_node = cur_layer.back();
                    for (size_t t = 0; t < 8; ++t)
                    {
                        size_t idx =
                            ((t & 1) ? x * 2 + 1 : x * 2) * sz_d2 +
                            ((t & 2) ? y * 2 + 1 : y * 2) * sz_d +
                            ((t & 4) ? z * 2 + 1 : z * 2);
                        if (prev_layer[idx].num_children)
                            cur_node.children[cur_node.num_children++] = idx;
                        cur_node.aabb.min.x = std::fmin(cur_node.aabb.min.x, prev_layer[idx].aabb.min.x);
                        cur_node.aabb.min.y = std::fmin(cur_node.aabb.min.y, prev_layer[idx].aabb.min.y);
                        cur_node.aabb.min.z = std::fmin(cur_node.aabb.min.z, prev_layer[idx].aabb.min.z);
                        cur_node.aabb.max.x = std::fmax(cur_node.aabb.min.x, prev_layer[idx].aabb.min.x);
                        cur_node.aabb.max.y = std::fmax(cur_node.aabb.min.y, prev_layer[idx].aabb.min.y);
                        cur_node.aabb.max.z = std::fmax(cur_node.aabb.min.z, prev_layer[idx].aabb.min.z);
                    }
                }
            }
        }
    }

    // Add temp nodes that hold information to the tree.
    struct TempStackFrame
    {
        TempStackFrame(
            OctreeNode& node, 
            std::vector<OctreeNode>& child_layer,
            size_t depth)
            :
            node { node },
            child_layer{ child_layer },
            depth{ depth },
            child { 0 }
        { }
        OctreeNode& node;
        std::vector<OctreeNode>& child_layer;
        size_t depth;
        size_t child;
    };
    std::vector<TempStackFrame> temp_stack;

    m_nodes.push_back(layers.back().back());
    temp_stack.emplace_back(m_nodes.back(), layers[layers.size() - 2], 1);
    size_t cur_stack_frame = 0;
    while (cur_stack_frame != invalid_index)
    {
        TempStackFrame& sf = temp_stack[cur_stack_frame];
        if (sf.child < sf.node.num_children)
        {
            OctreeNode& t_node = sf.child_layer[sf.node.children[sf.child]];
            sf.node.children[sf.child++] = m_nodes.size();
            m_nodes.push_back(t_node);

            if (sf.depth < m_octree_depth)
            {
                temp_stack.emplace_back(
                    m_nodes.back(), 
                    layers[layers.size() - 1 - sf.depth], 
                    sf.depth + 1);
                ++cur_stack_frame;
            }
        }
        else
        {
            temp_stack.pop_back();
            if (cur_stack_frame == 0)
                cur_stack_frame = invalid_index;
            else
                --cur_stack_frame;
        }
    }
}

bool ldplab::rtscpu::ParticleMeshOctree::intersectRecursive(
    const OctreeNode& node, 
    const size_t depth,
    const Ray& ray, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    if (depth == m_octree_depth)
    {
        return intersectBase(
            m_triangle_arrays[node.children[0]],
            ray,
            intersection_point,
            intersection_normal);
    }
    else
    {
        std::array<std::pair<size_t, double>, 8> nodes;
        double min, max;
        size_t intersections = 0;
        // Check for intersections with the subdivisions
        for (size_t i = 0; i < node.num_children; ++i)
        {
            const size_t c = node.children[i];
            if (IntersectionTest::rayAABB(ray, m_nodes[c].aabb, min, max))
            {
                if (min >= 0.0)
                    nodes[intersections++] = std::pair<size_t, double>(c, min);
                else
                    nodes[intersections++] = std::pair<size_t, double>(c, max);
            }   
        }
        // Sort based on distance of intersection
        for (size_t j = 0; j < intersections; ++j)
        {
            size_t insert_at = 0;
            for (size_t k = 0; k < j; ++k)
            {
                if (nodes[k].second <= nodes[j].second)
                    ++insert_at;
                else
                    break;
            }
            for (size_t k = j; k > insert_at; --k)
                std::swap(nodes[k], nodes[k - 1]);
        }
        // Recursive calls
        for (size_t j = 0; j < intersections; ++j)
        {
            if (intersectRecursive(
                m_nodes[nodes[j].first],
                depth + 1,
                ray,
                intersection_point,
                intersection_normal))
            {
                return true;
            }
        }
    }
    return false;
}

bool ldplab::rtscpu::ParticleMeshOctree::intersectBase(
    const utils::Array<Triangle>& triangles, 
    const Ray& ray,
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    size_t min_index = triangles.size();
    double min_dist = std::numeric_limits<double>::max(), t_dist;
    for (size_t i = 0; i < triangles.size(); ++i)
    {
        if (IntersectionTest::rayTriangle(ray, triangles[i], t_dist))
        {
            if (t_dist < min_dist)
            {
                min_index = i;
                min_dist = t_dist;
            }
        }
    }
    if (min_index < triangles.size())
    {
        intersection_point = ray.origin + ray.direction * min_dist;
        const Vec3 edge1 = triangles[min_index].b - triangles[min_index].a;
        const Vec3 edge2 = triangles[min_index].c - triangles[min_index].a;
        intersection_normal = glm::normalize(glm::cross(edge1, edge2));
        if (glm::dot(intersection_normal, ray.direction) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }
    return false;
}

bool ldplab::rtscpu::ParticleMeshOctree::intersectSegmentRecursive(
    const OctreeNode& node, 
    const size_t depth, 
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    if (depth == m_octree_depth)
    {
        return intersectSegmentBase(
            m_triangle_arrays[node.children[0]],
            segment_origin,
            segment_end,
            intersection_point,
            intersection_normal);
    }
    else
    {
        std::array<std::pair<size_t, double>, 8> nodes;
        double min, max;
        size_t intersections = 0;
        // Check for intersections with the subdivisions
        for (size_t i = 0; i < node.num_children; ++i)
        {
            const size_t c = node.children[i];
            if (IntersectionTest::lineAABB(
                segment_origin, segment_end, m_nodes[c].aabb, min, max))
            {
                if (min >= 0.0)
                    nodes[intersections++] = std::pair<size_t, double>(c, min);
                else
                    nodes[intersections++] = std::pair<size_t, double>(c, max);
            }
        }
        // Sort based on distance of intersection
        for (size_t j = 0; j < intersections; ++j)
        {
            size_t insert_at = 0;
            for (size_t k = 0; k < j; ++k)
            {
                if (nodes[k].second <= nodes[j].second)
                    ++insert_at;
                else
                    break;
            }
            for (size_t k = j; k > insert_at; --k)
                std::swap(nodes[k], nodes[k - 1]);
        }
        // Recursive calls
        for (size_t j = 0; j < intersections; ++j)
        {
            if (intersectSegmentRecursive(
                m_nodes[nodes[j].first],
                depth + 1,
                segment_origin,
                segment_end,
                intersection_point,
                intersection_normal))
            {
                return true;
            }
        }
    }
    return false;
}

bool ldplab::rtscpu::ParticleMeshOctree::intersectSegmentBase(
    const utils::Array<Triangle>& triangles, 
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    size_t min_index = triangles.size();
    double min_dist = std::numeric_limits<double>::max(), t_dist;
    for (size_t i = 0; i < triangles.size(); ++i)
    {
        if (IntersectionTest::lineTriangle(
            segment_origin, segment_end, triangles[i], t_dist))
        {
            if (t_dist < min_dist)
            {
                min_index = i;
                min_dist = t_dist;
            }
        }
    }
    if (min_index < triangles.size())
    {
        const Vec3 dir = segment_end - segment_origin;
        intersection_point = segment_origin + dir * min_dist;
        const Vec3 edge1 = triangles[min_index].b - triangles[min_index].a;
        const Vec3 edge2 = triangles[min_index].c - triangles[min_index].a;
        intersection_normal = glm::normalize(glm::cross(edge1, edge2));
        if (glm::dot(intersection_normal, dir) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }
    return false;
}

ldplab::rtscpu::ParticleMeshOctree::OctreeNode::OctreeNode()
    :
    num_children{ 0 },
    children{ 0 }
{
    aabb.min = Vec3(std::numeric_limits<double>::max());
    aabb.max = Vec3(std::numeric_limits<double>::lowest());
}
