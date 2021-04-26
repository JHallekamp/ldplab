#include "AcceleratorStructures.hpp"

#include "IntersectionTests.hpp"

#include <array>
#include <limits>
#include <utility>

bool ldplab::rtscpu::TriangleMeshGeometryList::intersectRay(
    const Ray& ray,
    Vec3& intersection_point,
    Vec3& intersection_normal,
    double& dist)
{
    size_t min_index = m_mesh.size();
    double t_dist;
    dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < m_mesh.size(); ++i)
    {
        if (IntersectionTest::intersectRayTriangle(ray, m_mesh[i], t_dist))
        {
            if (t_dist < dist)
            {
                min_index = i;
                dist = t_dist;
            }
        }
    }
    if (min_index < m_mesh.size())
    {
        intersection_point = ray.origin + dist * ray.direction;
        const Vec3 edge1 = m_mesh[min_index].b - m_mesh[min_index].a;
        const Vec3 edge2 = m_mesh[min_index].c - m_mesh[min_index].a;
        intersection_normal = glm::normalize(glm::cross(edge1, edge2));
        if (glm::dot(intersection_normal, ray.direction) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }
    return false;
}

bool ldplab::rtscpu::TriangleMeshGeometryList::constructInternal(
    const std::vector<Triangle>& mesh, 
    const IAcceleratorStructureParameter* parameter)
{
    m_mesh = std::vector<Triangle>(mesh);
    return true;
}

bool ldplab::rtscpu::TriangleMeshGeometryOctree::intersectRay(
    const Ray& ray, 
    Vec3& intersection_point, 
    Vec3& intersection_normal,
    double& dist)
{
    if (IntersectionTest::overlapRayAABB(ray, m_nodes[0].aabb, dist))
    {
        return intersectRecursive(
            m_nodes[0], 
            0, 
            ray, 
            intersection_point, 
            intersection_normal);
    }
    return false;
}

bool ldplab::rtscpu::TriangleMeshGeometryOctree::constructInternal(
    const std::vector<Triangle>& mesh, 
    const IAcceleratorStructureParameter* parameter)
{
    construct(
        mesh,
        ((AcceleratorStructureOctreeParameter*)parameter)->octree_depth);
    return true;
}

size_t ldplab::rtscpu::TriangleMeshGeometryOctree::pow8(size_t exp) const noexcept
{
    constexpr size_t base_bit = 1;
    constexpr size_t log2_8 = 3; // = log_2(8)
    return base_bit << (exp * log2_8);
}

size_t ldplab::rtscpu::TriangleMeshGeometryOctree::mapIndexP2C(
    size_t parent_idx,
    size_t child_no) const noexcept
{
    return parent_idx * 8 + child_no;
}

inline size_t ldplab::rtscpu::TriangleMeshGeometryOctree::mapIndexC2P(
    size_t child_idx) const noexcept
{
    // child_idx == parent_idx * 8 + child_no
    // => parent_index == (child_idx - child_no) / 8
    // Because
    // (i) child_no < 8 
    // and
    // (ii) child_no == 0 <=> child_idx % 8 == 0
    // the integer operation child_idx / 8 is sufficient
    return child_idx / 8;
}

inline size_t ldplab::rtscpu::TriangleMeshGeometryOctree::mapIndexGetChildNo(
    size_t child_idx) const noexcept
{
    // child_idx == parent_idx * 8 + child_no
    // => child_no == child_idx - parent_idx * 8
    // <=> child_no == child_idx - floor(child_idx / 8) * 8
    // => child_no == child_idx % 8
    return child_idx % 8;
}

void ldplab::rtscpu::TriangleMeshGeometryOctree::construct(
    const std::vector<Triangle>& mesh, 
    size_t octree_depth)
{
    // Check for empty mesh
    if (mesh.size() == 0)
    {
        m_nodes.emplace_back();
        m_nodes[0].aabb = AABB{ Vec3(0), Vec3(0) };
        m_nodes[0].num_children = 0;
        m_octree_depth = 0;
        return;
    }

    // Construct temporary octree layers
    m_octree_depth = octree_depth;    
    const AABB octree_aabb = constructOctreeAABB(mesh);
    
    std::vector<std::vector<OctreeNode>> layers;
    constructConstructionLayers(octree_aabb, layers);

    // Create temporary triangle storage
    std::vector<std::vector<Triangle>> triangle_storage(layers.back().size());
    for (size_t i = 0; i < layers.back().size(); ++i)
        layers.back()[i].children[0] = i;

    // Sort in triangles
    for (size_t i = 0; i < mesh.size(); ++i)
        constructSortTrianglesRecursive(mesh[i], 0, 0, layers, triangle_storage);

    // Move valuable information from temporary to octree storage and adjust
    // indices accordingly
    constructMakePersistentRecursive(0, 0, layers, triangle_storage);
}

ldplab::AABB ldplab::rtscpu::TriangleMeshGeometryOctree::constructOctreeAABB(
    const std::vector<Triangle>& mesh)
{
    AABB octree_aabb;
    octree_aabb.min = Vec3(
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max());
    octree_aabb.max = Vec3(
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest());

    for (size_t i = 0; i < mesh.size(); ++i)
    {
        octree_aabb.min.x = std::min(octree_aabb.min.x,
            std::min(mesh[i].a.x, std::min(mesh[i].b.x, mesh[i].c.x)));
        octree_aabb.max.x = std::max(octree_aabb.max.x,
            std::max(mesh[i].a.x, std::max(mesh[i].b.x, mesh[i].c.x)));

        octree_aabb.min.y = std::min(octree_aabb.min.y,
            std::min(mesh[i].a.y, std::min(mesh[i].b.y, mesh[i].c.y)));
        octree_aabb.max.y = std::max(octree_aabb.max.y,
            std::max(mesh[i].a.y, std::max(mesh[i].b.y, mesh[i].c.y)));

        octree_aabb.min.z = std::min(octree_aabb.min.z,
            std::min(mesh[i].a.z, std::min(mesh[i].b.z, mesh[i].c.z)));
        octree_aabb.max.z = std::max(octree_aabb.max.z,
            std::max(mesh[i].a.z, std::max(mesh[i].b.z, mesh[i].c.z)));
    }

    return octree_aabb;
}

void ldplab::rtscpu::TriangleMeshGeometryOctree::constructConstructionLayers(
    const AABB& octree_aabb,
    std::vector<std::vector<OctreeNode>>& layers)
{
    layers.resize(m_octree_depth + 1);
    for (size_t i = 0; i <= m_octree_depth; ++i)
    {
        const size_t num_layer_nodes = pow8(i);
        layers[i].resize(num_layer_nodes);
        if (i == 0)
        {
            // Construct root node
            layers[0][0].aabb = octree_aabb;
        }
        else
        {
            // Construct layer nodes
            for (size_t j = 0; j < num_layer_nodes; ++j)
            {
                OctreeNode& parent = layers[i - 1][mapIndexC2P(j)];
                // Set index in parents child array
                const size_t child_no = mapIndexGetChildNo(j);
                parent.children[child_no] = j;
                // Set AABB
                const Vec3 half_size = (parent.aabb.max - parent.aabb.min) * 0.5;
                const double b0 = (child_no & 1) ? 1.0 : 0.0;
                const double b1 = (child_no & 2) ? 1.0 : 0.0;
                const double b2 = (child_no & 4) ? 1.0 : 0.0;
                layers[i][j].aabb.min = Vec3(
                    parent.aabb.min.x + b0 * half_size.x,
                    parent.aabb.min.y + b1 * half_size.y, 
                    parent.aabb.min.z + b2 * half_size.z);
                layers[i][j].aabb.max = layers[i][j].aabb.min + half_size;
            }
        }
    }
}

bool ldplab::rtscpu::TriangleMeshGeometryOctree::constructSortTrianglesRecursive(
    const Triangle& triangle, 
    size_t current_layer, 
    size_t current_node, 
    std::vector<std::vector<OctreeNode>>& layers, 
    std::vector<std::vector<Triangle>>& triangle_storage)
{
    if (current_layer == m_octree_depth)
    {
        // Recursion base!
        OctreeNode& node = layers[current_layer][current_node];
        if (IntersectionTest::overlapTriangleAABB(triangle, node.aabb))
        {
            triangle_storage[node.children[0]].push_back(triangle);
            ++node.num_children;
            return true;
        }
        return false;
    }
    else
    {
        // Recursion!
        OctreeNode& node = layers[current_layer][current_node];
        if (!IntersectionTest::overlapTriangleAABB(triangle, node.aabb))
            return false;
        // Call recursive for children
        bool children_intersect = false;
        for (size_t i = 0; i < 8; ++i)
        {
            if (constructSortTrianglesRecursive(
                triangle,
                current_layer + 1,
                node.children[i],
                layers,
                triangle_storage))
            {
                children_intersect = true;
            }
        }
        if (children_intersect)
            ++node.num_children;
        return children_intersect;
    }
}

size_t ldplab::rtscpu::TriangleMeshGeometryOctree::constructMakePersistentRecursive(
    const size_t current_layer, 
    const size_t current_node, 
    const std::vector<std::vector<OctreeNode>>& layers, 
    const std::vector<std::vector<Triangle>>& triangle_storage)
{
    const OctreeNode& layer_node = layers[current_layer][current_node];
    const size_t node_idx = m_nodes.size();
    m_nodes.push_back(layer_node);
    if (current_layer == m_octree_depth)
    {
        if (layer_node.num_children)
        {
            m_nodes[node_idx].num_children = 1;
            m_nodes[node_idx].children[0] = m_triangle_arrays.size();
            m_triangle_arrays.emplace_back(
                triangle_storage[layer_node.children[0]]);
        }
    }
    else
    {
        size_t num_children = 0;
        for (size_t i = 0; i < 8; ++i)
        {
            if (layers[current_layer + 1][layer_node.children[i]].num_children)
            {
                size_t child_idx = constructMakePersistentRecursive(
                    current_layer + 1,
                    layer_node.children[i],
                    layers,
                    triangle_storage);
                m_nodes[node_idx].children[num_children++] = child_idx;
            }
        }

        m_nodes[node_idx].num_children = num_children;
    }
    return node_idx;
}

bool ldplab::rtscpu::TriangleMeshGeometryOctree::intersectRecursive(
    const OctreeNode& node, 
    const size_t depth,
    const Ray& ray, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    if (depth == m_octree_depth)
    {
        if (node.num_children == 0)
            return false;
        return intersectBase(
            m_triangle_arrays[node.children[0]],
            ray,
            intersection_point,
            intersection_normal);
    }
    else
    {
        std::array<std::pair<size_t, double>, 8> nodes;
        double dist;
        size_t intersections = 0;
        // Check for intersections with the subdivisions
        for (size_t i = 0; i < node.num_children; ++i)
        {
            const size_t c = node.children[i];
            if (IntersectionTest::overlapRayAABB(ray, m_nodes[c].aabb, dist))
                nodes[intersections++] = std::pair<size_t, double>(c, dist);
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

bool ldplab::rtscpu::TriangleMeshGeometryOctree::intersectBase(
    const utils::Array<Triangle>& triangles, 
    const Ray& ray,
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    size_t min_index = triangles.size();
    double min_dist = std::numeric_limits<double>::max(), t_dist;
    for (size_t i = 0; i < triangles.size(); ++i)
    {
        if (IntersectionTest::intersectRayTriangle(ray, triangles[i], t_dist))
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

ldplab::rtscpu::TriangleMeshGeometryOctree::OctreeNode::OctreeNode()
    :
    num_children{ 0 },
    children{ 0 }
{
    aabb.min = Vec3(std::numeric_limits<double>::max());
    aabb.max = Vec3(std::numeric_limits<double>::lowest());
}
