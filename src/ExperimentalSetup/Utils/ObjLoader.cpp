#include <LDPLAB/ExperimentalSetup/Utils/ObjLoader.hpp>

// Implement tiny obj loader functions for use
#ifndef TINYOBJLOADER_USE_DOUBLE
#define TINYOBJLOADER_USE_DOUBLE
#endif
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include "../../Utils/Log.hpp"

bool ldplab::ObjLoader::loadTriangleMesh(
    const std::string& filepath, 
    std::vector<Triangle>& loaded_mesh)
{
    AABB temp;
    return loadTriangleMesh(filepath, loaded_mesh, temp);
}

bool ldplab::ObjLoader::loadTriangleMesh(
    const std::string& filepath, 
    std::vector<Triangle>& loaded_mesh, 
    AABB& mesh_aabb)
{
    LDPLAB_LOG_INFO("ObjLoader: Begin to load triangle mesh from \"%s\"",
        filepath.c_str());

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn, err;
    bool ret = tinyobj::LoadObj(
        &attrib,
        &shapes,
        &materials,
        &warn,
        &err,
        filepath.c_str(),
        "./");
    if (!warn.empty())
    {
        if (warn.back() == '\n')
            warn.pop_back();
        LDPLAB_LOG_WARNING(
            "ObjLoader: tinyobjloader warning when loading \"%s\": %s",
            filepath.c_str(),
            warn.c_str());
    }
    if (!err.empty())
    {
        if (err.back() == '\n')
            err.pop_back();
        LDPLAB_LOG_ERROR(
            "ObjLoader: tinyobjloader error when loading \"%s\": %s",
            filepath.c_str(),
            err.c_str());
    }
    if (!ret)
    {
        LDPLAB_LOG_ERROR("ObjLoader: Could not load traingle mesh from \"%s\"",
            filepath.c_str());
        return false;
    }

    loaded_mesh.clear();
    Vec3& min = mesh_aabb.min;
    Vec3& max = mesh_aabb.max;
    min = Vec3(
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max());
    max = Vec3(
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest());
    for (size_t f = 0; f < shapes[0].mesh.indices.size() / 3; ++f)
    {
        tinyobj::index_t idx0 = shapes[0].mesh.indices[3 * f + 0];
        tinyobj::index_t idx1 = shapes[0].mesh.indices[3 * f + 1];
        tinyobj::index_t idx2 = shapes[0].mesh.indices[3 * f + 2];
        loaded_mesh.emplace_back();
        ldplab::Triangle& t = loaded_mesh.back();
        t.a[0] = attrib.vertices[3 * idx0.vertex_index + 0];
        t.a[1] = attrib.vertices[3 * idx0.vertex_index + 1];
        t.a[2] = attrib.vertices[3 * idx0.vertex_index + 2];
        t.b[0] = attrib.vertices[3 * idx1.vertex_index + 0];
        t.b[1] = attrib.vertices[3 * idx1.vertex_index + 1];
        t.b[2] = attrib.vertices[3 * idx1.vertex_index + 2];
        t.c[0] = attrib.vertices[3 * idx2.vertex_index + 0];
        t.c[1] = attrib.vertices[3 * idx2.vertex_index + 1];
        t.c[2] = attrib.vertices[3 * idx2.vertex_index + 2];
        min.x = std::min(min.x, std::min(t.a.x, std::min(t.b.x, t.c.x)));
        max.x = std::max(max.x, std::max(t.a.x, std::max(t.b.x, t.c.x)));
        min.y = std::min(min.y, std::min(t.a.y, std::min(t.b.y, t.c.y)));
        max.y = std::max(max.y, std::max(t.a.y, std::max(t.b.y, t.c.y)));
        min.z = std::min(min.z, std::min(t.a.z, std::min(t.b.z, t.c.z)));
        max.z = std::max(max.z, std::max(t.a.z, std::max(t.b.z, t.c.z)));
    }

    LDPLAB_LOG_INFO("ObjLoader: Finished loading triangle mesh from \"%s\", " \
        "loaded %i triangles",
        filepath.c_str(),
        loaded_mesh.size());
    return true;
}
