#ifndef WWU_LDPLAB_OBJ_LOADER_HPP
#define WWU_LDPLAB_OBJ_LOADER_HPP

#include <LDPLAB/Geometry.hpp>

#include <string>
#include <vector>

namespace ldplab
{
    class ObjLoader
    {
    public:
        static bool loadTriangleMesh(
            const std::string& filepath,
            std::vector<Triangle>& loaded_mesh);
        static bool loadTriangleMesh(
            const std::string& filepath,
            std::vector<Triangle>& loaded_mesh,
            AABB& mesh_aabb);
    };
}

#endif