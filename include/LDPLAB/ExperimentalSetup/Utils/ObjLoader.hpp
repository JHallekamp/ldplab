#ifndef WWU_LDPLAB_OBJ_LOADER_HPP
#define WWU_LDPLAB_OBJ_LOADER_HPP

#include <LDPLAB/Geometry.hpp>

#include <string>
#include <vector>

namespace ldplab
{
    class ObjLoader
    {
        bool loadTriangleMesh(
            const std::string& filepath,
            std::vector<Triangle>& loaded_mesh);
    };
}

#endif