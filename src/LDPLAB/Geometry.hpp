#ifndef WWU_LDPLAB_GEOMETRY_HPP
#define WWU_LDPLAB_GEOMETRY_HPP

#include "..\..\libs\glm\glm.hpp"

namespace ldplab
{
    /**
     * @brief Structure modeling a 3D vertex.
     */
    typedef glm::dvec2 Vec2;
    typedef glm::dvec3 Vec3;

    /**
     * @brief Structure modeling a axially parallel bounding box.
     */
    struct BoundingBox
    {
        Vec3 min;
        Vec3 max;
    };

    /**
     * @brief Structure modeling a bounding sphere.
     */
    struct BoundingSphere
    {
        Vec3 origin;
        double radius;
    };

    /**
     * @brief Structure modeling light rays.
     */
    struct Ray
    {
        Vec3 origin;
        Vec3 direction;
        double intensity;
    };
}

#endif