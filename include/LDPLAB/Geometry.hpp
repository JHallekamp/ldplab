#ifndef WWU_LDPLAB_GEOMETRY_HPP
#define WWU_LDPLAB_GEOMETRY_HPP

#include <LDPLAB/Datatypes.hpp>

namespace ldplab
{
    /** @brief Structure modeling light rays. */
    struct Ray
    {
        Vec3 origin;
        Vec3 direction;
        real_t intensity;
    };

    /**
     * @brief Structure modling a Triangle in three dimensional space.
     *        The corner points are given in counter clockwise order and hence 
     *        given a orientation in space.
     */
    struct Triangle
    {
        Vec3 a;
        Vec3 b;
        Vec3 c;
        /** @brief The triangle surface normal pointing to the outside. */
        Vec3 outside_normal;
    };

    /** @brief Structure modeling an axis aligned bounding box. */
    struct AABB
    {
        Vec3 min;
        Vec3 max;
    };
    
    /** @brief Defines the order of orientation around each axis. */
    enum class RotationOrder
    {
        /** @brief Rotates first around x, then y and finally z. */
        xyz,
        /** @brief Rotates first around x, then z and finally y. */
        xzy,
        /** @brief Rotates first around y, then x and finally z. */
        yxz,
        /** @brief Rotates first around y, then z and finally x. */
        yzx,
        /** @brief Rotates first around z, then x and finally y. */
        zxy,
        /** @brief Rotates first around z, then y and finally x. */
        zyx
    };

    constexpr RotationOrder invertRotationOrder(RotationOrder order)
    {
        switch (order)
        {
        case ldplab::RotationOrder::xyz: return RotationOrder::zyx;
        case ldplab::RotationOrder::xzy: return RotationOrder::yzx;
        case ldplab::RotationOrder::yxz: return RotationOrder::zxy;
        case ldplab::RotationOrder::yzx: return RotationOrder::xzy;
        case ldplab::RotationOrder::zxy: return RotationOrder::yxz;
        case ldplab::RotationOrder::zyx: return RotationOrder::xyz;
        default:
            return order;
        }
    };
}

#endif