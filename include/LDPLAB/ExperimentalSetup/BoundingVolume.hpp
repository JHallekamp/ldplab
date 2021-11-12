#ifndef WWU_LDPLAB_BOUNDING_VOLUME_HPP
#define WWU_LDPLAB_BOUNDING_VOLUME_HPP

#include "../Geometry.hpp"
#include <string>

namespace ldplab
{
    struct IBoundingVolume
    {
        enum class Type { sphere, box };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        IParticleGeometry.
         */
        virtual ~IBoundingVolume() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const std::string typeString() const
        {
            switch (type())
            {
            case Type::sphere: return "sphere";
            case Type::box: return "box";
            default: return "unknown_type";
            }
        }
    };

    /** @brief Contains parameters of bounding sphere. */
    struct BoundingVolumeSphere : public IBoundingVolume
    {
        BoundingVolumeSphere(Vec3 center, double radius)
            :
            center{ center },
            radius{ radius }
        {}
        /** 
         * @brief Returns the type of the instance.
         * @returns IBoundingVolume::Type::sphere
         */
        Type type() const override { return Type::sphere; }
        /** @brief Radius of the sphere in world space. */
        double radius;
        /** @brief Center of the sphere in particle space. */
        Vec3 center;
    };

    /** 
     * @brief Contains bounding box data. 
     * @details The bounding box is aligned along the axes.
     */
    struct BoundingVolumeBox : public IBoundingVolume
    {
        BoundingVolumeBox(Vec3 min, Vec3 max)
            :
            min{ min },
            max{ max }
        {}
        /**
         * @brief Returns the type of the instance.
         * @returns IBoundingVolume::Type::box
         */
        Type type() const override { return Type::box; }
        /** @brief Contains the minimum corner coordinates in each dimension. */
        Vec3 min;
        /** @brief Contains the maximum corner coordinates in each dimension. */
        Vec3 max;
    };
}

#endif