#ifndef WWU_LDPLAB_ILIGHT_GEOMETRY_HPP
#define WWU_LDPLAB_ILIGHT_GEOMETRY_HPP

namespace
{
    /**
     * @brief Generic interface class which defines the shape of a light source.
     */
    struct ILightGeometry
    {
        enum class Type { square };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        ILightGeometry.
         */
        virtual ~ILightGeometry() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
    };
}

#endif