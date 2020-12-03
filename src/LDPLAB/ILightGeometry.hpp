#ifndef WWU_LDPLAB_ILIGHT_GEOMETRY_HPP
#define WWU_LDPLAB_ILIGHT_GEOMETRY_HPP

namespace
{
    /**
     * @brief Generic interface class which defines the shape of a light source.
     */
    class ILightGeometry
    {
    public:
        enum class Type { square };
    public:
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