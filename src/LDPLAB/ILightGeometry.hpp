#ifndef WWU_LDPLAB_ILIGHT_GEOMETRY_HPP
#define WWU_LDPLAB_ILIGHT_GEOMETRY_HPP

namespace
{
    /**
     * @brief Generic interface class which defines the shape of a light source.
     */
    class ILightGeometry
    {
    private:
        enum class Type { square } type;
    public:
        /**
         * @brief The destructor is virtual since classes inherit from
         *        ILightGeometry.
         */
        virtual ~ILightGeometry() { }
        Type getType() { return type; }
    };
}

#endif