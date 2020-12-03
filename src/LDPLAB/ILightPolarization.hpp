#ifndef WWU_LDPLAB_ILIGHT_POLARIZATION_HPP
#define WWU_LDPLAB_ILIGHT_POLARIZATION_HPP

namespace
{
    /**
     * @brief Generic interface class which defines representations of 
     *        polarization states of a light source. 
     */
    class ILightPolarisation
    {
    public:
        enum class Type { unpolarized };
    public:
        /**
         * @brief The destructor is virtual since classes inherit from
         *        ILightPolarisation.
         */
        virtual ~ILightPolarisation() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
    };
}

#endif