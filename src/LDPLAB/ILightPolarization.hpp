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
    private:
        enum class Type { unpolarized } type;
    public:
        /**
         * @brief The destructor is virtual since classes inherit from
         *        ILightPolarisation.
         */
        virtual ~ILightPolarisation() { }
        Type getType() { return type; }
    };
}

#endif