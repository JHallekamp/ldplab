#ifndef WWU_LDPLAB_ILIGHT_POLARIZATION_HPP
#define WWU_LDPLAB_ILIGHT_POLARIZATION_HPP

namespace ldplab
{
    /**
     * @brief Generic interface class which defines representations of 
     *        polarization states of a light source. 
     */
    struct ILightPolarisation
    {
        enum class Type { unpolarized };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        ILightPolarisation.
         */
        virtual ~ILightPolarisation() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const char* typeString() const
        {
            switch (type())
            {
            case Type::unpolarized: return "unpolarized";
            default: return "unknown_type";
            }
        }
    };

    /**
     * @brief Unpolarized light.
     */
    struct LightPolarisationUnpolarized : public ILightPolarisation
    {
        Type type() const override { return Type::unpolarized; }
    };
}

#endif