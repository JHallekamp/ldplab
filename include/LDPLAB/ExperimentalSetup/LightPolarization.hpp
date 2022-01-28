#ifndef WWU_LDPLAB_ILIGHT_POLARIZATION_HPP
#define WWU_LDPLAB_ILIGHT_POLARIZATION_HPP

#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    /**
     * @brief Generic interface class which defines representations of 
     *        polarization states of a light source. 
     */
    struct ILightPolarisation
    {
        enum class Type { unpolarized, polarized };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        ILightPolarisation.
         */
        virtual ~ILightPolarisation() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const std::string typeString() const
        {
            switch (type())
            {
            case Type::unpolarized: return "unpolarized";
            case Type::polarized: return "polarized";
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
    /**
     * @brief Polarized light described with the Stokes parameter.
     */
    struct LightPolarisationPolarized : public ILightPolarisation
    {
        LightPolarisationPolarized(Vec4 stokes_parameter)
            : stokes_parameter{ stokes_parameter }
        {
          
        }
        Type type() const override { return Type::polarized; }
        Vec4 stokes_parameter;
    };

}

#endif