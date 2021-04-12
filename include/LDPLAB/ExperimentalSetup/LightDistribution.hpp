#ifndef WWU_LDPLAB_ILIGHT_DISTRIBUTION_HPP
#define WWU_LDPLAB_ILIGHT_DISTRIBUTION_HPP

namespace ldplab
{
    /**
     * @brief Generic interface class which defines the intensity distribution
     *        of a light source.
     */
    struct ILightDistribution
    {
        enum class Type { homogeneous };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        ILightDirection.
         */
        virtual ~ILightDistribution() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
        /** @brief Returns the type of the instance as string. */
        const char* typeString() const
        {
            switch (type())
            {
            case Type::homogeneous: return "homogeneous";
            default: return "unknown_type";
            }
        }
    };

    /**
     * @brief Homogeneous light distribution over a complete light source.
     */
    struct LightDistributionHomogeneous : public ILightDistribution
    {
        LightDistributionHomogeneous(double intensity)
            :
            intensity{ intensity }
        {}
        Type type() const override { return Type::homogeneous; }
        /** @brief Light intensity emitted by the light source. */
        double intensity;
    };
}

#endif
