#ifndef WWU_LDPLAB_ILIGHT_DISTRIBUTION_HPP
#define WWU_LDPLAB_ILIGHT_DISTRIBUTION_HPP

namespace
{
    /**
     * @brief Generic interface class which defines the intensity distribution
     *        of a light source.
     */
    struct ILightDistribution
    {
        enum class Type { homogenous };
        /**
         * @brief The destructor is virtual since classes inherit from
         *        ILightDirection.
         */
        virtual ~ILightDistribution() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
    };

    /**
     * @brief Homogenous light distribution over a complete light source.
     */
    struct LightDistributionHomogenous : public ILightDistribution
    {
        Type type() const override { return Type::homogenous; }
        /** @brief Light intensity emited by the light source. */
        double intensity;
    };
}

#endif
