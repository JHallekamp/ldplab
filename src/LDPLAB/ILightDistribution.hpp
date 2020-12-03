#ifndef WWU_LDPLAB_ILIGHT_DISTRIBUTION_HPP
#define WWU_LDPLAB_ILIGHT_DISTRIBUTION_HPP

namespace
{
    /**
     * @brief Generic interface class which defines the intensity distribution
     *        of a light source.
     */
    class ILightDistribution
    {
        public:
            enum class Type { homogenous };
        public:
        /**
         * @brief The destructor is virtual since classes inherit from
         *        ILightDirection.
         */
        virtual ~ILightDistribution() { }
        /** @brief Returns the type of the instance. */
        virtual Type type() const = 0;
    };
}

#endif
