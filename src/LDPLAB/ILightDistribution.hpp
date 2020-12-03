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
        private:
            enum class Type { homogenous } type;
        public:
        /**
         * @brief The destructor is virtual since classes inherit from
         *        ILightDirection.
         */
        virtual ~ILightDistribution() { }
        Type getType() { return type; }
    };
}

#endif
