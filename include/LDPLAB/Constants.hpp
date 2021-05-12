#ifndef WWU_LDPLAB_CONSTANTS_HPP
#define WWU_LDPLAB_CONSTANTS_HPP

#include <cstdint>

namespace ldplab
{
    namespace constant
    {
        namespace rod_particle
        {
            /** 
             * @brief The minimum kappa threshold before cap and indentation of 
             *        rod particles are assumed to be flat. 
             * @details Defining a rod particle with kappa less or equal to the
             *          minimum kappa threshold will create a perfect cylinder.
             */
            constexpr real_t min_kappa_threshold = 1e-3;
        }

        namespace intersection_tests
        {
#ifdef LDPLAB_BUILD_OPTION_USE_SINGLE_PRECISION
            /** @brief Defines an epsilon region for intersection tests. */
            constexpr real_t epsilon = 1e-7;
#else
            /** @brief Defines an epsilon region for intersection tests. */
            constexpr real_t epsilon = 1e-10;
#endif // LDPLAB_BUILD_OPTION_USE_SINGLE_PRECISION
        }
    }
}

#endif