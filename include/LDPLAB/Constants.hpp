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
            constexpr double min_kappa_threshold = 1e-3;
        }

        namespace intersection_tests
        {
            /** @brief Defines an epsilon region for intersection tests. */
            constexpr double epsilon = 1e-9;
        }
    }
}

#endif