#ifndef WWU_LDPLAB_UTILS_COMPUTE_HELPER_HPP
#define WWU_LDPLAB_UTILS_COMPUTE_HELPER_HPP

#include <cstdint>

namespace ldplab
{
    namespace utils
    {
        /**
         * @brief Contains static helper methods used during GPU computation or
         *        GPU setup.
         */
        class ComputeHelper
        {
        public:
            /** @brief There is no need to create instances of this class. */
            ComputeHelper() = delete;
            /**
             * @brief Calculates the number of work groups based on total
             *        number of threads and local size (in a given dimension 
             *        each).
             * @param[in] num_threads Total number of threads.
             * @param[in] local_size Local size of a work group.
             */
            static size_t getNumWorkGroups(
                size_t num_threads, size_t local_size);
        };
    }
}

#endif