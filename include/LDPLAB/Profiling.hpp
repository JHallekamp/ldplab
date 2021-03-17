#ifndef WWU_LDPLAB_PROFILING_HPPs
#define WWU_LDPLAB_PROFILING_HPP

#include <string>

namespace ldplab
{
    class Profiling
    {
    public:
        /**
         * @brief Prints the detailed profiling reports to the given file.
         * @param[in] file Path to the output file.
         * @note This method will do nothing, if the library has not been build
         *       with the LDPLAB_BUILD_OPTION_ENABLE_PROFILING macro defined.
         */
        static void printReports(const std::string& file);
        /**
         * @brief Resets the profiling data.
         * @note This method will do nothing, if the library has not been build
         *       with the LDPLAB_BUILD_OPTION_ENABLE_PROFILING macro defined.
         */
        static void reset();
    };
}

#endif