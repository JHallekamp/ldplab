#ifndef WWU_LDPLAB_BUILD_INFO_HPP
#define WWU_LDPLAB_BUILD_INFO_HPP

namespace ldplab
{
    /**
     * @brief Contains static methods which can be used to query information
     *        about the library build.
     */
    struct BuildInfo
    {
        /** @brief No instances allowed. */
        BuildInfo() = delete;
        /** @brief Querys whether logging is enabled in the build. */
        static bool loggingEnabled();
        /** @brief Querys whether debug logging is enabled in the build. */
        static bool debugLoggingEnabled();
        /** @brief Querys whether debug asserts are enabled in the build. */
        static bool debugAssertsEnabled();
        /** @brief Querys whether profiling is enabled in the build. */
        static bool profilingEnabled();
    };
}

#endif