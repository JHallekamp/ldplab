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
        /** @brief Queries whether logging is enabled in the build. */
        static bool loggingEnabled();
        /** @brief Queries whether debug logging is enabled in the build. */
        static bool debugLoggingEnabled();
        /** @brief Queries whether debug asserts are enabled in the build. */
        static bool debugAssertsEnabled();
        /** @brief Queries whether profiling is enabled in the build. */
        static bool profilingEnabled();
        /** @brief Queries if the build supports RTS on GPU using OpenGL. */
        static bool OpenGLRayTracingEnabled();
        /** @brief Queries if the build supports CUDA ray tracing steps. */
        static bool CUDARayTracingEnabled();
    };
}

#endif