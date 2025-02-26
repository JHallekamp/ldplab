#include <LDPLAB/BuildInfo.hpp>

bool ldplab::BuildInfo::loggingEnabled()
{
#ifdef LDPLAB_BUILD_OPTION_DISABLE_LOGGING
    return false;
#else
    return true;
#endif 
}

bool ldplab::BuildInfo::debugLoggingEnabled()
{
#ifdef LDPLAB_BUILD_OPTION_DISABLE_LOGGING
    return false;
#else
#   ifdef LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING
    return true;
#   else
    return false;
#   endif
#endif
}

bool ldplab::BuildInfo::debugAssertsEnabled()
{
#ifdef LDPLAB_BUILD_OPTION_ENABLE_ASSERT
    return true;
#else
    return false;
#endif
}

bool ldplab::BuildInfo::profilingEnabled()
{
#ifdef LDPLAB_BUILD_OPTION_ENABLE_PROFILING
    return true;
#else
    return false;
#endif
}

bool ldplab::BuildInfo::OpenGLRayTracingEnabled()
{
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSOGL
    return true;
#else
    return false;
#endif
}

bool ldplab::BuildInfo::CudaRayTracingEnabled()
{
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
    return true;
#else
    return false;
#endif
}
