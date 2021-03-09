#include "Profiling.hpp"
#include "Utils/Profiler.hpp"

void ldplab::Profiling::printReports(const std::string& file)
{
#ifdef LDPLAB_BUILD_OPTION_ENABLE_PROFILING
    Profiler::printReport(file);
#endif // LDPLAB_BUILD_OPTION_ENABLE_PROFILING
}

void ldplab::Profiling::reset()
{
#ifdef LDPLAB_BUILD_OPTION_ENABLE_PROFILING
    Profiler::reset();
#endif // LDPLAB_BUILD_OPTION_ENABLE_PROFILING
}