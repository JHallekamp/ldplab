#ifndef WWU_LDPLAB_UTILS_ASSERT_HPPs
#define WWU_LDPLAB_UTILS_ASSERT_HPP

#include <chrono>
#include <map>
#include <mutex>
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

// Macros for in-library use
#ifndef LDPLAB_BUILD_OPTION_ENABLE_PROFILING
#   define LDPLAB_PROFILING_START(x)
#   define LDPLAB_PROFILING_PAUSE(x)
#   define LDPLAB_PROFILING_UNPAUSE(x)
#   define LDPLAB_PROFILING_STOP(x)
#else
#   define LDPLAB_PROFILING_START(x)    ldplab::ProfilingMeasurement x(#x)
#   define LDPLAB_PROFILING_PAUSE(x)    x.pause()
#   define LDPLAB_PROFILING_UNPAUSE(x)  x.unpause()
#   define LDPLAB_PROFILING_STOP(x)     x.stop()
namespace ldplab
{
    class Profiler
    {
    public:
        static void addMeasurement(const std::string& key, double time_secs);
        static void printReport(const std::string& file);
        static void reset();
    private:
        Profiler() { }
        static Profiler& instance();
    private:
        struct Result
        {
            Result();
            std::mutex mtx;
            double total_time;
            double max_time;
            double min_time;
            size_t call_counter;
        };
    private:
        std::map<std::string, Result> m_results;
    };

    class ProfilingMeasurement
    {
    public:
        ProfilingMeasurement() = delete;
        ProfilingMeasurement(const ProfilingMeasurement&) = delete;
        ProfilingMeasurement(ProfilingMeasurement&&) = delete;
        ProfilingMeasurement(std::string&& key);
        ~ProfilingMeasurement();
        void pause();
        void unpause();
        void stop();
    private:
        std::chrono::steady_clock::time_point m_start;
        double m_runtime;
        std::string m_key;
        enum class State { running, paused, stopped } m_state;
    };
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_PROFILING

#endif