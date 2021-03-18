#include "Profiler.hpp"

#include <fstream>
#include <limits>

// ============================================================================
// Implementing the profiler
#ifdef LDPLAB_BUILD_OPTION_ENABLE_PROFILING
void ldplab::utils::Profiler::addMeasurement(
    const std::string& key, double time_secs)
{
    Profiler& me = instance();
    Result& result = me.m_results[key];
    std::lock_guard<std::mutex> lck{ result.mtx };
    result.call_counter++;
    result.total_time += time_secs;
    if (result.max_time < time_secs)
        result.max_time = time_secs;
    if (result.min_time > time_secs)
        result.min_time = time_secs;
}

void ldplab::utils::Profiler::printReport(const std::string& file)
{
    Profiler& me = instance();
    std::ofstream out(file);
    for (std::map<std::string, Result>::iterator it = me.m_results.begin();
        it != me.m_results.end();
        ++it)
    {
        Result& result = it->second;
        std::lock_guard<std::mutex> lck{ result.mtx };
        out << "Profiling result: " << it->first << std::endl;
        out << "  call count..: " << result.call_counter << std::endl;
        out << "  total time..: " << result.total_time << "s" << std::endl; 
        if (result.call_counter > 0)
        {
            double avg_time = result.total_time / 
                static_cast<double>(result.call_counter);
            out << "  avg time....: " << avg_time << "s" << std::endl;
            out << "  min time....: " << result.min_time << "s" << std::endl;
            out << "  max time....: " << result.max_time << "s" << std::endl;
        }
        out << std::endl;
    }
}

void ldplab::utils::Profiler::reset()
{
    Profiler& me = instance();
    for (std::map<std::string, Result>::iterator it = me.m_results.begin();
        it != me.m_results.end();
        ++it)
    {
        Result& result = it->second;
        std::lock_guard<std::mutex> lck{ result.mtx };
        result.call_counter = 0;
        result.total_time = 0;
        result.max_time = std::numeric_limits<double>::min();
        result.min_time = std::numeric_limits<double>::max();
    }
}

ldplab::utils::Profiler& ldplab::utils::Profiler::instance()
{
    static Profiler profiler;
    return profiler;
}

ldplab::utils::Profiler::Result::Result()
    :
    call_counter{ 0 },
    total_time{ 0 },
    max_time{ std::numeric_limits<double>::min() },
    min_time{ std::numeric_limits<double>::max() }
{ }

ldplab::utils::ProfilingMeasurement::ProfilingMeasurement(std::string&& key)
    :
    m_key { key },
    m_runtime{ 0 },
    m_state{ State::running }
{
    m_start = std::chrono::steady_clock::now();
}

ldplab::utils::ProfilingMeasurement::~ProfilingMeasurement()
{
    stop();
}

void ldplab::utils::ProfilingMeasurement::pause()
{
    if (m_state == State::running)
    {
        const std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now();
        m_runtime += std::chrono::duration<double>(now - m_start).count();
        m_state = State::paused;
    }
}

void ldplab::utils::ProfilingMeasurement::unpause()
{
    if (m_state == State::paused)
    {
        m_state = State::running;
        m_start = std::chrono::steady_clock::now();
    }
}

void ldplab::utils::ProfilingMeasurement::stop()
{
    if (m_state != State::stopped)
    {
        if (m_state == State::running)
        {
            const std::chrono::steady_clock::time_point now =
                std::chrono::steady_clock::now();
            m_runtime += std::chrono::duration<double>(now - m_start).count();
        }
        Profiler::addMeasurement(m_key, m_runtime);
        m_state = State::stopped;
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_PROFILING