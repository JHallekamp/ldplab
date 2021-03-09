#include "Log.hpp"

#include <stdio.h>
#include <stdarg.h>

constexpr unsigned int MAX_LOG_ENTRY_STRING = 512;

void ldplab::utils::Log::logFatal(const char* format, ...)
{
#ifndef LDPLAB_BUILD_OPTION_DISABLE_LOGGING

    if (getLogDispatcher().numCallbacks() == 0)
        return;

    char buffer[MAX_LOG_ENTRY_STRING];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, MAX_LOG_ENTRY_STRING, format, args);
    const std::string log_entry = "[FATAL] " + std::string(buffer);
    getLogDispatcher().log(LOG_LEVEL_FATAL, log_entry);
    va_end(args);

#endif // LDPLAB_BUILD_OPTION_DISABLE_LOGGING
}

void ldplab::utils::Log::logError(const char* format, ...)
{
#ifndef LDPLAB_BUILD_OPTION_DISABLE_LOGGING

    if (getLogDispatcher().numCallbacks() == 0)
        return;

    char buffer[MAX_LOG_ENTRY_STRING];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, MAX_LOG_ENTRY_STRING, format, args);
    const std::string log_entry = "[ERROR] " + std::string(buffer);
    getLogDispatcher().log(LOG_LEVEL_ERROR, log_entry);
    va_end(args);

#endif // LDPLAB_BUILD_OPTION_DISABLE_LOGGING
}

void ldplab::utils::Log::logWarning(const char* format, ...)
{
#ifndef LDPLAB_BUILD_OPTION_DISABLE_LOGGING

    if (getLogDispatcher().numCallbacks() == 0)
        return;

    char buffer[MAX_LOG_ENTRY_STRING];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, MAX_LOG_ENTRY_STRING, format, args);
    const std::string log_entry = "[WARNING] " + std::string(buffer);
    getLogDispatcher().log(LOG_LEVEL_WARNING, log_entry);
    va_end(args);

#endif // LDPLAB_BUILD_OPTION_DISABLE_LOGGING
}

void ldplab::utils::Log::logInfo(const char* format, ...)
{
#ifndef LDPLAB_BUILD_OPTION_DISABLE_LOGGING

    if (getLogDispatcher().numCallbacks() == 0)
        return;

    char buffer[MAX_LOG_ENTRY_STRING];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, MAX_LOG_ENTRY_STRING, format, args);
    const std::string log_entry = "[INFO] " + std::string(buffer);
    getLogDispatcher().log(LOG_LEVEL_INFO, log_entry);
    va_end(args);

#endif // LDPLAB_BUILD_OPTION_DISABLE_LOGGING
}

void ldplab::utils::Log::logDebug(const char* format, ...)
{
#ifndef LDPLAB_BUILD_OPTION_DISABLE_LOGGING
#ifdef LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING

    if (getLogDispatcher().numCallbacks() == 0)
        return;

    char buffer[MAX_LOG_ENTRY_STRING];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, MAX_LOG_ENTRY_STRING, format, args);
    const std::string log_entry = "[DEBUG] " + std::string(buffer);
    getLogDispatcher().log(LOG_LEVEL_DEBUG, log_entry);
    va_end(args);

#endif // LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING
#endif // LDPLAB_BUILD_OPTION_DISABLE_LOGGING
}

void ldplab::utils::Log::logTrace(const char* format, ...)
{
#ifndef LDPLAB_BUILD_OPTION_DISABLE_LOGGING
#ifdef LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING

    if (getLogDispatcher().numCallbacks() == 0)
        return;

    char buffer[MAX_LOG_ENTRY_STRING];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, MAX_LOG_ENTRY_STRING, format, args);
    const std::string log_entry = "[TRACE] " + std::string(buffer);
    getLogDispatcher().log(LOG_LEVEL_TRACE, log_entry);
    va_end(args);

#endif // LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING
#endif // LDPLAB_BUILD_OPTION_DISABLE_LOGGING
}

ldplab::utils::Log::LogDispatcher& ldplab::utils::Log::getLogDispatcher()
{
    static LogDispatcher log_dispatcher;
    return log_dispatcher;
}

ldplab::utils::Log::LogDispatcher::~LogDispatcher()
{
    std::lock_guard<std::mutex> guard{ m_callback_mutex };
    for (std::list<ILogCallback*>::iterator it = m_callbacks.begin();
        it != m_callbacks.end();
        ++it)
    {
        std::lock_guard<std::mutex> callback_lock{ (*it)->m_mutex };
        (*it)->m_state = ILogCallback::State::dispatcher_deconstructed;
    }
    m_callbacks.clear();
}

void ldplab::utils::Log::LogDispatcher::log(
    LogLevel log_level, const std::string& log_entry)
{
    std::lock_guard<std::mutex> guard{ m_log_mutex };
    for (std::list<ILogCallback*>::iterator it = m_callbacks.begin();
        it != m_callbacks.end();
        ++it)
    {
        if ((*it)->m_log_level >= log_level)
            (*it)->log(log_level, log_entry);
    }
}

size_t ldplab::utils::Log::LogDispatcher::numCallbacks()
{
    std::lock_guard<std::mutex> guard{ m_callback_mutex };
    return m_callbacks.size();
}

bool ldplab::utils::Log::LogDispatcher::subscribeCallback(ILogCallback* callback)
{
    std::lock_guard<std::mutex> guard{ m_callback_mutex };
    for (std::list<ILogCallback*>::iterator it = m_callbacks.begin();
        it != m_callbacks.end();
        ++it)
    {
        if (*it == callback)
            return false;
    }
    m_callbacks.push_back(callback);
    return true;
}

bool ldplab::utils::Log::LogDispatcher::unsubscribeCallback(ILogCallback* callback)
{
    std::lock_guard<std::mutex> guard{ m_callback_mutex };
    for (std::list<ILogCallback*>::iterator it = m_callbacks.begin();
        it != m_callbacks.end();
        ++it)
    {
        if (*it == callback)
        {
            m_callbacks.remove(callback);
            return true;
        }
    }
    return false;
}