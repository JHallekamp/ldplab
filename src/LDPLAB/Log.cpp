#include "Log.hpp"

#include <stdio.h>
#include <stdarg.h>

constexpr unsigned int MAX_LOG_ENTRY_STRING = 512;

void ldplab::Log::logFatal(const char* format, ...)
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

void ldplab::Log::logError(const char* format, ...)
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

void ldplab::Log::logWarning(const char* format, ...)
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

void ldplab::Log::logInfo(const char* format, ...)
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

void ldplab::Log::logDebug(const char* format, ...)
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

void ldplab::Log::logTrace(const char* format, ...)
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

ldplab::Log::LogDispatcher& ldplab::Log::getLogDispatcher()
{
    static LogDispatcher log_dispatcher;
    return log_dispatcher;
}

ldplab::Log::LogDispatcher::~LogDispatcher()
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

void ldplab::Log::LogDispatcher::log(
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

unsigned int ldplab::Log::LogDispatcher::numCallbacks()
{
    std::lock_guard<std::mutex> guard{ m_callback_mutex };
    return m_callbacks.size();
}

bool ldplab::Log::LogDispatcher::subscribeCallback(ILogCallback* callback)
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

bool ldplab::Log::LogDispatcher::unsubscribeCallback(ILogCallback* callback)
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

bool ldplab::Log::isLoggingEnabled()
{
#ifndef LDPLAB_BUILD_OPTION_DISABLE_LOGGING
    return true;
#else
    return false;
#endif
}

bool ldplab::Log::isDebugLoggingEnabled()
{
#ifdef LDPLAB_BUILD_OPTION_DISABLE_LOGGING
    return false;
#elif defined(LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING)
    return true;
#else
    return false;
#endif
}
