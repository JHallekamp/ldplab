#include "LogCallback.hpp"
#include "Utils/Log.hpp"

#include <iostream>

ldplab::ILogCallback::ILogCallback()
    :
    m_state { State::unsubscribed },
    m_log_level { LOG_LEVEL_INFO }
{ }

ldplab::ILogCallback::~ILogCallback()
{
    unsubscribeLog();
}

void ldplab::ILogCallback::setLogLevel(LogLevel log_level)
{
    m_log_level = log_level;
}

void ldplab::ILogCallback::subscribeLog()
{
    std::lock_guard<std::mutex> guard{ m_mutex };
    if (m_state == State::unsubscribed)
    {
        if (Log::getLogDispatcher().subscribeCallback(this))
            m_state = State::subscribed;
    }
}

void ldplab::ILogCallback::unsubscribeLog()
{
    std::lock_guard<std::mutex> guard{ m_mutex };
    if (m_state == State::subscribed)
    {
        if (Log::getLogDispatcher().unsubscribeCallback(this))
            m_state = State::unsubscribed;
    }
}

ldplab::LogCallbackFileStream::LogCallbackFileStream(const std::string& path)
{
    file = std::ofstream(path);
}

ldplab::LogCallbackFileStream::~LogCallbackFileStream()
{
    file.close();
}

void ldplab::LogCallbackFileStream::subscribe()
{
    subscribeLog();
}

void ldplab::LogCallbackFileStream::unsubscribe()
{
    unsubscribeLog();
}

void ldplab::LogCallbackFileStream::log(LogLevel log_level, const std::string& log_entry)
{
    file << log_entry << std::endl;
}

void ldplab::LogCallbackStdout::subscribe()
{
    subscribeLog();
}

void ldplab::LogCallbackStdout::unsubscribe()
{
    unsubscribeLog();
}

void ldplab::LogCallbackStdout::log(LogLevel log_level, const std::string& log_entry)
{
    std::cout << log_entry << std::endl;
}
