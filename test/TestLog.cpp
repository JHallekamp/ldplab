#include <ldplab.hpp>
#include "../src/Utils/Log.hpp"
#include <iostream>

void printConfiguration()
{
    if (ldplab::BuildInfo::loggingEnabled())
        std::cout << "logging is enabled" << std::endl;
    else
        std::cout << "logging is not enabled" << std::endl;

    if (ldplab::BuildInfo::debugLoggingEnabled())
        std::cout << "debug logging is enabled" << std::endl;
    else
        std::cout << "debug logging is not enabled" << std::endl;
}

void testLog(int time)
{
    LDPLAB_LOG_FATAL("log no. %i", time);
    LDPLAB_LOG_ERROR("log no. %i", time);
    LDPLAB_LOG_WARNING("log no. %i", time);
    LDPLAB_LOG_INFO("log no. %i", time);
    LDPLAB_LOG_DEBUG("log no. %i", time);
    LDPLAB_LOG_TRACE("log no. %i", time);
}

int main()
{
    ldplab::LogCallbackFileStream fstream_log{"test.log"};
    fstream_log.subscribe();
    ldplab::LogCallbackStdout stdout_log;
    stdout_log.subscribe();
    stdout_log.setLogLevel(ldplab::LOG_LEVEL_TRACE);

    printConfiguration();
    testLog(0);

    stdout_log.unsubscribe();
    testLog(1);

    stdout_log.subscribe();
    testLog(2);

    return 0;
}