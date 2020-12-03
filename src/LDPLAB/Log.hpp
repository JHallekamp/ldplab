#ifndef WWU_LDPLAB_LOG_HPP
#define WWU_LDPLAB_LOG_HPP

#include "LogCallback.hpp"

#include <mutex>
#include <list>

#ifndef LDPLAB_BUILD_OPTION_DISABLE_LOGGING
#   define LDPLAB_LOG_FATAL(format, ...) ldplab::Log::logFatal(format, __VA_ARGS__)
#   define LDPLAB_LOG_ERROR(format, ...) ldplab::Log::logError(format, __VA_ARGS__)
#   define LDPLAB_LOG_WARNING(format, ...) ldplab::Log::logWarning(format, __VA_ARGS__)
#   define LDPLAB_LOG_INFO(format, ...) ldplab::Log::logInfo(format, __VA_ARGS__)
#   ifdef LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING
#       define LDPLAB_LOG_DEBUG(format, ...) ldplab::Log::logDebug(format, __VA_ARGS__)
#       define LDPLAB_LOG_TRACE(format, ...) ldplab::Log::logTrace(format, __VA_ARGS__)
#   else // LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING
#       define LDPLAB_LOG_DEBUG(format, ...)
#       define LDPLAB_LOG_TRACE(format, ...)
#   endif // LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING
#else // LDPLAB_BUILD_OPTION_DISABLE_LOGGING
#   define LDPLAB_LOG_FATAL(format, ...)
#   define LDPLAB_LOG_ERROR(format, ...)
#   define LDPLAB_LOG_WARNING(format, ...)
#   define LDPLAB_LOG_INFO(format, ...)
#   define LDPLAB_LOG_DEBUG(format, ...)
#   define LDPLAB_LOG_TRACE(format, ...)
#endif // LDPLAB_BUILD_OPTION_DISABLE_LOGGING

namespace ldplab
{
    /**
     * @brief Log provides static methods used for logging.
     * @details Log entries are distributed to subscribed callbacks. If no 
     *          callbacks are subscriped, each log entry is discarded.
     * @note It is recommended to use the LDPLAB_LOG_xx macros to perform
     *       actual logging.
     * @note Logging can be deactivated during framework build time by defining
     *       the LDPLAB_BUILD_OPTION_DISABLE_LOGGING macro.
     */
    class Log
    {
        friend ILogCallback;
    public:
        /** @brief Checks whether logging is enabled. */
        static bool isLoggingEnabled();
        /** @brief Checks whether debug logging is enabled. */
        static bool isDebugLoggingEnabled();
        /** @brief Logs a fatal issue that results in application abort. */
        static void logFatal(const char* format, ...);
        /** @brief Logs an error that prevents correct operation. */
        static void logError(const char* format, ...);
        /** @brief Logs a warning that might be interessting for operators. */
        static void logWarning(const char* format, ...);
        /** @brief Logs an information during normal operation. */
        static void logInfo(const char* format, ...);
        /**
         * @brief Logs debug information.
         * @note Debug logging has to be explicitly activated for framework
         *       builds by defining LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING
         *       build time macro.
         */
        static void logDebug(const char* format, ...);
        /**
         * @brief Logs very detailed trace info.
         * @note Debug logging has to be explicitly activated for framework
         *       builds by defining LDPLAB_BUILD_OPTION_ENABLE_DEBUG_LOGGING
         *       build time macro.
         */
        static void logTrace(const char* format, ...);
    private:
        class LogDispatcher
        {
        public:
            ~LogDispatcher();
            void log(LogLevel log_level, const std::string& log_entry);
            unsigned int numCallbacks();
            bool subscribeCallback(ILogCallback* callback);
            bool unsubscribeCallback(ILogCallback* callback);
        private:
            std::list<ILogCallback*> m_callbacks;
            std::mutex m_log_mutex;
            std::mutex m_callback_mutex;
        };
    private:
        static  LogDispatcher& getLogDispatcher();
    };
}

#endif