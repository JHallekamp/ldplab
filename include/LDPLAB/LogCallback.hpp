#ifndef WWU_LDPLAB_LOG_CALLBACK_HPP
#define WWU_LDPLAB_LOG_CALLBACK_HPP

#include <fstream>
#include <mutex>
#include <string>

namespace ldplab
{
    // Prototypes
    namespace utils { class Log; }

    /** @brief Defines the level of severity of a log entry. */
    enum LogLevel
    {
        /** @brief Logs a fatal issue, that causes application abort. */
        LOG_LEVEL_FATAL = 0,
        /** @brief Logs an error that prevents normal operation. */
        LOG_LEVEL_ERROR,
        /** @brief Logs a warning that might be relevant to operators. */
        LOG_LEVEL_WARNING,
        /** @brief Logs an information during normal operation. */
        LOG_LEVEL_INFO,
        /** @brief Logs debug information for framework developers. */
        LOG_LEVEL_DEBUG,
        /** @brief Logs very detailed trace information. */
        LOG_LEVEL_TRACE
    };

    /**
     * @brief Abstract baseclass for log callbacks.
     * @details A log callback is an entity that can subscribe to the LDPLAB 
     *          Log class. Upon registration the callbacks log function will be
     *          called whenever a new log entry is available.
     * @note The LDPLAB Log class will use this callback in a thread-safe way.
     */
    class ILogCallback
    {
        friend utils::Log;
    public:
        ILogCallback();
        virtual ~ILogCallback();
        /**
         * @brief Applies a log level filter.
         * @details Log entries with a higher log level than the given filter
         *          will not trigger the log callback.
         * @param[in] log_level The treshold above which log entries are
         *                      ignored.
         * @note The default log level filter is LOG_LEVEL_INFO
         */
        void setLogLevel(LogLevel log_level);
    protected:
        /**
         * @brief Subscribes this callback to the LDPLAB Log class.
         * @details Upon subscription, the callback will receive log entries.
         */
        void subscribeLog();
        /**
         * @brief Unsubscribes this callback from the LDPLAB Log class.
         * @details Upon unsubscription, the callback will no longer receive
         *          log entries.
         */
        void unsubscribeLog();
        /**
         * @brief This is the callback called (upon subscription) when a log
         *        entry is logged to the LDPLAB Log class.
         * @param[in] log_level The level of severity of the log entry.
         * @param[in] log_entry The log entry as a string.
         */
        virtual void log(LogLevel log_level, const std::string& log_entry) = 0;
    private:
        enum class State : char {
            unsubscribed,
            subscribed,
            dispatcher_deconstructed
        } m_state;
        int m_log_level;
        std::mutex m_mutex;
    };

    /**
     * @brief Implements a log callback that writes log entries to a fstream.
     */
    class LogCallbackFileStream : public ILogCallback
    {
    public:
        /**
         * @brief Constructs the log callback.
         * @param[in] path The path to the file into which the log entries are
         *                 written.
         */
        LogCallbackFileStream(const std::string& path);
        ~LogCallbackFileStream();
        
        /**
         * @brief Subscribes this callback to the LDPLAB Log class.
         * @details Upon subscription, the callback will receive log entries.
         */
        void subscribe();
        /**
         * @brief Unsubscribes this callback from the LDPLAB Log class.
         * @details Upon unsubscription, the callback will no longer receive
         *          log entries.
         */
        void unsubscribe();
    protected:
        /**
         * @brief This is the callback called (upon subscription) when a log
         *        entry is logged to the LDPLAB Log class.
         * @param[in] log_level The level of severity of the log entry.
         * @param[in] log_entry The log entry as a string.
         */
        void log(LogLevel log_level, const std::string& log_entry) override;
    private:
        std::ofstream file;
    };

    /**
     * @brief Implements a log callback that writes log entries to the stdout.
     */
    class LogCallbackStdout : public ILogCallback
    {
    public:
        /**
         * @brief Subscribes this callback to the LDPLAB Log class.
         * @details Upon subscription, the callback will receive log entries.
         */
        void subscribe();
        /**
         * @brief Unsubscribes this callback from the LDPLAB Log class.
         * @details Upon unsubscription, the callback will no longer receive
         *          log entries.
         */
        void unsubscribe();
    protected:
        /**
         * @brief This is the callback called (upon subscription) when a log
         *        entry is logged to the LDPLAB Log class.
         * @param[in] log_level The level of severity of the log entry.
         * @param[in] log_entry The log entry as a string.
         */
        void log(LogLevel log_level, const std::string& log_entry) override;
    };
}

#endif