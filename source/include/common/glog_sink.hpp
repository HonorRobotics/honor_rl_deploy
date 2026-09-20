#ifndef GLOG_SINK_HPP
#define GLOG_SINK_HPP

#include <glog/logging.h>
#include <boost/asio.hpp>
#include <memory>
#include <mutex>
#include <fstream>
#include <queue>
#include <thread>
#include <string>
#include <iostream>
#include <chrono>
#include <sys/time.h>
#include <vector>  // thread pool

// log message struct
struct LogMessage {
    google::LogSeverity severity;
    std::string base_filename;
    int line;
    std::string time_str;
    std::string message;
};
class LogManager {
   public:
    class CustomFileSink : public google::LogSink {
       public:
        explicit CustomFileSink(const std::string& base_filename);
        ~CustomFileSink() override;

        void send(google::LogSeverity severity, const char* full_filename, const char* base_filename, int line,
                  const struct ::tm* tm_time, const char* message, size_t message_len) override;

       private:
        void flush_logs();
        void start_flush_timer();

        // Member variables
        std::string base_filename_;
        boost::asio::io_context io_context_;
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
        std::vector<std::thread> thread_pool_;
        boost::asio::steady_timer flush_timer_;
        std::mutex queue_mutex_;
        std::mutex file_mutex_;
        std::queue<LogMessage> log_queue_;
        bool running_;
        static constexpr size_t batch_size_ = 5;
        static constexpr size_t thread_pool_size_ = 4;  // Added: thread pool size
    };

    // Singleton accessor
    static LogManager& GetInstance() {
        static LogManager instance;
        return instance;
    }

    // Initialize the logging system
    void Initialize(const std::string& log_file, const char* argv0);

   private:
    LogManager() = default;
    ~LogManager();
    std::unique_ptr<CustomFileSink> custom_sink_;
};

#endif  // GLOG_SINK_HPP