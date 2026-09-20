#include "common/glog_sink.hpp"

LogManager::CustomFileSink::CustomFileSink(const std::string& base_filename)
    : base_filename_(base_filename),
      work_guard_(boost::asio::make_work_guard(io_context_)),
      running_(true),
      flush_timer_(io_context_) {
    // create thread pool
    thread_pool_.reserve(thread_pool_size_);
    for (size_t i = 0; i < thread_pool_size_; ++i) {
        thread_pool_.emplace_back([this] { io_context_.run(); });
    }
    std::ofstream outfile(base_filename_, std::ios_base::app);
    if (!outfile) {
        std::cerr << "Failed to open log file: " << base_filename_ << std::endl;
    }

    // Start the first timer
    start_flush_timer();
}

LogManager::CustomFileSink::~CustomFileSink() {
    running_ = false;
    work_guard_.reset();
    flush_timer_.cancel();

    // wait for all threads to finish
    for (auto& thread : thread_pool_) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    flush_logs();
}

void LogManager::CustomFileSink::send(
    google::LogSeverity severity,
    const char* full_filename,
    const char* base_filename,
    int line,
    const struct ::tm* tm_time,
    const char* message,
    size_t message_len) {
    // Get current precise time
    struct timeval tv;
    gettimeofday(&tv, nullptr);

    // actual time
    struct tm result;
    localtime_r(&tv.tv_sec, &result);

    // seconds
    char time_buffer[30];
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", &result);

    // microsecond
    char full_time_buffer[50];
    snprintf(full_time_buffer, sizeof(full_time_buffer), "%s.%06ld", time_buffer, static_cast<long>(tv.tv_usec));

    // Create log message object
    LogMessage log_msg{
        severity,
        base_filename ? std::string(base_filename) : "",
        line,
        std::string(full_time_buffer),
        std::string(message, message_len)
    };
    if (!running_) return;
    // Post the log message object to the thread pool
    boost::asio::post(io_context_, [this, msg = std::move(log_msg)] {
        bool should_flush = false;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            log_queue_.push(std::move(msg));

            // Check the batch flush condition
            if (log_queue_.size() >= batch_size_) {
                should_flush = true;
                // Cancel the current timer (if any); it will be restarted afterwards
                flush_timer_.cancel();
            }
        }

        if (should_flush) {
            flush_logs();
            // Restart the 3-second timer after a batch flush
            start_flush_timer();
        }
    });
}

void LogManager::CustomFileSink::start_flush_timer() {
    if (!running_)
        return;

    flush_timer_.expires_after(std::chrono::seconds(3));
    flush_timer_.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
            return;  // Timer was cancelled

        // Flush logs when the timer fires
        flush_logs();

        // Restart the timer
        start_flush_timer();
    });
}

void LogManager::CustomFileSink::flush_logs() {
    std::queue<LogMessage> temp_queue;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (log_queue_.empty())
            return;
        std::swap(log_queue_, temp_queue);
    }
    {
        std::lock_guard<std::mutex> lock(file_mutex_);
        std::ofstream outfile(base_filename_, std::ios_base::app);
        if (!outfile) {
            std::cerr << "Failed to open log file for appending: " << base_filename_ << std::endl;
            return;
        }

        // format and write the log
        while (!temp_queue.empty()) {
            const auto& msg = temp_queue.front();

            // Format the log message
            std::string formatted_log;

            // Append time and severity
            formatted_log += msg.time_str + " ";

            // Append the severity level
            switch (msg.severity) {
                case google::INFO:
                    formatted_log += "I";
                    break;
                case google::WARNING:
                    formatted_log += "W";
                    break;
                case google::ERROR:
                    formatted_log += "E";
                    break;
                case google::FATAL:
                    formatted_log += "F";
                    break;
                default:
                    formatted_log += "?";
            }

            // Append filename, line number and message content
            formatted_log += " " + msg.base_filename + ":" + std::to_string(msg.line) + "] " + msg.message;

            outfile << formatted_log << std::endl;
            temp_queue.pop();
        }
    }
}

void LogManager::Initialize(const std::string& log_file, const char* argv0) {
    custom_sink_ = std::make_unique<CustomFileSink>(log_file);
    google::AddLogSink(custom_sink_.get());
    // Disable the default log destinations
    google::SetLogDestination(google::INFO, "");
    google::SetLogDestination(google::WARNING, "");
    google::SetLogDestination(google::ERROR, "");
    google::SetLogDestination(google::FATAL, "");

    google::InitGoogleLogging(argv0);
}

LogManager::~LogManager() {
    if (custom_sink_) {
        google::RemoveLogSink(custom_sink_.get());
    }
    google::ShutdownGoogleLogging();
}
