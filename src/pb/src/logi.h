#ifndef LOGI_H
#define LOGI_H

#pragma once
#include <cstdio>
#include <cstdarg>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

class AsyncFileLogger
{
public:
    AsyncFileLogger(const std::string& folderPath, const std::string& prefix)
    {
        // создаём директорию, если её нет
        createFolderIfNotExist(folderPath);

        // формируем имя файла
        std::string filename = generateFileName(folderPath, prefix);

        // открываем файл
        file_ = std::fopen(filename.c_str(), "a");
        if (!file_) {
            std::perror("AsyncFileLogger fopen");
            throw std::runtime_error("Cannot open log file");
        }

        running_ = true;
        workerThread_ = std::thread(&AsyncFileLogger::threadFunc, this);
    }

    ~AsyncFileLogger()
    {
        running_ = false;
        cv_.notify_all();
        if (workerThread_.joinable())
            workerThread_.join();

        if (file_)
            std::fclose(file_);
    }

    void log(const char* fmt, ...)
    {
        char buffer[1024];
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);

        auto now = std::chrono::system_clock::now();

        std::ostringstream oss;
        auto itt = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
        localtime_r(&itt, &tm);

        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) % 1000;

        oss << "[" << std::setw(4) << tm.tm_year + 1900
            << "-" << std::setw(2) << std::setfill('0') << tm.tm_mon + 1
            << "-" << std::setw(2) << std::setfill('0') << tm.tm_mday
            << " " << std::setw(2) << tm.tm_hour
            << ":" << std::setw(2) << tm.tm_min
            << ":" << std::setw(2) << tm.tm_sec
            << "." << std::setw(3) << ms.count()
            << "] " << buffer; // перенос только если есть в fmt

        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            messageQueue_.push(oss.str());
        }
        cv_.notify_one();
    }

private:
    FILE* file_ = nullptr;
    std::queue<std::string> messageQueue_;
    mutable std::mutex queueMutex_;
    std::condition_variable cv_;
    std::thread workerThread_;
    std::atomic<bool> running_{false};

    static std::string generateFileName(const std::string& folder, const std::string& prefix)
    {
        std::time_t t = std::time(nullptr);
        std::tm tm{};
        localtime_r(&t, &tm);

        std::ostringstream oss;
        oss << folder;
        if (!folder.empty() && folder.back() != '/') oss << "/";
        oss << prefix << "_"
            << (tm.tm_year + 1900) << "-"
            << std::setw(2) << std::setfill('0') << (tm.tm_mon + 1) << "-"
            << std::setw(2) << std::setfill('0') << tm.tm_mday << "_"
            << std::setw(2) << tm.tm_hour << "-"
            << std::setw(2) << tm.tm_min << "-"
            << std::setw(2) << tm.tm_sec
            << ".log";
        return oss.str();
    }

    void threadFunc()
    {
        while (running_ || !messageQueue_.empty())
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            cv_.wait(lock, [this] { return !messageQueue_.empty() || !running_; });

            while (!messageQueue_.empty())
            {
                const std::string& msg = messageQueue_.front();
                std::fwrite(msg.c_str(), 1, msg.size(), file_);
                messageQueue_.pop();
            }

            std::fflush(file_);
        }
    }

    static void createFolderIfNotExist(const std::string& path)
    {
        struct stat st = {0};
        if (stat(path.c_str(), &st) == -1)
        {
            // создаём директорию рекурсивно (аналог mkdir -p)
            mkdir(path.c_str(), 0755);
        }
    }

    AsyncFileLogger(const AsyncFileLogger&) = delete;
    AsyncFileLogger& operator=(const AsyncFileLogger&) = delete;
};



/*
#include "AsyncFileLogger.hpp"

int main()
{
    // Папка создастся автоматически, если её нет
    AsyncFileLogger log("/home/pi/RoboBIM-Linux/src/pb/log", "pose_node");

    double x = 3.14;

    log.log("x = %f\n", x);       // перенос добавится только если есть в fmt
    log.log("step = %d", 42);     // без переноса

    return 0;
}

*/

#endif