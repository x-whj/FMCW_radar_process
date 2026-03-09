#pragma once
#include <iostream>
#include <mutex>
#include <sstream>
namespace radar
{
    class Logger
    {
    public:
        template <typename... Args>
        static void info(Args &&...args)
        {
            log("[INFO] ", std::forward<Args>(args)...);
        }
        template <typename... Args>
        static void warn(Args &&...args)
        {
            log("[WARN] ", std::forward<Args>(args)...);
        }
        template <typename... Args>
        static void error(Args &&...args)
        {
            log("[ERROR] ", std::forward<Args>(args)...);
        }

    private:
        template <typename... Args>
        static void log(const char *prefix, Args &&...args)
        {
            static std::mutex m;
            std::lock_guard<std::mutex> lk(m);
            std::ostringstream oss;
            (oss << ... << args);
            std::cout << prefix << oss.str() << std::endl;
        }
    };
} // namespace radar
