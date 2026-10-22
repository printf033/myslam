#pragma once

/**
 * @file log_config.hpp
 * @brief mylog 的 C++ API 配置封装：日志级别 + FileManager 文件输出
 *
 * 日志文件由 mylog::FileManager 管理，使用其共享单文件输出回调、缓冲、
 * 滚动和保留策略。文件名由 FileManager 按时间戳和线程号生成，存放在当前
 * 工作目录下。
 *
 * @code
 * myslam::LogConfig logConfig;
 * logConfig.level = "debug";
 * logConfig.fileManager.fileCapacity = 50ULL << 20;
 * myslam::configureLogging(logConfig);
 * @endcode
 */

#include "fileManager.hpp"
#include <string>

namespace myslam
{

    /**
     * @brief 日志配置
     */
    struct LogConfig
    {
        /// FileManager 运行配置，包含缓冲、文件容量、刷盘间隔和历史文件数
        mylog::FileManager::Config fileManager{};
        /// 抑制门限级别（忽略大小写）：trace / debug / info / warn / error / fatal
        std::string level{"info"};
    };

    /**
     * @brief 通过 mylog C++ API 配置日志：级别 + FileManager 共享单文件输出
     *
     * @param config 日志配置
     * @return true 配置成功
     * @return false 日志级别无效；此时不修改当前输出配置
     */
    inline bool configureLogging(const LogConfig &config = {})
    {
        if (!mylog::setSuppressLevel(config.level))
        {
            return false;
        }

        mylog::FileManager::getInstance().setConfig(config.fileManager);
        mylog::Logger::getInstance().setOutputFunction(mylog::FileManager::createFileOutputFunction());
        mylog::Logger::getInstance().setFlushFunction(mylog::FileManager::createFileFlushFunction());
        return true;
    }

    /**
     * @brief 刷新当前线程的 FileManager 日志缓冲区
     */
    inline void flushLogging()
    {
        mylog::FileManager::getInstance().flushFile();
    }

} // namespace myslam
