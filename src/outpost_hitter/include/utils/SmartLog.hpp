// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <chrono>
#include <iostream>
#include <unordered_map>

// 自定义cout，方便调试时监视
namespace LY_UTILS
{
#define autoaim_debug true

#define GET_PACKAGE_NAME(file) ([] { \
    std::string filePath(file); \
    size_t pos = filePath.find("/modules/"); \
    if (pos != std::string::npos) { \
        size_t start = pos + strlen("/modules/"); \
        size_t end = filePath.find("/", start); \
        if (end != std::string::npos) { \
            return filePath.substr(start, end - start); \
        } \
    } \
    return std::string(""); }())

// ANSI转义码
#define RESET "\033[0m"

#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN "\033[36m"
#define ORANGE "\033[38;5;208m"

#define BOLD "\033[1m"
#define UNDERLINE "\033[4m"

#define BLACKBACK "\033[40m"
#define REDBACK "\033[41m"
#define GREENBACK "\033[42m"
#define YELLOWBACK "\033[43m"
#define BLUEBACK "\033[44m"
#define MAGENTABACK "\033[45m"
#define CYANBACK "\033[46m"
#define GREYBACK "\033[47m"

#define COUT(text, color)                                                                                                           \
    do                                                                                                                              \
    {                                                                                                                               \
        if (autoaim_debug)                                                                                                          \
        {                                                                                                                           \
            std::string name = GET_PACKAGE_NAME(__BASE_FILE__);                                                                     \
            if (name == "simulator" || name == "driver")                                                                            \
            {                                                                                                                       \
                /*std::cout << BLUEBACK << BOLD << "{" << name << "}:" << RESET << " - " << color << text << RESET << std::endl; */ \
            }                                                                                                                       \
            else if (name == "encoder")                                                                                             \
            {                                                                                                                       \
                std::cout << GREYBACK << BOLD << "{" << name << "}:" << RESET << " - " << color << text << RESET << std::endl;      \
            }                                                                                                                       \
            else if (name == "detector")                                                                                            \
            {                                                                                                                       \
                std::cout << GREENBACK << BOLD << "{" << name << "}:" << RESET << " - " << color << text << RESET << std::endl;     \
            }                                                                                                                       \
            else if (name == "tracker")                                                                                             \
            {                                                                                                                       \
                std::cout << YELLOWBACK << BOLD << "{" << name << "}:" << RESET << " - " << color << text << RESET << std::endl;    \
            }                                                                                                                       \
            else if (name == "predictor")                                                                                           \
            {                                                                                                                       \
                std::cout << MAGENTABACK << BOLD << "{" << name << "}:" << RESET << " - " << color << text << RESET << std::endl;   \
            }                                                                                                                       \
            else if (name == "solver")                                                                                              \
            {                                                                                                                       \
                std::cout << CYANBACK << BOLD << "{" << name << "}:" << RESET << " - " << color << text << RESET << std::endl;      \
            }                                                                                                                       \
        }                                                                                                                           \
    } while (0)

}