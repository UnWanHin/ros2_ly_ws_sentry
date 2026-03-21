// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <string>
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>  // 请确保已安装 nlohmann/json 库
#include <Logger/Logger.hpp>

//usage:    param.get<std::string>("model_path")
//          param["model_path"].to<std::string>()
//          param["model_path"].String()
namespace param{

    class Param {
    public:
        // 构造函数：从指定文件中加载 JSON 参数
        explicit Param(const std::string & filePath) {
            std::ifstream in(filePath);
            if (!in.is_open()) {
                roslog::error("无法打开文件：{}", filePath);
            }
            try {
                in >> json_;
            } catch (const nlohmann::json::parse_error & e) {
                roslog::error("JSON解析错误: {}", e.what());
            }
        }
        
        // 支持层次调用，返回子级 Param 对象
        Param operator[](const std::string & key) const {
            if (!json_.contains(key)) {
                roslog::error("参数不存在：{}", key);
            }
            return Param(json_.at(key), InternalTag::Internal);
        }
        
        // 模板接口：根据参数名称获取对应类型的值
        template<typename T>
        T get(const std::string & key) const {
            if (!json_.contains(key)) {
                roslog::error("参数不存在：{}", key);
            }
            return json_.at(key).template get<T>();
        }

        template<typename T>
        T to() const {
            return json_.template get<T>();
        }
        
        // 判断某参数是否存在
        bool exists(const std::string & key) const {
            return json_.contains(key);
        }

        int Int() const {
            return to<int>();
        }

        float Float() const {
            return to<float>();
        }
        
        double Double() const {
            return to<double>();
        }
        
        bool Bool() const {
            return to<bool>();
        }

        std::string String() const {
            return to<std::string>();
        }

    private:
        // 内部标识类型，用于区分内部构造函数
        enum class InternalTag { Internal };

        // 私有构造函数，通过 JSON 对象构造 Param（用于 operator[] 链式调用）
        explicit Param(const nlohmann::json & j, InternalTag) : json_(j) {}

        nlohmann::json json_;
    };

}