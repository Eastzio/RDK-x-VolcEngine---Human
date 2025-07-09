// AIGateway.cpp
#include "AIGateway.h"
#include <iostream>
#include <sstream>
#include <jsoncpp/json/json.h>

AIGateway::AIGateway(const std::string& base_url, const std::string& api_key)
    : base_url_(base_url), api_key_(api_key), 
      system_prompt_("You are a helpful assistant.") {
    curl_global_init(CURL_GLOBAL_DEFAULT);
}

AIGateway::~AIGateway() {
    curl_global_cleanup();
}

void AIGateway::setSystemPrompt(const std::string& prompt) {
    system_prompt_ = prompt;
}

void AIGateway::sendMessage(const std::string& message, 
                           std::function<void(const std::string&)> callback) {
    // 在新线程中处理请求，避免阻塞主线程
    std::thread([this, message, callback]() {
        try {
            std::string json_data = buildRequestJson(message);
            std::string response = sendHttpRequest(json_data);
            
            // 解析响应
            ::Json::Value root;
            ::Json::Reader reader;
            if (reader.parse(response, root)) {
                if (root.isMember("choices") && root["choices"].isArray() && 
                    root["choices"].size() > 0) {
                    ::Json::Value choice = root["choices"][0];
                    if (choice.isMember("message") && 
                        choice["message"].isMember("content")) {
                        std::string content = choice["message"]["content"].asString();
                        callback(content);
                        return;
                    }
                }
            }
            
            // 如果解析失败，返回错误信息
            callback("抱歉，我无法理解您的问题。");
            
        } catch (const std::exception& e) {
            std::cerr << "AI Gateway error: " << e.what() << std::endl;
            callback("抱歉，服务暂时不可用。");
        }
    }).detach();
}

std::string AIGateway::buildRequestJson(const std::string& message) {
    ::Json::Value root;
    ::Json::Value messages(::Json::arrayValue);
    
    // 添加系统消息
    ::Json::Value system_msg;
    system_msg["role"] = "system";
    system_msg["content"] = system_prompt_;
    messages.append(system_msg);
    
    // 添加用户消息
    ::Json::Value user_msg;
    user_msg["role"] = "user";
    user_msg["content"] = message;
    messages.append(user_msg);
    
    root["model"] = "Doubao-1.5-pro-32k";
    root["messages"] = messages;
    
    ::Json::StreamWriterBuilder builder;
    return ::Json::writeString(builder, root);
}

std::string AIGateway::sendHttpRequest(const std::string& json_data) {
    CURL* curl;
    CURLcode res;
    std::string response_data;
    
    curl = curl_easy_init();
    if (curl) {
        // 设置URL
        std::string url = base_url_ + "/chat/completions";
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        
        // 设置请求头
        struct curl_slist* headers = nullptr;
        std::string auth_header = "Authorization: Bearer " + api_key_;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        headers = curl_slist_append(headers, auth_header.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        
        // 设置POST数据
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
        
        // 设置回调函数
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
        
        // 设置超时
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
        
        // 执行请求
        res = curl_easy_perform(curl);
        
        // 清理
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        
        if (res != CURLE_OK) {
            throw std::runtime_error("HTTP request failed: " + std::string(curl_easy_strerror(res)));
        }
    }
    
    return response_data;
}

size_t AIGateway::WriteCallback(void* contents, size_t size, size_t nmemb, std::string* data) {
    size_t total_size = size * nmemb;
    data->append((char*)contents, total_size);
    return total_size;
}
