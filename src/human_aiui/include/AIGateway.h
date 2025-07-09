// AIGateway.h
#ifndef AI_GATEWAY_H
#define AI_GATEWAY_H

#include <string>
#include <functional>
#include <thread>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>

class AIGateway {
public:
    AIGateway(const std::string& base_url, const std::string& api_key);
    ~AIGateway();
    
    // 异步发送消息到AI网关
    void sendMessage(const std::string& message, 
                    std::function<void(const std::string&)> callback);
    
    // 设置系统提示词
    void setSystemPrompt(const std::string& prompt);
    
private:
    std::string base_url_;
    std::string api_key_;
    std::string system_prompt_;
    
    // HTTP响应回调
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* data);
    
    // 发送HTTP请求
    std::string sendHttpRequest(const std::string& json_data);
    
    // 构建请求JSON
    std::string buildRequestJson(const std::string& message);
};

#endif // AI_GATEWAY_H
