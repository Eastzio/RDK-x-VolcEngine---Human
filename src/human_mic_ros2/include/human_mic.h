#ifndef __human_mic_H_
#define __human_mic_H_

#include <string>
#include <iostream>
#include <unistd.h>
#include <serial/serial.h>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/u_int32.hpp"
#include "jsoncpp/json/json.h"
#include "rclcpp/rclcpp.hpp"
#include "human_mic_msg/srv/set_awake_word.hpp"
#include "human_mic_msg/srv/get_device_type.hpp"
#include "human_mic_msg/srv/switch_mic.hpp"

using namespace std;

#define FRAME_HEADER        0XA5        
#define USER_ID             0X01        
#define TIMEOUT             10.0

enum class MsgType : unsigned char {
    Shake =             0x01,
    AIUI_MSG =          0x04,
    CONTROL =           0x05,
    CONFIRM =           0xFF
};

enum class ServiceType{
    AWAKE_WORD,
    SWITCH_MIC,
    DEVICE_VER
};

struct ServicePacket
{
    unsigned short sid;

    string type;
    string threshold;
    string awake_word;
    string mic_type;

    ServiceType msgType;
};

struct MsgPacket
{
    unsigned char uid;
    unsigned char type;

    unsigned short size;
    unsigned short sid;

    string bytes;
};

class human_mic : public rclcpp::Node{
public:
    human_mic(const std::string &node_name);
    ~human_mic();
	void run();

    serial::Serial MicArr_Serial;

private:
    int angle,serial_baud_rate;
    bool process_result;
    bool serial_initialized;
    unsigned char Receive_Data[1024] = {0};
    string device_message,usart_port_name;

    MsgPacket MsgPkg;
    ServicePacket ServicePkg; 

    rclcpp::Time start_time, last_time;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr angle_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_words_pub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr awake_flag_pub,voice_flag_pub;
    rclcpp::Service<human_mic_msg::srv::GetDeviceType>::SharedPtr get_device_srv_;
    rclcpp::Service<human_mic_msg::srv::SetAwakeWord>::SharedPtr set_awake_word_srv_;
    rclcpp::Service<human_mic_msg::srv::SwitchMic>::SharedPtr switch_mic_srv_;

    bool Get_Serial_Data();
    bool UnPackMsgPacket(const string &content, MsgPacket &data);
    void setupMicArrayServices();
    void handle_serial_error();
    void initialize_serial();
    int uart_analyse(unsigned char buffer);
    int process_data(const unsigned char *buf, int len);
    string MakeMsgPacket(unsigned short sid, MsgType type, const string &content);

    bool Send_Serial_Data(ServicePacket &pkg);
    bool getDeviceTypeCallback(const std::shared_ptr<human_mic_msg::srv::GetDeviceType::Request>& request,
                                std::shared_ptr<human_mic_msg::srv::GetDeviceType::Response>& response);
    bool setAwakeWordCallback(const std::shared_ptr<human_mic_msg::srv::SetAwakeWord::Request>& request,
                                std::shared_ptr<human_mic_msg::srv::SetAwakeWord::Response>& response);
    bool switchMicCallback(const std::shared_ptr<human_mic_msg::srv::SwitchMic::Request>& request,
                                std::shared_ptr<human_mic_msg::srv::SwitchMic::Response>& response);
};

#endif
