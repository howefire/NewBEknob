#pragma once


#pragma once

#include "../proto_gen/smartknob.pb.h"
#include "motor_task.h"
#include "serial_protocol.h"
#include "uart_stream.h"
#include "cJSON.h"


class SerialProtocolJson: public SerialProtocol{
    public:
    //接收参数后直接将其实例化
    SerialProtocolJson(Stream& stream, MotorTask& motor_task)
    : SerialProtocol(),
    stream_(stream),
    motor_task_(motor_task){}
    ~SerialProtocolJson(){}
   
    void log(const char* msg) override ;
    void loop() override;
    void handleState(const PB_SmartKnobState& state) override;
    void Json_Analyze_func(cJSON* root);//以后你就属于我字写的类啦哈哈
    template<typename T>
    void Json_Adjust_SmartKnobConfig(uint8_t Specific_Mode,uint8_t Specific_data,T data);  

    char recvdata[2048];
    private:
        Stream& stream_;
        MotorTask& motor_task_;
        PB_SmartKnobState latest_state_ = {};
        
        
};
