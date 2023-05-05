#pragma once

#include "../proto_gen/smartknob.pb.h"
#include "motor_task.h"
#include "serial_protocol.h"
#include "uart_stream.h"
#include <BleKeyboard.h>
#include "cJSON.h"

typedef struct {
    uint8_t device_id;
    uint8_t write_cmd;
    uint8_t str_len;
    uint8_t run_mode_list;
    uint8_t regist_star;
    uint8_t mode_p;
    uint8_t data_H;
    uint8_t data_L;

}recv_buf;

typedef union {
    float org_float;
    uint8_t after_uchar[4];
}Reterns_float;

typedef struct {
    uint8_t current_mode;
    uint8_t current_power_H;
    uint8_t current_power_L;
    uint8_t current_speed_H;
    uint8_t current_speed_L;
    uint8_t current_location_H;
    uint8_t current_location_L;
    uint8_t current_data_H;
    uint8_t current_data_L;
    uint8_t current_temp;
    uint8_t current_worngcode;
}System_run_state;


typedef struct
{
    uint8_t Send_buf[120];
    uint8_t Send_len;
   
}Sendconfig;




typedef std::function<void(void)> DemoConfigChangeCallback;
class SerialProtocolPlaintext : public SerialProtocol {
    public:
        SerialProtocolPlaintext(Stream& stream, MotorTask& motor_task) : SerialProtocol(), stream_(stream), motor_task_(motor_task) {}
        ~SerialProtocolPlaintext(){}
        void log(const char* msg) override;
        void loop() override;
        void handleState(const PB_SmartKnobState& state) override;
        void esp32_serial_send(uint8_t *data, uint8_t length);
        void send_system_state(uint8_t st,uint8_t end,PB_SmartKnobState& state);
        void init(DemoConfigChangeCallback cb);
        uint8_t crc8_MAXIM(uint8_t *data, uint8_t len);
        void changeConfigtoSpecific(uint8_t mode);
        void Adjust_SmartKnobConfig(uint8_t Specific_Mode,uint8_t Specific_data,uint8_t datalen);
        
       
    private:
        Stream& stream_;
        MotorTask& motor_task_;
        PB_SmartKnobState latest_state_ = {};
        DemoConfigChangeCallback demo_config_change_callback_;
};

