#if SK_LEDS
#include <FastLED.h>
#endif

#if SK_STRAIN
#include <HX711.h>
#endif

#if SK_ALS
#include <Adafruit_VEML7700.h>
#endif
#include "BleKeyboard.h"
#include "interface_task.h"
#include "util.h"
#include <BluetoothSerial.h>
#include "bleradial.h"
#include "btAudio.h"
#include "freertos/FreeRTOS.h"
#include "btAudio.h"

#if SK_LEDS
CRGB leds[NUM_LEDS];
#endif

#if SK_STRAIN
HX711 scale;
#endif

#if SK_ALS
Adafruit_VEML7700 veml = Adafruit_VEML7700();
#endif
uint8_t pages;

BleKeyboard bleKeyboard("NBTLE","ZC",100);
//tAudio audio = btAudio("ESP_Speaker66");

 PB_SmartKnobConfig configs[] = {
   

    // int32_t position; 配置模式初始值
    // int32_t min_position;最小限位值
    // int32_t max_position;最大限位值
    // float position_width_radians;最大最小值限位点周长
    // float detent_strength_unit;反馈强度设置
    // float endstop_strength_unit;限位强度
    // float snap_point;喀啪输出时机点
    // char text[51];模式描述
    // pb_size_t detent_positions_count;
    // int32_t detent_positions[5];增加喀啪检测点
    // float snap_point_bias;
    {
        0,
        0,
        100,
        2.4 * PI / 180,
        1.3,
        3,
        1.1,
        "volume \nadjustment ",
        0,
        {},
        0,
    },
    {
        0,
        0,
        -1, // max position < min position indicates no bounds
        3.6 * PI / 180,//一圈=2π
        1.0,
        1,
        1.1,
        "UP_DOWN",
        0,
        {},
        0,
    },
    {
        0,
        0,
        -1,
        3.6 * PI / 180,
        1.0,
        1,
        1.1,
        "LIFT_RIGHT",
        0,
        {},
        0,
    },
    {
        0,
        0,
        72,
        10 * PI / 180,
        0,
        1,
        1.1,
        "Multi-rev\nNo detents ",
        0,
        {},
        0,
    },
    {
        0,
        0,
        1,
        90 * PI / 180,
        1,
        1,
        0.55, // Note the snap point is slightly past the midpoint (0.5); compare to normal detents which use a snap point *past* the next value (i.e. > 1)
        "On/off ",
        0,
        {},
        0,
    },
    {
        0,
        0,
        0,
        60 * PI / 180,
        0.01,
        0.6,
        1.1,
        "Return-to-center ",
        0,
        {},
        0,
    },
    {
        127,
        0,
        255,
        1 * PI / 180,
        0,
        1,
        1.1,
        "Fine values\nNo detents ",
        0,
        {},
        0,
    },
    {
        0,
        0,
        31,
        8.225806452 * PI / 180,
        2,
        1,
        1.1,
        "Coarse values\nStrong detents ",
        0,
        {},
        0,
    },
    {
        0,
        0,
        31,
        8.225806452 * PI / 180,
        0.2,
        1,
        1.1,
        "Coarse values\nWeak detents ",
        0,
        {},
        0,
    },
    {
        0,
        0,
        31,
        7 * PI / 180,
        2.5,
        1,
        0.7,
        "Magnetic detents ",
        4,
        {2, 10, 21, 22},
        0,
    },
    {
        0,
        -6,
        6,
        60 * PI / 180,
        1,
        1,
        0.55,
        "Return-to-center\nwith detents ",
        0,
        {},
        0.4
    },
};



//构造初始化运行
InterfaceTask::InterfaceTask(const uint8_t task_core, MotorTask& motor_task, DisplayTask* display_task) : 
        Task("Interface", 6000, 1, task_core),
        stream_(),
        motor_task_(motor_task),
        display_task_(display_task),
        plaintext_protocol_(stream_, motor_task_),//赋给通信协议使用stream_，motor_task_类能力
        Json_protocol_(stream_, motor_task_),
        proto_protocol_(stream_, motor_task_) {
    #if SK_DISPLAY
        assert(display_task != nullptr);
    #endif

    log_queue_ = xQueueCreate(10, sizeof(std::string *));
    assert(log_queue_ != NULL);

    knob_state_queue_ = xQueueCreate(1, sizeof(PB_SmartKnobState));
    assert(knob_state_queue_ != NULL);
}

void InterfaceTask::run() {

    stream_.begin();
   
    //stream_.println("Starting BLE work!");
    #if SK_LEDS
        FastLED.addLeds<SK6812, PIN_LED_DATA, GRB>(leds, NUM_LEDS);
    #endif

    #if SK_ALS && PIN_SDA >= 0 && PIN_SCL >= 0
        Wire.begin(PIN_SDA, PIN_SCL);
        Wire.setClock(400000);
    #endif
    #if SK_STRAIN
        scale.begin(38, 2);
    #endif

    #if SK_ALS
        if (veml.begin()) {
            veml.setGain(VEML7700_GAIN_2);
            veml.setIntegrationTime(VEML7700_IT_400MS);
        } else {
            log("ALS sensor not found!");
        }
    #endif

    motor_task_.setConfig(configs[0]);
    motor_task_.addListener(knob_state_queue_);
  
    //bleKeyboard.begin();
    // audio.begin();
    // audio.reconnect();
    // Start in legacy protocol mode
    //
    plaintext_protocol_.init([this] () {//使用lambada表达式，实际上传递InterfaceTask本类地址
        changeConfig(true);//在SerialProtocolPlaintext类中init()方法中调用changeConfig
    });
    //将现在协议，指向SerialProtocolPlaintext协议、
    SerialProtocol* current_protocol = &proto_protocol_;
    //判断current_protocol通信协议为哪个
    //[this, &current_protocol]可以访问的列表，本类，当前协议指针，指定协议类型，
    //也就是说调用该方法，传入protocol值，即可修改使用的协议对象
    ProtocolChangeCallback protocol_change_callback = [this, &current_protocol] (uint8_t protocol) 
    {
        switch (protocol) {
            case SERIAL_PROTOCOL_LEGACY:
                current_protocol = &plaintext_protocol_;
                log("Switch to legacy protocol");
                break;
            case SERIAL_PROTOCOL_PROTO:
                current_protocol = &proto_protocol_;
                 log("Switch to proto protocol");
                break;
            case SERIAL_PROTOCOL_JSON:
                current_protocol = &Json_protocol_;
                 log("Switch to proto Json_protocol_");
            default:
                log("Unknown protocol requested");
                break;
        }
    };
    //启用修改后的协议
    plaintext_protocol_.setProtocolChangeCallback(protocol_change_callback);
    proto_protocol_.setProtocolChangeCallback(protocol_change_callback);
    
     //uint32_t last_idle_start = 0;
    uint32_t last_publish = 0;

    while (1) {
        PB_SmartKnobState state;
      
        //主动上传
        if (xQueueReceive(knob_state_queue_, &state, 0) == pdTRUE  ) {
            //
            current_protocol->handleState(state);
        }
        current_protocol->loop();

        std::string* log_string;
        while (xQueueReceive(log_queue_, &log_string, 0) == pdTRUE) {
            current_protocol->log(log_string->c_str());
            delete log_string;
      
        }

        updateHardware(&state);
        // if (millis() - last_publish > 500) {
        //     char *buf = (char*)malloc(40);
        //     // vTaskGetRunTimeStats(buf);
        //     // log(buf);
        //     snprintf(buf_, sizeof(buf_), "esp_get_free_heap_size %d ",esp_get_free_heap_size());
        //     log(buf_);
        //     free(buf);
        //     last_publish = millis();
        // }
        
        
    }
}

void InterfaceTask::log(const char* msg) {
    // Allocate a string for the duration it's in the queue; it is free'd by the queue consumer
    std::string* msg_str = new std::string(msg);

    // Put string in queue (or drop if full to avoid blocking)
    xQueueSendToBack(log_queue_, &msg_str, 0);
}


void InterfaceTask::changeConfig(bool next) {
    if (next) {
        current_config_ = (current_config_ + 1) % COUNT_OF(configs);
    } else {
        if (current_config_ == 0) {
            current_config_ = COUNT_OF(configs) - 1;
        } else {
            current_config_ --;
        }
    }
    pages = current_config_;
    char buf_[256];
    //snprintf(buf_, sizeof(buf_), "Changing config to %d -- %s", current_config_, configs[current_config_].text);
    //log(buf_);
    motor_task_.setConfig(configs[current_config_]);
}


float cur_positon;
BleRadialInput dial;
void InterfaceTask::updateHardware(PB_SmartKnobState* stas) {
    // How far button is pressed, in range [0, 1]
     float press_value_unit = 0;
    
        #if SK_ALS
        const float LUX_ALPHA = 0.005;
        static float lux_avg;
        float lux = veml.readLux();
        lux_avg = lux * LUX_ALPHA + lux_avg * (1 - LUX_ALPHA);
        static uint32_t last_als;
        if (millis() - last_als > 1000) {
            snprintf(buf_, sizeof(buf_), "millilux: %.2f", lux*1000);
            log(buf_);
            last_als = millis();
        }
    #endif

    #if SK_STRAIN
        if (scale.wait_ready_timeout(100)) {
            int32_t reading = scale.read();
            //stream_.println(reading);
            static uint32_t last_reading_display;
            if (millis() - last_reading_display > 1000) {
                // snprintf(buf_, sizeof(buf_), "HX711 reading: %d", reading);
                // log(buf_);
                last_reading_display = millis();
            }

            // TODO: calibrate and track (long term moving average) zero point (lower); allow calibration of set point offset
            const int32_t lower = -500000;
            const int32_t upper = 1000000;
            // Ignore readings that are way out of expected bounds
            if (reading >= lower - (upper - lower) && reading < upper + (upper - lower)*2) {
                long value = CLAMP(reading, lower, upper);
                press_value_unit = 1. * (value - lower) / (upper - lower);
            
                static bool pressed;
                static uint8_t press_count;
            //    stream_.println(cur_positon);

                if ( press_value_unit >0.1) {
                   // 
                    press_count++;
                   
                    if(press_count==1){
                        cur_positon= motor_task_.motor.shaft_angle; //按下按键时记录当前的旋钮角度
                        motor_task_.playHaptic(true); 
                    }
                    if (press_count ==30) {
                        //如果移动角度大于0.1
                      if( motor_task_.motor.shaft_angle-cur_positon>0.11){
                          //dial.sendValue();
                            changeConfig(true);
                            motor_task_.playHaptic(true);
                        }else if( motor_task_.motor.shaft_angle-cur_positon<-0.11)
                        {
                            motor_task_.playHaptic(true);                            
                            changeConfig(false);
                        
                        }
                    }
                }else{
                        cur_positon = motor_task_.motor.shaft_angle;
                        press_count =0;
                }

    
            }
        } else {
            log("HX711 not found.");

            #if SK_LEDS
                for (uint8_t i = 0; i < NUM_LEDS; i++) {
                    leds[i] = CRGB::Red;
                }
                FastLED.show(); 
            #endif
        }
    #endif
   
    uint16_t brightness = UINT16_MAX;
    // TODO: brightness scale factor should be configurable (depends on reflectivity of surface)
    #if SK_ALS
        brightness = (uint16_t)CLAMP(lux_avg * 13000, (float)1280, (float)UINT16_MAX);
    #endif

    #if SK_DISPLAY
        display_task_->setBrightness(brightness); // TODO: apply gamma correction
    #endif

    #if SK_LEDS
        for (uint8_t i = 0; i < NUM_LEDS; i++) {
            leds[i].setHSV(200 * press_value_unit, 255, brightness >> 8);

            // Gamma adjustment
            leds[i].r = dim8_video(leds[i].r);
            leds[i].g = dim8_video(leds[i].g);
            leds[i].b = dim8_video(leds[i].b);
        }
        FastLED.show();
    #endif
}
