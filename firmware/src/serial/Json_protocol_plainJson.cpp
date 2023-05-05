#include "Json_protocol_plainJson.h"
#include <typeinfo>
#include <stdio.h> 
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo>
#include <iostream>
#include <Arduino.h>
extern float target;
uint8_t strle;

 PB_SmartKnobConfig motor_configs[] = {
   
    {
        0,
        0,
        -1,
        1 * PI / 180,
        0,
        0,
        1.1,
        "nangle_mode",
        0,
        {},
        0,
    },
    };
extern uint8_t pages;
extern PB_SmartKnobConfig configs[];
void SerialProtocolJson::handleState(const PB_SmartKnobState& state) {
    bool substantial_change = (latest_state_.current_position != state.current_position)
        || (latest_state_.config.detent_strength_unit != state.config.detent_strength_unit)
        || (latest_state_.config.endstop_strength_unit != state.config.endstop_strength_unit)
        || (latest_state_.config.min_position != state.config.min_position)
        || (latest_state_.config.max_position != state.config.max_position);
    
    #ifdef my_debug1
        stream_.println(latest_state_.current_position); 
    #endif
            
    if (substantial_change && output_io) {
        
        if( pages == 0 ){
            
            if(state.current_position > latest_state_.current_position){
                //bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
                log("VOLUME_up");
            }else{
                //bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
                log("VOLUME_DOWN");
            }
        }


        if (pages == 1 )
        {
            if(state.current_position > latest_state_.current_position){
                //bleKeyboard.write(KEY_UP_ARROW);
                log("KEY_up");
            }else{
                //bleKeyboard.write(KEY_DOWN_ARROW);
                log("KEY_DOWN");
            }

        }   

          if (pages == 2 )
        {
            if(state.current_position > latest_state_.current_position){
                //bleKeyboard.write(KEY_RIGHT_ARROW);
                log("KEY_RIGHT");
            }else{
               // bleKeyboard.write(KEY_LEFT_ARROW);
               // log("KEY_LEFT");
            }

        }   

            //send_system_state(start,end,&state);
    
    }
    latest_state_ = state;
}

void SerialProtocolJson::log(const char* msg) {
    //stream_.print("LOG: ");
    stream_.println(msg);
}

template<typename T>
void SerialProtocolJson::Json_Adjust_SmartKnobConfig(uint8_t Specific_Mode,uint8_t Specific_data,T data)  
{
    stream_.printf("Specific_Mode = %d Specific_data = %d\r\n",Specific_Mode,Specific_data);
    log("no such modetype");
    //有数据就配置
    PB_SmartKnobConfig* myc = configs[Specific_Mode];
    //修改指定数值
     switch (Specific_data)
        {
            case 0x00://配置模式初始值
                myc->position = *data;
            case 0x01://最小限位值
                myc->min_position = *data;
                #ifdef my_debug
                printspect<int32_t>(myc,temp_data);
                #endif
            break;  
            case 0x02://最大限位值
                myc->max_position = *data;
            break;  
            case 0x03://最大最小值限位点周长
                myc->position_width_radians = *data;
            break;  
            case 0x04://强度设置
                myc->detent_strength_unit = *data;
            break;
            case 0x05://限位强度
                myc->endstop_strength_unit = *data;
            break;
            case 0x06://喀啪输出时机点
                myc->snap_point = *data;
            break;
            case 0x07://模式描述
                memset(myc->text,0,sizeof(myc->text));//先清空
               // stream_.printf("data = %s\r\n",data);
                 for(uint8_t i=1;i<=strle;i++)
                    myc->text[i-1] = data[i];//在写入修改的数据
            break;
            case 0x08:
                myc->detent_positions_count = *data;
            break;
            case 0x09://增加喀啪检测点
                memset(myc->text,0,sizeof(myc->text));//先清空
                for(uint8_t i=0;i<=sizeof(data);i++){
                    myc->text[i] = data[i];//在写入修改的数据
                }        
            break;
            case 0x0a:
                myc->snap_point_bias  = *data;
            break;
            default:
               return;
            break;
        }
 
    motor_task_.setConfig(*myc);
    
}

extern uint8_t sw ;
void SerialProtocolJson::Json_Analyze_func(cJSON* root)
{

 if(root ==NULL){
    return;
   }else{
        cJSON* mode = cJSON_GetObjectItem(root,"mode");
        cJSON* data = cJSON_GetObjectItem(root,"data");
        cJSON* type = cJSON_GetObjectItem(root,"type");
        if(strcmp(mode->valuestring, "setEventFeedback") == 0){
        cJSON* serialDataType = cJSON_GetObjectItem(data,"serialDataType");
           if (strcmp(serialDataType->valuestring, "bin") == 0)
            {
                sw = 1;
            }else{
                sw = 2;
            }
        }else if(strcmp(mode->valuestring, "angle") == 0)
        {//切换显示
            motor_task_.setConfig(motor_configs[0]);
            //切换控制
            motor_task_.controltype = 2 ;
            if (strcmp(type->valuestring, "set") == 0)
            {
               cJSON* value = cJSON_GetObjectItem(data,"value");
               stream_.printf("data = %lf\r\n",value->valuedouble);
               motor_task_.target = (float)value->valuedouble;
            }
        } else if(strcmp(data->valuestring, "promise") == 0)
        {
            cJSON* switchType = cJSON_GetObjectItem(data,"switchType");
            PB_SmartKnobConfig* myc = &configs[4];
             
            if(strcmp(switchType->valuestring, "selfLocking")==0)
            {
              myc->snap_point_bias= 0.6f;
              myc->snap_point=0.75f;
              //myc->endstop_strength_unit = (float)torque->valuedouble;
            }
        } else if(strcmp(data->valuestring, "limit") == 0)
        {
            cJSON* torque = cJSON_GetObjectItem(data,"torque");
            cJSON* switchType = cJSON_GetObjectItem(data,"switchType");
            PB_SmartKnobConfig* myc = &configs[4];
             
            if(strcmp(switchType->valuestring, "selfLocking")==0){
              stream_.println("cash3");
              myc->snap_point_bias= 0.6f;
              stream_.println("cash4");
              myc->snap_point=0.75f;
              //myc->endstop_strength_unit = (float)torque->valuedouble;
            }
        }
   }

}


void SerialProtocolJson ::loop()
{


  if ( stream_.available() > 0) {
    cJSON *myroot = NULL;
    // 读取串口数据
    int len = stream_.readBytesUntil('\n',recvdata,stream_.available());
    recvdata[len]='\0';
    myroot = cJSON_Parse(recvdata);
    Json_Analyze_func(myroot);
    myroot = NULL;
    cJSON_Delete(myroot);
   
  }

}

