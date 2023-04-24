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
        -1,
        1 * PI / 180,
        0,
        0,
        1.1,
        "angle_contorl_mode ",
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
// void SerialProtocolJson::log(const char* msg) {
//     //stream_.print("LOG: ");
//     stream_.println(msg);
// }

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


void SerialProtocolJson::Json_Analyze_func(cJSON* root)
{

 if(root ==NULL){
    return;
   }else{

        cJSON* spmode = cJSON_GetObjectItem(root,"mode");  
        if(strcmp(spmode->valuestring, "angle") == 0)
        {//切换显示
            motor_task_.setConfig(motor_configs[0]);
            motor_task_.controltype = 2 ;
            cJSON* type = cJSON_GetObjectItem(root,"type");
            cJSON* data = cJSON_GetObjectItem(root,"data");
            if (strcmp(type->valuestring, "set") == 0)
            {
               cJSON* value = cJSON_GetObjectItem(data,"value");
               stream_.printf("data = %lf\r\n",value->valuedouble);
               motor_task_.target = (float)value->valuedouble;
               
            
             }
        }else if(strcmp(spmode->valuestring, "angle") == 0)
        {
            



        }
        // cJSON* spdata = cJSON_GetObjectItem(root,"type");
        // cJSON* doubledata  =  cJSON_GetObjectItem(root,"double");
        // cJSON* intdata  =  cJSON_GetObjectItem(root,"int");
        // cJSON* arry  =  cJSON_GetObjectItem(root,"Array");
        // cJSON* angle = cJSON_GetObjectItem(root,"angle");
        // if(spmode)
        // {
        //     doubledata = NULL; //互斥锁确保一次只能对一个数据进行修改
        //     intdata = NULL;
        //     angle =   NULL;
        //     // for(uint16_t i =0;i<cJSON_GetArraySize(arry);i++){chardata[i] = cJSON_GetArrayItem(arry,i)->valueint;}
        //     // stream_.printf("arry = %s",chardata);
        //     strle=strlen(arry->valuestring);
        //     Json_Adjust_SmartKnobConfig<char*>(spmode->valueint,spdata->valueint,(char*)arry->valuestring);
        // }
        // if(doubledata){//如果是浮点
        //     arry = NULL; //互斥锁确保一次只能对一个数据进行修改
        //     intdata = NULL;
        //     angle =   NULL;
        //     //stream_.printf("doubldata =%s type =%d",cJSON_Print(doubledata),doubledata->type);
        //     Json_Adjust_SmartKnobConfig<float*>(spmode->valueint,spdata->valueint,(float*)&doubledata->valuedouble);
        // }
        // if(intdata){//如果是整形数字
        //     arry = NULL; //互斥锁确保只能对一个数据进行修改
        //     doubledata = NULL;
        //     angle =   NULL;
        //     //stream_.printf("spmode = %d spdata=%d intdata =%d type =%d",spmode->valueint,
        //     //spdata->valueint,intdata->valueint,intdata->type);
            
        //     Json_Adjust_SmartKnobConfig<int32_t*>(spmode->valueint,spdata->valueint,(int32_t*)&intdata->valueint);
        // }     
        // if(angle){
           
              
        // }
   
   
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

