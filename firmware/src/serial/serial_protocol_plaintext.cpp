#include "../proto_gen/smartknob.pb.h"

#include "serial_protocol_plaintext.h"
#include <typeinfo>
#include <stdio.h> 
#include <stdint.h>
#include <math.h>
#include "mt6701_sensor.h"
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo>
#include <iostream>
#include <Arduino.h>

//  #define my_debug2
System_run_state runstate;
System_run_state *prunstate = &runstate;
uint8_t *p = (uint8_t*)prunstate; 
Reterns_float s1,s2,s3,s4;
extern uint8_t pages;

uint8_t SerialProtocolPlaintext::crc8_MAXIM(uint8_t *temp_data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;

    while(len--)
    {
        crc ^= *temp_data++;
        for(i = 0;i < 8;i++)
        {
            if(crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8c;
            }
                else crc >>= 1;
        }
    }
    #ifdef my_debug3
        stream_.println(crc); 
    #endif
    return crc;
}


// void SendBTLE(BleKeyboard* bleKeyboard)
// {

//        static PB_SmartKnobState latest_state_ = {};
//        
//         latest_state_ = state;
// }

void SerialProtocolPlaintext::handleState(const PB_SmartKnobState& state) {
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


void SerialProtocolPlaintext::log(const char* msg) {
    //stream_.print("LOG: ");
    stream_.println(msg);
}

void SerialProtocolPlaintext::send_system_state(uint8_t st,uint8_t end,PB_SmartKnobState& state)
{
    
    #ifdef my_debug2
            stream_.println(state.current_position); 
    #endif
    //转换数据将Float转换成HEX
    s1.org_float = latest_state_.config.detent_strength_unit;
    s2.org_float = motor_task_.motor.shaft_velocity;
    s3.org_float = motor_task_.motor.shaftAngle();
    
    runstate.current_mode = motor_task_.motor.controller;
    runstate.current_power_H = s1.after_uchar[3];
    runstate.current_power_L = s1.after_uchar[2];
    runstate.current_speed_H = s2.after_uchar[3];
    runstate.current_speed_L = s2.after_uchar[2];
    runstate.current_location_H = s3.after_uchar[3];
    runstate.current_location_L = s3.after_uchar[2];
    runstate.current_data_H =  state.current_position>>8 ;
    runstate.current_data_L =  state.current_position ;
    runstate.current_temp = 0;
    runstate.current_worngcode = 0;

    //组织发送数据；
    sned_len = 5+end-start;
    send_buf[0] = 0x8C;
    send_buf[1] = 0x01; 
    send_buf[2] = 0x85;
    send_buf[3] = sned_len;
    send_buf[4] = output_io;
    uint8_t temp=5;
    //截取指定数据发送.
    for (uint8_t i=start;i<=end;i++){
        
        send_buf[temp] = *(p+i);
        temp ++;
    }
   
    send_buf[sned_len+1]=crc8_MAXIM(send_buf,sned_len+1);
    stream_.write(send_buf,sned_len+2);

}



void SerialProtocolPlaintext::esp32_serial_send(uint8_t *temp_data, uint8_t length)
{ 
    #ifdef my_debug
            stream_.println("esp32_serial_send"); 
    #endif
    //组织数据 
    uint8_t *packet = (uint8_t*)malloc(length);
    packet[0] = 0x8C;
    packet[1] = 0x01;
    packet[2] = 0x80 | Recv_buf[2]; 

    
    memcpy(packet+3, temp_data+3, length-3);
    
    //发送数据
    stream_.write(packet,length-1);
    stream_.write(crc8_MAXIM(packet,length));

    //释放内存
    free(packet);
}
extern PB_SmartKnobConfig configs[];



union Data { uint32_t i; float f; };

float hex_to_float(uint32_t hex) 
{
    union Data data;
    data.i = hex; 
    return data.f;
     
}



//组合数据
void SerialProtocolPlaintext::Adjust_SmartKnobConfig(uint8_t Specific_Mode,uint8_t Specific_data,uint8_t datalen)  
{

    //将接收参数组合
   switch (datalen-8)
   {
    case 0x01:
        temp_data = Recv_buf[7] & 0x000000ff;
    break;
    case 0x02:
        temp_data = (Recv_buf[7] <<8  | Recv_buf[8]) & 0x0000ffff;
    break; 
    case 0x03:
        temp_data = (Recv_buf[7] <<16  | Recv_buf[8]<<8 | Recv_buf[9] ) & 0x00ffffff;
    break;
    case 0x04:
        temp_data = (Recv_buf[7] <<24 | Recv_buf[8]<<16 | Recv_buf[9]<<8 | Recv_buf[10] ) & 0xffffffff;
    break;
    default:
        if(datalen>4)//如果数据长度大于4字节，说明是数组
        memcpy(num_data,&Recv_buf[8],datalen);
    break;
   }
    //有数据就配置
    PB_SmartKnobConfig* myc = &configs[Specific_Mode];
    uint8_t **temp_buf;
    //修改指定数值
     switch (Specific_data)
        {
            case 0x00://配置模式初始值
                myc->position = temp_data;
                #ifdef my_debug
                printspect<int32_t>(myc,temp_data);
                #endif
            case 0x01://最小限位值
                myc->min_position = temp_data;
                #ifdef my_debug
                printspect<int32_t>(myc,temp_data);
                #endif
            break;  
            case 0x02://最大限位值
                myc->max_position = temp_data;
                #ifdef my_debug
                printspect<int32_t>(myc,temp_data);
                #endif
            break;  
            case 0x03://最大最小值限位点周长
                myc->position_width_radians = hex_to_float(temp_data);
                #ifdef my_debug
                printspect<float>(myc,temp_data);
                #endif
            break;  
            case 0x04://强度设置
                myc->detent_strength_unit = hex_to_float(temp_data);
                #ifdef my_debug
                printspect<float>(myc,temp_data);
                #endif
            break;
            case 0x05://限位强度
                myc->endstop_strength_unit = hex_to_float(temp_data);
            break;
            case 0x06://喀啪输出时机点
                myc->snap_point = hex_to_float(temp_data);
            break;
            case 0x07://模式描述
                memset(myc->text,0,sizeof(myc->text));//先清空
                for(uint8_t i=0;i<=rec_len-8;i++){
                    myc->text[i] = num_data[i];//在写入修改的数据
                }
            case 0x08:
                myc->detent_positions_count = temp_data;
            break;
            case 0x09://增加喀啪检测点
                memset(myc->text,0,sizeof(myc->text));//先清空
                for(uint8_t i=0;i<=rec_len-8;i++){
                    myc->text[i] = num_data[i];//在写入修改的数据
                }        
            break;
            case 0x0a:
                myc->snap_point_bias  = hex_to_float(temp_data);
            break;
            case 0x0b:
                *temp_buf = (uint8_t *)(&configs[Specific_Mode]);
                stream_.write(*temp_buf,sizeof(configs[Specific_Mode]));
            break;
            default:
            break;
        }
 
    motor_task_.setConfig(*myc);
    
}

void SerialProtocolPlaintext::loop() 
{
//   do
//     {
//      rec_len = stream_.available(); 
//       if(rec_len!=0 ){
//          for(uint8_t i=0;i<rec_len;i++){ 
//                 Recv_buf[i] = stream_.read();
//          } 
//     }
//     } while (stream_.available());//确定没有数据

    
 // = (char *)malloc(2046); 
 //= (char *)malloc(1024);

        // motor_task_.motor_task_.motor.monitor();
        // motor_task_.motor_task_.motor.disable();



//     //帧头帧尾数据长度校验
    // if(rec_len-1 == Recv_buf[3] && Recv_buf[0]== 0xc8 && Recv_buf[rec_len-1] == crc8_MAXIM(Recv_buf,rec_len-1))//处理数据流
    // {  
            
    //     #ifdef my_debug
    //     stream_.println("into annalzy chuli"); 
    //     #endif 

    //     switch (Recv_buf[2])
    //     {
    //         case 0x05://读取寄存操作
    //             switch (Recv_buf[4])
    //             {
    //                 case  0x01:
    //                    send_system_state(Recv_buf[5],Recv_buf[6],latest_state_);//发送系统运行状态
    //                 break;                                   
    //                 default:
    //                 break;
    //                 //发送反馈帧
    //                 esp32_serial_send(Recv_buf,rec_len);
    //             }
        
    //         break;
            
    //         case 0x06://写入修改寄存器操作
    //             switch (Recv_buf[4])
    //             {
    //                 case 0x02://写入运行寄存器

    //                     switch (Recv_buf[5])
    //                     {   case 0x0a:
    //                         //限制起始位，与结束位
    //                             if(rec_len <= 9 &&  0x0B > Recv_buf[7] && Recv_buf[6]< Recv_buf[7]){
    //                                 output_io =true;
    //                                 start = Recv_buf[6];
    //                                 end = Recv_buf[7];
    //                             } 
    //                         break;
    //                          case 0x0b:
                
    //                                 output_io = false;
    //                         break;
    //                         case 0x0c:
    //                             motor_task_.runCalibration();
    //                         break;
                           
    //                         default:
    //                          //切换模式&调整模式参数
                                 //Adjust_SmartKnobConfig(Recv_buf[5],Recv_buf[6],rec_len);
    //                         break;
    //                     }
    //                     esp32_serial_send(Recv_buf,rec_len);
    //                 break;
    //             default:
    //             break;
    //             }
                        
    //         break; 
    //     default:
    //     break;      
               
    //     }
              
    //     }
    //     rec_len =0; 
       
    
}



void SerialProtocolPlaintext::changeConfigtoSpecific(uint8_t mode) 
{
   
    if (0<=mode && 11>mode) {
        motor_task_.setConfig(configs[mode]);
    } else {
        log("no such modetype");
    }  
    
}
void SerialProtocolPlaintext:: init(DemoConfigChangeCallback cb) {
    demo_config_change_callback_ = cb;
    stream_.println("SmartKnob starting!\n\nSerial mode: plaintext\nPress 'C' at any time to calibrate.\nPress <Space> to change haptic modes.");
}
