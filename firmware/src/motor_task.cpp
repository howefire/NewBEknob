#include <SimpleFOC.h>

#include "motor_task.h"
#if SENSOR_MT6701
#include "mt6701_sensor.h"
#endif
#if SENSOR_TLV
#include "tlv_sensor.h"
#endif
#include "util.h"

double MotorTask::diff(double a,double b)
{
    //计算误差值
    float max_val = std::max(target,motor.shaft_angle); 
    float min_val = std::min(target,motor.shaft_angle); 
    float diff = std::abs(max_val -min_val) ;  
    return diff;    
}
// #### 
// Hardware-specific motor calibration constants.
// Run calibration once at startup, then update these constants with the calibration results.
static const float ZERO_ELECTRICAL_OFFSET =  5.57;
static const Direction FOC_DIRECTION = Direction::CW;
static const int MOTOR_POLE_PAIRS = 7;
// ####

//死区制动百分率
static const float DEAD_ZONE_DETENT_PERCENT = 0.2;
//死区RAD
static const float DEAD_ZONE_RAD = 1 * _PI / 180;
//怠速速度ewma alpha
static const float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
//怠速速度每秒钟弧度
static const float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
//怠速修正延迟millis
static const uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
//怠速校正最大角度rad
static const float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
//怠速修正率
static const float IDLE_CORRECTION_RATE_ALPHA = 0.0005;
//新增加切换震动效果

MotorTask::MotorTask(const uint8_t task_core) : Task("Motor", 2500, 1, task_core) 
{
    queue_ = xQueueCreate(5, sizeof(Command));
    assert(queue_ != NULL);
}

MotorTask::~MotorTask() {}


#if SENSOR_TLV
    TlvSensor encoder = TlvSensor();
#elif SENSOR_MT6701
    MT6701Sensor encoder = MT6701Sensor();
#endif
TaskHandle_t myTaskHandle;

void MotorTask::run() {

    //设置驱动器电压
    
    driver.voltage_power_supply = 3.5;
    //驱动器初始化
    driver.init();
    
    #if SENSOR_TLV
    encoder.init(&Wire, false);
    #endif

    #if SENSOR_MT6701
    //初始化传感器
    encoder.init();
    #endif
    //驱动器与电机连接
    motor.linkDriver(&driver);
    //设黑控制环类型:
    motor.controller = MotionControlType::torque; 
    motor.voltage_limit = 3;
    motor.velocity_limit = 10000;
    //传感器与电机连接
    motor.linkSensor(&encoder);

    // Not actually using the velocity loop built into SimpleFOC; but I'm using those PID variables
    // to run PID for torque (and SimpleFOC studio supports updating them easily over serial for tuning)
    //pid调整qd 正反转输出为1时转矩最大，
    motor.PID_velocity.P = 4;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0.04;
    motor.PID_velocity.output_ramp = 10000;
    motor.PID_velocity.limit = 10;

    //电机初始化
    motor.init();
    encoder.update();
    delay(10);
    motor.pole_pairs = MOTOR_POLE_PAIRS;
    //校准编码器、启用FOC
    motor.initFOC(ZERO_ELECTRICAL_OFFSET, FOC_DIRECTION);
    motor.monitor_downsample = 0; // disable monitor at first - optional

    // disableCore0WDT();
    //上次相对位置
    float current_detent_center = motor.shaft_angle;
    //先设置一个默认值，等消息队列发送状态过来
    PB_SmartKnobConfig config = {
        .position = 0,
        .min_position = 0,
        .max_position = 1,
        .position_width_radians = 60 * _PI / 180,
        .detent_strength_unit = 0,
    };

    float idle_check_velocity_ewma = 0;
    uint32_t last_idle_start = 0;
    uint32_t last_publish = 0;
    while (1) {
        motor.loopFOC();
    
        // Check queue for pending requests from other tasks
        Command command;
        //检测配置的改变
        if (xQueueReceive(queue_, &command, 0) == pdTRUE) {
            switch (command.command_type) {
                case CommandType::CALIBRATE:
                    calibrate();
                    break;
                case CommandType::CONFIG: {//检查输入参数是否有错误
                    // Check new config for validity
                    if (command.data.config.detent_strength_unit < 0) {
                        log("Ignoring invalid config: detent_strength_unit cannot be negative");
                        break;
                    }
                    if (command.data.config.endstop_strength_unit < 0) {
                        log("Ignoring invalid config: endstop_strength_unit cannot be negative");
                        break;
                    }
                    if (command.data.config.snap_point < 0.5) {
                        log("Ignoring invalid config: snap_point must be >= 0.5 for stability");
                        break;
                    }
                    if (command.data.config.detent_positions_count > COUNT_OF(command.data.config.detent_positions)) {
                        log("Ignoring invalid config: detent_positions_count is too large");
                        break;
                    }
                    if (command.data.config.snap_point_bias < 0) {
                        log("Ignoring invalid config: snap_point_bias cannot be negative or there is risk of instability");
                        break;
                    }

                    // Change haptic input mode
                    PB_SmartKnobConfig newConfig = command.data.config;
                    if (newConfig.position == INT32_MIN) {
                        // INT32_MIN indicates no change to position, so restore from latest_config 没改变数值，就不管
                        log("maintaining position");
                        newConfig.position = config.position;
                    }
                    if (newConfig.position != config.position //如果点位数值和限位宽度被修改
                            || newConfig.position_width_radians != config.position_width_radians) {
                        // Only adjust the detent center if the position or width has changed
                         //调整止动中心
                        //log("adjusting detent center");
                        //将现在角度，变成新的制动点
                        current_detent_center = motor.shaft_angle;
                        #if SK_INVERT_ROTATION //开启反转后将制动点变为负角度
                            current_detent_center = -motor.shaft_angle;
                        #endif
                    }
                    config = newConfig;
                    //log("Got new config");

                    const float derivative_lower_strength = config.detent_strength_unit * 0.1;//反馈力量降低配
                    const float derivative_upper_strength = config.detent_strength_unit * 0.02;//反馈力量高配
                    const float derivative_position_width_lower = radians(3); //震动点宽度系数低配
                    const float derivative_position_width_upper = radians(8); //震动点宽度系数增高配
                    //在position_width_radians这个值里我们知道了，一个值它所需占用的角度。
                    //要在这个角度中算出这个角度的中心位置，也就是输出扭矩的位置
                    const float raw = derivative_lower_strength + (derivative_upper_strength - derivative_lower_strength)/(derivative_position_width_upper - derivative_position_width_lower)*(config.position_width_radians - derivative_position_width_lower);
                   
                    motor.PID_velocity.D = config.detent_positions_count > 0 ? 0 : CLAMP(
                        raw,
                        min(derivative_lower_strength, derivative_upper_strength),
                        max(derivative_lower_strength, derivative_upper_strength)
                    );
                    //motor_shake(3,5);
                    break;
                }
                case CommandType::HAPTIC: {
                    // Play a hardcoded haptic "click"
                    float strength = command.data.haptic.press ? 5 : 1.5;
                    //正吸
                    motor.move(strength);
                    for (uint8_t i = 0; i < 3; i++) {
                        motor.loopFOC();
                        delay(1);
                    }
                    //反吸
                    motor.move(-strength);
                    for (uint8_t i = 0; i < 3; i++) {
                        motor.loopFOC();
                        delay(1);
                    }
                    //停止吸
                    motor.move(0);
                    //motor.loopFOC();
                   // motor_shake(3,4);
                    break;
                }
               
            }
        }
        
        
        
        idle_check_velocity_ewma = motor.shaft_velocity * IDLE_VELOCITY_EWMA_ALPHA + idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
        if (fabsf(idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC) {
            last_idle_start = 0;
        } else {
            if (last_idle_start == 0) {
                last_idle_start = millis();
            }
        }
        if (last_idle_start > 0 && millis() - last_idle_start > IDLE_CORRECTION_DELAY_MILLIS && fabsf(motor.shaft_angle - current_detent_center) < IDLE_CORRECTION_MAX_ANGLE_RAD) {
            current_detent_center = motor.shaft_angle * IDLE_CORRECTION_RATE_ALPHA + current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
        }

        float angle_to_detent_center = motor.shaft_angle - current_detent_center;
        #if SK_INVERT_ROTATION //按钮反转
            angle_to_detent_center = -motor.shaft_angle - current_detent_center;
        #endif

        float snap_point_radians = config.position_width_radians * config.snap_point;
        float bias_radians = config.position_width_radians * config.snap_point_bias;
        float snap_point_radians_decrease = snap_point_radians + (config.position <= 0 ? bias_radians : -bias_radians);
        float snap_point_radians_increase = -snap_point_radians + (config.position >= 0 ? -bias_radians : bias_radians); 

        int32_t num_positions = config.max_position - config.min_position + 1;//计算出多少个吸附点
        if (angle_to_detent_center > snap_point_radians_decrease && (num_positions <= 0 || config.position > config.min_position)) 
        {
            current_detent_center += config.position_width_radians;
            angle_to_detent_center -= config.position_width_radians;
            config.position--;
        } else if (angle_to_detent_center < snap_point_radians_increase && (num_positions <= 0 || config.position < config.max_position)) {
            current_detent_center -= config.position_width_radians;
            angle_to_detent_center += config.position_width_radians;
            config.position++;
        }
         // CLAMP可以将随机变化的值限制在一个给定的区间[min,max]内
        //死区调整
        float dead_zone_adjustment = CLAMP(
            angle_to_detent_center,
            fmaxf(-config.position_width_radians*DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
            fminf(config.position_width_radians*DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD));
        //出界
        bool out_of_bounds = num_positions > 0 && ((angle_to_detent_center > 0 && config.position == config.min_position) || (angle_to_detent_center < 0 && config.position == config.max_position));
        motor.PID_velocity.limit = 10; //out_of_bounds ? 10 : 3;
        //如果越界，增大磁吸强度。
        //控制p值大，磁性强度也变大
        motor.PID_velocity.P = out_of_bounds ? config.endstop_strength_unit * 4 : config.detent_strength_unit * 4;


        // Apply motor torque based on our angle to the nearest detent (detent strength, etc is handled by the PID_velocity parameters)
        if (fabsf(motor.shaft_velocity) > 60) {
           //如果转动速度过快，不要施加扭矩，否则电流过大芯片供应不上 立马就死给看
            motor.move(0);
        } else {
            //转的不快的话，就根据上面计算的数值，给出施加扭矩的时机
            float input = -angle_to_detent_center + dead_zone_adjustment;
            //如果在界限内 
            if (!out_of_bounds && config.detent_positions_count > 0) {
                bool in_detent = false;
                for (uint8_t i = 0; i < config.detent_positions_count; i++) {
                    if (config.detent_positions[i] == config.position) {
                        in_detent = true;
                        break;
                    }
                }
                if (!in_detent) {
                    input = 0;
                }
            }
            float angle_torque=0,torque = motor.PID_velocity(input);
            #if SK_INVERT_ROTATION
                torque = -torque;
            #endif
                if(controltype == 0){
                    // snprintf(buf_, sizeof(buf_), "torque= %f input =%f",torque,input);
                    // log(buf_);
                    motor.move(torque);
                }else{
                    static double P = 0 ;//加入误差比例项P
                    if(0.2 > diff(motor.shaft_angle,target) && diff(motor.shaft_angle,target) > 0.001)
                    {
                        P = diff(motor.shaft_angle,target)*1.3; 
                    }else{
                        P=0;
                    }
                    float an_input = motor.shaft_angle - target + dead_zone_adjustment;
                    angle_torque = (abs(an_input)*0.9)+P;
                    // snprintf(buf_, sizeof(buf_), "controltype = %d an_input = %f,angle_torque = %f, target= %f,now_angle = %f,",controltype,an_input,angle_torque,target,motor.shaft_angle);
                    // log(buf_);
                    if(target<motor.shaft_angle)
                    {   
                        //log("turn left??");
                        motor.move(-angle_torque);
                     }else if(target>motor.shaft_angle)
                    {   //log("turn right??");
                        motor.move(angle_torque);
                    }   
                }
        }
        // Publish current status to other registered tasks periodically
        if (millis() - last_publish > 5) {
            publish({
                .current_position = config.position,
                .sub_position_unit = -angle_to_detent_center / config.position_width_radians,
                .has_config = true,
                .now_angle = motor.shaft_angle,
                .config = config,
               
            });
            last_publish = millis();
        }

        motor.monitor();

        delay(1);
    }
}

void MotorTask::setConfig(const PB_SmartKnobConfig& config) {
    Command command = {
        .command_type = CommandType::CONFIG,
        .data = {
            .config = config,
        }
    };
    xQueueSend(queue_, &command, portMAX_DELAY);
}


void MotorTask::playHaptic(bool press) {
    Command command = {
        .command_type = CommandType::HAPTIC,
        .data = {
            .haptic = {
                .press = press,
            },
        }
    };
    xQueueSend(queue_, &command, portMAX_DELAY);
}

void MotorTask::runCalibration() {
    Command command = {
        .command_type = CommandType::CALIBRATE,
        .data = {
            .unused = 0,
        }
    };
    xQueueSend(queue_, &command, portMAX_DELAY);
}


void MotorTask::addListener(QueueHandle_t queue) {
    listeners_.push_back(queue);
}

void MotorTask::publish(const PB_SmartKnobState& state) {
    for (auto listener : listeners_) {
        xQueueOverwrite(listener, &state);
    }
}

void MotorTask::calibrate() {
    // SimpleFOC is supposed to be able to determine this automatically (if you omit params to initFOC), but
    // it seems to have a bug (or I've misconfigured it) that gets both the offset and direction very wrong!
    // So this value is based on experimentation.
    // TODO: dig into SimpleFOC calibration and find/fix the issue

    log("\n\n\nStarting calibration, please DO NOT TOUCH MOTOR until complete!");

    motor.controller = MotionControlType::angle_openloop;
    motor.pole_pairs = 1;
    motor.initFOC(0, Direction::CW);

    float a = 0;

    // #### Determine direction motor rotates relative to angle sensor
    for (uint8_t i = 0; i < 200; i++) {
        encoder.update();
        motor.move(a);
        delay(1);
    }
    float start_sensor = encoder.getAngle();

    for (; a < 3 * _2PI; a += 0.01) {
        encoder.update();
        motor.move(a);
        delay(1);
    }

    for (uint8_t i = 0; i < 200; i++) {
        encoder.update();
        delay(1);
    }
    float end_sensor = encoder.getAngle();


    motor.voltage_limit = 0;
    motor.move(a);

    //log("");

    // TODO: check for no motor movement!

    log("Sensor measures positive for positive motor rotation:");
    if (end_sensor > start_sensor) {
        log("YES, Direction=CW");
        motor.initFOC(0, Direction::CW);
    } else {
        log("NO, Direction=CCW");
        motor.initFOC(0, Direction::CCW);
    }


    // #### Determine pole-pairs
    // Rotate 20 electrical revolutions and measure mechanical angle traveled, to calculate pole-pairs
    uint8_t electrical_revolutions = 20;
    snprintf(buf_, sizeof(buf_), "Going to measure %d electrical revolutions...", electrical_revolutions);
    log(buf_);
    motor.voltage_limit = 5;
    motor.move(a);
    log("Going to electrical zero...");
    float destination = a + _2PI;
    for (; a < destination; a += 0.03) {
        encoder.update();
        motor.move(a);
        delay(1);
    }
    log("pause..."); // Let momentum settle...
    for (uint16_t i = 0; i < 1000; i++) {
        encoder.update();
        delay(1);
    }
    log("Measuring...");

    start_sensor = motor.sensor_direction * encoder.getAngle();
    destination = a + electrical_revolutions * _2PI;
    for (; a < destination; a += 0.03) {
        encoder.update();
        motor.move(a);
        delay(1);
    }
    for (uint16_t i = 0; i < 1000; i++) {
        encoder.update();
        motor.move(a);
        delay(1);
    }
    end_sensor = motor.sensor_direction * encoder.getAngle();
    motor.voltage_limit = 0;
    motor.move(a);

    if (fabsf(motor.shaft_angle - motor.target) > 1 * PI / 180) {
        log("ERROR: motor did not reach target!");
        while(1) {}
    }

    float electrical_per_mechanical = electrical_revolutions * _2PI / (end_sensor - start_sensor);
    snprintf(buf_, sizeof(buf_), "Electrical angle / mechanical angle (i.e. pole pairs) = %.2f", electrical_per_mechanical);
    log(buf_);

    int measured_pole_pairs = (int)round(electrical_per_mechanical);
    snprintf(buf_, sizeof(buf_), "Pole pairs set to %d", measured_pole_pairs);
    log(buf_);

    delay(1000);


    // #### Determine mechanical offset to electrical zero
    // Measure mechanical angle at every electrical zero for several revolutions
    motor.voltage_limit = 5;
    motor.move(a);
    float offset_x = 0;
    float offset_y = 0;
    float destination1 = (floor(a / _2PI) + measured_pole_pairs / 2.) * _2PI;
    float destination2 = (floor(a / _2PI)) * _2PI;
    for (; a < destination1; a += 0.4) {
        motor.move(a);
        delay(100);
        for (uint8_t i = 0; i < 100; i++) {
            encoder.update();
            delay(1);
        }
        float real_electrical_angle = _normalizeAngle(a);
        float measured_electrical_angle = _normalizeAngle( (float)(motor.sensor_direction * measured_pole_pairs) * encoder.getMechanicalAngle()  - 0);

        float offset_angle = measured_electrical_angle - real_electrical_angle;
        offset_x += cosf(offset_angle);
        offset_y += sinf(offset_angle);

        snprintf(buf_, sizeof(buf_), "%.2f, %.2f, %.2f", degrees(real_electrical_angle), degrees(measured_electrical_angle), degrees(_normalizeAngle(offset_angle)));
        log(buf_);
    }
    for (; a > destination2; a -= 0.4) {
        motor.move(a);
        delay(100);
        for (uint8_t i = 0; i < 100; i++) {
            encoder.update();
            delay(1);
        }
        float real_electrical_angle = _normalizeAngle(a);
        float measured_electrical_angle = _normalizeAngle( (float)(motor.sensor_direction * measured_pole_pairs) * encoder.getMechanicalAngle()  - 0);

        float offset_angle = measured_electrical_angle - real_electrical_angle;
        offset_x += cosf(offset_angle);
        offset_y += sinf(offset_angle);

        snprintf(buf_, sizeof(buf_), "%.2f, %.2f, %.2f", degrees(real_electrical_angle), degrees(measured_electrical_angle), degrees(_normalizeAngle(offset_angle)));
        log(buf_);
    }
    motor.voltage_limit = 0;
    motor.move(a);

    float avg_offset_angle = atan2f(offset_y, offset_x);


    // #### Apply settings
    // TODO: save to non-volatile storage
    motor.pole_pairs = measured_pole_pairs;
    motor.zero_electric_angle = avg_offset_angle + _3PI_2;
    motor.voltage_limit = 5;
    motor.controller = MotionControlType::torque;

    log("\n\nRESULTS:\n  Update these constants at the top of " __FILE__);
    snprintf(buf_, sizeof(buf_), "  ZERO_ELECTRICAL_OFFSET: %.2f", motor.zero_electric_angle);
    log(buf_);
    if (motor.sensor_direction == Direction::CW) {
        log("  FOC_DIRECTION: Direction::CW");
    } else {
        log("  FOC_DIRECTION: Direction::CCW");
    }
    snprintf(buf_, sizeof(buf_), "  MOTOR_POLE_PAIRS: %d", motor.pole_pairs);
    log(buf_);
    delay(2000);
}

void MotorTask::checkSensorError() {
#if SENSOR_TLV
    if (encoder.getAndClearError()) {
        log("LOCKED!");
    }
#elif SENSOR_MT6701
    MT6701Error error = encoder.getAndClearError();
    if (error.error) {
        snprintf(buf_, sizeof(buf_), "CRC error. Received %d; calculated %d", error.received_crc, error.calculated_crc);
        log(buf_);
    }
#endif
    
}

void MotorTask::setLogger(Logger* logger) {
    logger_ = logger;
}

void MotorTask::log(const char* msg) {
    if (logger_ != nullptr) {
        logger_->log(msg);
    }
}
