#include <Arduino.h>

#include "display_task.h"
#include "interface_task.h"
#include "motor_task.h"

#include "btAudio.h"

#if SK_DISPLAY
static DisplayTask display_task(0);
static DisplayTask* display_task_p = &display_task;
#else
static DisplayTask* display_task_p = nullptr;
#endif
MotorTask motor_task(1);
btAudio audio = btAudio("ESP_Speaker66");
InterfaceTask interface_task(0, motor_task, display_task_p);//实例化类
Stream& stream = Serial;

void setup() {
  audio.begin();
  audio.reconnect();
  display_task.setLogger(&interface_task);
  display_task.begin();//模版直接被运行，创建任务
  // Connect display to motor_task's knob state feed
  motor_task.addListener(display_task.getKnobStateQueue());
  
  motor_task.setLogger(&interface_task);
  motor_task.begin();
 
  interface_task.begin();
  
  // Free up the Arduino loop task
  vTaskDelete(NULL);
}

void loop() {
  // char buf[50];
  // static uint32_t last_stack_debug;
  // if (millis() - last_stack_debug > 1000) {
  //   interface_task.log("Stack high water:");
  //   snprintf(buf, sizeof(buf), "  main: %d", uxTaskGetStackHighWaterMark(NULL));
  //   interface_task.log(buf);
  //   #if SK_DISPLAY
  //     snprintf(buf, sizeof(buf), "  display: %d", uxTaskGetStackHighWaterMark(display_task.getHandle()));
  //     interface_task.log(buf);
  //   #endif
  //   snprintf(buf, sizeof(buf), "  motor: %d", uxTaskGetStackHighWaterMark(motor_task.getHandle()));
  //   interface_task.log(buf);
  //   snprintf(buf, sizeof(buf), "  interface: %d", uxTaskGetStackHighWaterMark(interface_task.getHandle()));
  //   interface_task.log(buf);
  //   snprintf(buf, sizeof(buf), "Heap -- free: %d, largest: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  //   interface_task.log(buf);
  //   last_stack_debug = millis();
  // }
}