/*
   Copyright 2020 Scott Bezek and the splitflap contributors

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#pragma once

#include<Arduino.h>

// Static polymorphic abstract base class for a FreeRTOS task using CRTP pattern. Concrete implementations
// should implement a run() method.
// Inspired by https://fjrg76.wordpress.com/2018/05/23/objectifying-task-creation-in-freertos-ii/
//泛型模版 奇异递归 吊！

template<class T>
class Task {
    public:
        Task(const char* name, uint32_t stackDepth, UBaseType_t priority, const BaseType_t coreId = tskNO_AFFINITY) : 
                name { name },
                stackDepth {stackDepth},
                priority { priority },
                coreId { coreId }
        {}
        virtual ~Task() {};

        TaskHandle_t getHandle() {
            return taskHandle;
        }
        //main 调用这里创建线程，taskFunction 运行该静态函数
        void begin() {
            BaseType_t result = xTaskCreatePinnedToCore(taskFunction, name, stackDepth, this, priority, &taskHandle, coreId);
            assert("Failed to create task" && result == pdPASS);
        }
        
    private:
        //使用模板函数类型，将给过来的类，类型，转为指针，然后调用该指向该类指针里的run函数
        static void taskFunction(void* params) {
            T* t = static_cast<T*>(params);
            t->run();
        }

        const char* name;
        uint32_t stackDepth;
        UBaseType_t priority;
        TaskHandle_t taskHandle;
        const BaseType_t coreId;
};
