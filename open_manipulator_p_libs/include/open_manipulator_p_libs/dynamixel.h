/*******************************************************************************
 * Copyright 2019 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#if defined(__OPENCR__)
#include <RobotisManipulator.h>
#include <DynamixelWorkbench.h>
#else
#include <robotis_manipulator/robotis_manipulator.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#endif

namespace dynamixel
{

#define SYNC_WRITE_HANDLER 0
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// #define CONTROL_LOOP_TIME 10;    //ms

// Protocol 2.0
#define ADDR_PRESENT_CURRENT_2 574      // 126
#define ADDR_PRESENT_VELOCITY_2 576     // 128
#define ADDR_PRESENT_POSITION_2 580     // 132
#define ADDR_VELOCITY_TRAJECTORY_2 584  // 136
#define ADDR_POSITION_TRAJECTORY_2 588  // 140
#define ADDR_PROFILE_ACCELERATION_2 556 // 108
#define ADDR_PROFILE_VELOCITY_2 560     // 112
#define ADDR_GOAL_POSITION_2 564        // 116

#define LENGTH_PRESENT_CURRENT_2 2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4
#define LENGTH_VELOCITY_TRAJECTORY_2 4
#define LENGTH_POSITION_TRAJECTORY_2 4
#define LENGTH_PROFILE_ACCELERATION_2 4
#define LENGTH_PROFILE_VELOCITY_2 4
#define LENGTH_GOAL_POSITION_2 4

    // DO not support anymore
    // Protocol 1.0
    // #define ADDR_PRESENT_CURRENT_1 = 40;
    // #define ADDR_PRESENT_VELOCITY_1 = 38;
    // #define ADDR_PRESENT_POSITION_1 = 36;

    // #define LENGTH_PRESENT_CURRENT_1 = 2;
    // #define LENGTH_PRESENT_VELOCITY_1 = 2;
    // #define LENGTH_PRESENT_POSITION_1 = 2;

    typedef struct
    {
        std::vector<uint8_t> id; // 电机ID列表（机械臂多个关节对应多个ID）
        uint8_t num;             // 电机数量
    } Joint;

    // note 基础关节电机控制，继承自 robotis_manipulator::JointActuator，实现对多个关节电机的 “基础控制”（初始化、使能、读写目标值 / 当前值）
    class JointDynamixel : public robotis_manipulator::JointActuator
    {
    private:
        DynamixelWorkbench *dynamixel_workbench_; // 指向 Dynamixel Workbench 库的实例（核心工具库，封装了电机通信细节，如串口读写、寄存器操作）
        Joint dynamixel_;                         // 存储当前控制的关节电机组信息（ID 和数量）

    public:
        JointDynamixel() {}
        virtual ~JointDynamixel() {}

        /*****************************************************************************
        ** Joint Dynamixel Control Functions
        *****************************************************************************/
        // 初始化电机：传入电机 ID 列表和配置参数（如串口名、波特率），内部调用 initialize
        virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
        // 设置电机工作模式（如位置模式、速度模式），内部调用 setOperatingMode
        virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
        // 返回当前控制的电机 ID vector 列表（用于上层模块确认控制对象）
        virtual std::vector<uint8_t> getId();

        // 使能电机（解锁电机，允许接收控制指令）；
        virtual void enable();
        // 失能电机（锁定电机，防止意外转动，保护机械结构）；
        virtual void disable();

        // TODO 下发——发送目标值到电机：传入电机 ID 和目标值（如位置、速度），内部调用 writeGoalPosition 等，返回发送是否成功。
        virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);
        // TODO 反馈——读取电机当前值：传入电机 ID，返回当前位置、速度、电流等数据（封装为 ActuatorValue 结构体），内部调用 receiveAllDynamixelValue。
        virtual std::vector<robotis_manipulator::ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);

        /*****************************************************************************
        ** Functions called in Joint Dynamixel Control Functions
        *****************************************************************************/
        // 底层初始化：配置串口（如 /dev/ttyUSB0）、波特率（如 57600），连接电机并检测是否在线。
        bool initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
        // 设置电机模式：支持 position_mode（位置控制）、velocity_mode（速度控制）等，是电机工作的前提。
        bool setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode = "position_mode");
        bool setSDKHandler(uint8_t actuator_id);
        // 写入轨迹参数：如 profile_velocity（运动速度）、profile_acceleration（加速度），控制电机运动平滑度。
        bool writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value);
        // 写入目标位置：将 “弧度值”（ROS 标准单位）转换为电机寄存器的 “脉冲值”，发送到电机。
        bool writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector);
        std::vector<robotis_manipulator::ActuatorValue> receiveAllDynamixelValue(std::vector<uint8_t> actuator_id);
    };

    // note 带轨迹规划的关节控制，继承自 robotis_manipulator::JointActuator，是 JointDynamixel 的增强版，增加了轨迹规划功能（通过 “速度 / 加速度限制” 实现平滑运动，避免电机急启急停）
    class JointDynamixelProfileControl : public robotis_manipulator::JointActuator
    {
    private:
        DynamixelWorkbench *dynamixel_workbench_;
        Joint dynamixel_;
        // unit: ms，控制周期（单位：ms，默认 10ms），用于轨迹规划的时间步长计算；
        float control_loop_time_; // unit: ms
        // 存储上一周期的目标值，用于计算当前周期的增量（实现平滑过渡）。
        std::map<uint8_t, robotis_manipulator::ActuatorValue> previous_goal_value_;

    public:
        // 构造函数，指定控制周期；
        JointDynamixelProfileControl(float control_loop_time = 0.010);
        virtual ~JointDynamixelProfileControl() {}

        /*****************************************************************************
        ** Joint Dynamixel Profile Control Functions
        *****************************************************************************/
        virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
        virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
        virtual std::vector<uint8_t> getId();

        virtual void enable();
        virtual void disable();

        // 重写父类方法，内部调用 writeGoalProfilingControlValue，根据控制周期和上一周期目标值，计算当前周期的 “平滑目标值”（避免跳变），再发送到电机。
        virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);
        virtual std::vector<robotis_manipulator::ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);

        /*****************************************************************************
        ** Functions called in Joint Dynamixel Profile Control Functions
        *****************************************************************************/
        bool initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
        bool setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode = "position_mode");
        bool setSDKHandler(uint8_t actuator_id);
        bool writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value);
        bool writeGoalProfilingControlValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);
        std::vector<robotis_manipulator::ActuatorValue> receiveAllDynamixelValue(std::vector<uint8_t> actuator_id);
    };

    // note 夹爪电机控制，继承自 robotis_manipulator::ToolActuator，专门用于 单电机夹爪（如 ROBOTIS OpenManipulator 夹爪）的控制，接口与关节控制类一致，但适配 “单电机” 特性
    class GripperDynamixel : public robotis_manipulator::ToolActuator
    {
    private:
        DynamixelWorkbench *dynamixel_workbench_;
        Joint dynamixel_;

    public:
        GripperDynamixel() {}
        virtual ~GripperDynamixel() {}

        // ps 此部分控制对象为 “单个电机”（而非多个关节），因此方法参数中 ID 为 uint8_t（单个），而非 std::vector<uint8_t>（多个）；

        /*****************************************************************************
        ** Tool Dynamixel Control Functions
        *****************************************************************************/
        virtual void init(uint8_t actuator_id, const void *arg); // 初始化夹爪电机（单个 ID），配置串口和波特率。
        virtual void setMode(const void *arg);
        virtual uint8_t getId();

        virtual void enable();
        virtual void disable();

        // 发送夹爪目标位置（如 “闭合” 对应电机转动 -90°，“打开” 对应 90°）
        virtual bool sendToolActuatorValue(robotis_manipulator::ActuatorValue value);
        // 读取夹爪当前位置（判断是否完全闭合 / 打开）
        virtual robotis_manipulator::ActuatorValue receiveToolActuatorValue();

        /*****************************************************************************
        ** Functions called in Tool Dynamixel Profile Control Functions
        *****************************************************************************/
        bool initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
        bool setOperatingMode(STRING dynamixel_mode = "position_mode");
        bool writeProfileValue(STRING profile_mode, uint32_t value);
        bool setSDKHandler();
        bool writeGoalPosition(double radian);
        double receiveDynamixelValue();
    };

} // namespace DYNAMIXEL
#endif // DYNAMIXEL_H_
