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

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#if defined(__OPENCR__)
#include <RobotisManipulator.h>
#else
// #include <robotis_manipulator/robotis_manipulator.h>
#include <../../../robotis_manipulator/include/robotis_manipulator/robotis_manipulator.h>
#endif

// #define KINEMATICS_DEBUG

using namespace Eigen;
using namespace robotis_manipulator;

namespace kinematics
{
    /**
     * @brief 逆运动学求解器类
     * Manipulator：机器人模型/状态容器（关节、连杆、工具、命名树、当前关节值/极限/姿态缓存等）。
     * Name：字符串别名（如 "gripper", "tool0"，或末端 link 名）。
     * Pose：位姿（通常包含 position (Vector3d) + orientation (Matrix3d 或 quaternion)）。
     * JointValue：关节值结构（位置/速度/加速度等分量或其一）。
     * MatrixXd：Eigen 动态矩阵。
     */

    /*****************************************************************************
    ** Kinematics Solver Using Chain Rule and Jacobian：基础方法，用标准雅可比矩阵迭代求解逆运动学
    *****************************************************************************/
    class SolverUsingCRAndJacobian : public robotis_manipulator::Kinematics
    {
    private:
        // 通过【链式法则】计算机械臂的【正运动学】
        // 正运动学：根据关节角度计算末端执行器（或指定部件）的位置和姿态；
        // 链式法则通过依次计算每个关节的变换矩阵（旋转 + 平移），并将这些矩阵相乘（从基座到末端依次传递），最终得到末端执行器相对于基座坐标系的位姿（位置 + 姿态）。
        void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
        // 使用【标准雅可比矩阵伪逆（无阻尼）法】求解【逆运动学】
        // 逆运动学：根据末端执行器的目标位姿（target_pose），计算对应的关节角度（goal_joint_value）
        /* 雅可比矩阵描述了关节速度与末端执行器速度的关系，该方法通过迭代优化：
            1、计算当前末端位姿与目标位姿的误差；
            2、利用雅可比矩阵的伪逆，将位姿误差转换为关节角度的修正量；
            3、重复修正关节角度，直到误差小于阈值或达到最大迭代次数。
            4、最终将求解得到的关节角度存入 goal_joint_value，返回求解是否成功的布尔值。
        */
        bool inverseSolverUsingJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value);

    public:
        // 初始化求解器的内部参数（如迭代精度、最大迭代次数等默认值）
        SolverUsingCRAndJacobian() {}
        // 释放求解器占用的资源（如动态分配的内存）
        virtual ~SolverUsingCRAndJacobian() {}

        // 设置求解器的参数（迭代步长、阻尼、阈值、最大迭代次数、是否只控位置等），而不是设置机械臂的结构参数
        // struct IKOption {
        //     double position_tolerance;    // 允许的末端位置误差，单位：米
        //     double orientation_tolerance; // 允许的姿态误差，单位：弧度
        //     int max_iteration;            // 迭代次数上限（防止收敛困难或目标不可达导致无限循环，设一个最大步数（例如 50、100、200））
        //     double damping;               // 阻尼系数 λ（抗奇异用）；λ 越大，数值越稳（远离奇异）；但步子也会小、可能更保守。
        //     bool only_position;           // 是否只求位置（不管姿态）；适合“先把末端移动到目标点附近，再二阶段调姿态”的任务；
        // };

        // 用法：
        // IKOption opt = {1e-4, 1e-3, 100, 0.01, false};
        // solver.setOption(&opt);

        virtual void setOption(const void *arg);
        // 计算机械臂在当前关节角度下的雅可比矩阵（6×n 或 3×n）
        virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
        // ps 对外提供的正运动学求解接口
        virtual void solveForwardKinematics(Manipulator *manipulator);
        // ps 对外提供的逆运动学求解接口
        virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value);
    };

    /*****************************************************************************
    ** Kinematics Solver Using Chain Rule and Singularity Robust Jacobian：抗奇异方法，在雅可比矩阵中加入阻尼项，避免奇异构型（雅可比矩阵不可逆）问题
    *****************************************************************************/
    class SolverUsingCRAndSRJacobian : public robotis_manipulator::Kinematics
    {
    private:
        void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
        bool inverseSolverUsingSRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value);

    public:
        SolverUsingCRAndSRJacobian() {}
        virtual ~SolverUsingCRAndSRJacobian() {}

        virtual void setOption(const void *arg);
        virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
        virtual void solveForwardKinematics(Manipulator *manipulator);
        virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value);
    };

    /*****************************************************************************
    ** Kinematics Solver Using Chain Rule and Singularity Robust Position Only Jacobian：简化版抗奇异方法，只关注末端位置（而忽略姿态）的逆解计算
    *****************************************************************************/
    class SolverUsingCRAndSRPositionOnlyJacobian : public robotis_manipulator::Kinematics
    {
    private:
        void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
        bool inverseSolverUsingPositionOnlySRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value);

    public:
        SolverUsingCRAndSRPositionOnlyJacobian() {}
        virtual ~SolverUsingCRAndSRPositionOnlyJacobian() {}

        virtual void setOption(const void *arg);
        virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
        virtual void solveForwardKinematics(Manipulator *manipulator);
        virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value);
    };

    /*****************************************************************************
    ** Kinematics Solver Customized for OpenManipulator Chain：专门针对 OpenManipulator 机械臂结构的定制方法，结合硬件特性优化求解效率
    *****************************************************************************/
    class SolverCustomizedforOMChain : public robotis_manipulator::Kinematics
    {
    private:
        void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
        bool chainCustomInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value);

    public:
        SolverCustomizedforOMChain() {}
        virtual ~SolverCustomizedforOMChain() {}

        virtual void setOption(const void *arg);
        virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
        virtual void solveForwardKinematics(Manipulator *manipulator);
        virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value);
    };

    /*****************************************************************************
    ** Kinematics Solver Using Geometry Approach：几何法（如三角法/余弦定理），常见于 3~4 DoF 的平面臂或某些 6R 结构
    *****************************************************************************/
    class SolverUsingCRAndGeometry : public robotis_manipulator::Kinematics
    {
    private:
        bool with_gripper_ = false; // 某些几何法会把夹爪厚度/工具延伸考虑进去，影响腕点位置

        void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
        bool inverseSolverUsingGeometry(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value);

    public:
        SolverUsingCRAndGeometry() {}
        virtual ~SolverUsingCRAndGeometry() {}

        virtual void setOption(const void *arg);
        virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
        virtual void solveForwardKinematics(Manipulator *manipulator);
        virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value);
    };

} // namespace KINEMATICS

#endif // KINEMATICS_H_
