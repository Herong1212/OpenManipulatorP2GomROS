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

#include "../include/open_manipulator_p_libs/kinematics.h"

using namespace robotis_manipulator;
using namespace kinematics;

/*****************************************************************************
** Kinematics Solver Using Chain Rule and Jacobian
*****************************************************************************/
void SolverUsingCRAndJacobian::setOption(const void *arg) {}

Eigen::MatrixXd SolverUsingCRAndJacobian::jacobian(Manipulator *manipulator, Name tool_name)
{
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

    Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

    Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
    Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

    //////////////////////////////////////////////////////////////////////////////////

    int8_t index = 0;
    Name my_name = manipulator->getWorldChildName();

    for (int8_t size = 0; size < manipulator->getDOF(); size++)
    {
        Name parent_name = manipulator->getComponentParentName(my_name);
        if (parent_name == manipulator->getWorldName())
        {
            joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
        }
        else
        {
            joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
        }

        position_changed = math::skewSymmetricMatrix(joint_axis) *
                           (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
        orientation_changed = joint_axis;

        pose_changed << position_changed(0),
            position_changed(1),
            position_changed(2),
            orientation_changed(0),
            orientation_changed(1),
            orientation_changed(2);

        jacobian.col(index) = pose_changed;
        index++;
        my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
    }
    return jacobian;
}

// ps 正运动学的对外接口--1
void SolverUsingCRAndJacobian::solveForwardKinematics(Manipulator *manipulator)
{
    forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

// ps 逆运动学的对外接口--1
bool SolverUsingCRAndJacobian::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
    return inverseSolverUsingJacobian(manipulator, tool_name, target_pose, goal_joint_value);
}

// * private--1
/**
 * @brief 使用【链式法则】进行前向求解，计算机械臂组件在世界坐标系下的位姿、速度和加速度。
 *
 * 该函数通过递归方式遍历机械臂的组件树结构，从根节点开始逐级向下计算每个组件的绝对位姿，
 * 并将结果存储到 Manipulator 对象中。此过程基于父组件的位姿以及相对变换关系（包括关节旋转）来推导子组件的位姿。
 *
 * @param manipulator 指向 Manipulator 类实例的指针，用于获取组件之间的连接关系及运动学参数。
 * @param component_name 当前处理的组件名称。
 */
void SolverUsingCRAndJacobian::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
    Name my_name = component_name;                                               // 保存当前要计算的“这个组件”的名字（关节/连杆/末端），后面方便多次使用。
    Name parent_name = manipulator->getComponentParentName(my_name);             // 获取当前组件的父组件名称
    int8_t number_of_child = manipulator->getComponentChildName(my_name).size(); // 获取当前组件的子组件数量

    Pose parent_pose_value; // 存放父节点的世界位姿
    Pose my_pose_value;     // 存放当前组件的位姿

    // Get Parent Pose
    // case1 ：当前组件的父组件是是基座 ——> 父位姿直接用世界位姿（一般是单位旋转+原点）
    if (parent_name == manipulator->getWorldName())
    {
        // 直接拿世界位姿（通常是单位姿态）
        parent_pose_value = manipulator->getWorldPose();
    }
    // case2 ：当前组件的父组件不是基座 ——> 去模型里拿父组件相对于世界坐标系下的位姿
    else
    {
        // 获取父组件相对于世界坐标系的位姿
        parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
    }

    // ps 计算当前 position：父位置 + 父方向 * 相对位置
    my_pose_value.kinematic.position = parent_pose_value.kinematic.position + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));

    // ps 计算当前 orientation：父方向 * 相对方向 * 关节旋转矩阵（Rodrigues 公式）
    // manipulator->getAxis(my_name)：这个关节的旋转轴方向（单位向量）；
    // manipulator->getJointPosition(my_name)：这个关节当前的角度 𝑞𝑖；
    // math::rodriguesRotationMatrix(axis, angle)：罗德里格斯公式生成绕任意轴旋转的 3×3 矩阵；
    my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * manipulator->getComponentRelativeOrientationFromParent(my_name) * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));

    // linear velocity
    // 初始化动态属性为零向量
    my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
    // angular velocity
    my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
    // linear acceleration
    my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
    // angular acceleration
    my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

    // 将计算得到的位姿设置回 manipulator 中
    manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

    // 递归调用所有子组件，继续进行前向求解
    for (int8_t index = 0; index < number_of_child; index++)
    {
        Name child_name = manipulator->getComponentChildName(my_name).at(index);
        forwardSolverUsingChainRule(manipulator, child_name);
    }
}

// NOTICE--1 private：使用【标准雅可比矩阵】和【逆运动学】方法求解机械臂关节角度目标值
/**
 * @brief 使用雅可比矩阵和逆运动学方法求解机械臂关节角度目标值。
 *
 * 该函数通过迭代方式使用雅可比矩阵伪逆法（或带阻尼的最小二乘）来逼近给定的目标位姿，
 * 直到满足精度要求或超过最大迭代次数为止。
 *
 * @param manipulator 输入的机械臂对象指针，用于获取初始状态及进行正向运动学计算。
 * @param tool_name 工具坐标系名称，表示末端执行器的位置与姿态参考点。
 * @param target_pose 目标位姿，包括位置(position)和方向(orientation)，是期望达到的空间位姿。
 * @param goal_joint_value 输出参数，若求解成功则存储最终得到的一组关节角度值及相关动力学信息。
 *                         若失败，则其内容将被置为空。
 *
 * @return bool 返回是否成功收敛到目标位姿。true 表示在限定迭代次数内达到了足够高的精度；
 *              false 表示未达到指定精度即已达到最大迭代次数。
 */
bool SolverUsingCRAndJacobian::inverseSolverUsingJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
    const double lambda = 0.7;   // 步长缩放系数（step size）
    const int8_t iteration = 10; // 最大迭代次数

    Manipulator _manipulator = *manipulator; // 拷贝一份局部副本

    // ps 初始化临时变量
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF()); // 雅可比矩阵，6xDOF，先占位成 6×n 的矩阵；（这里暂设为单位矩阵，随后每轮会被真正的 J 覆盖）
    Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);                        // Δx，6×1 的末端位姿误差（3 平移 + 3 姿态）；（暂设为 0 向量）
    Eigen::VectorXd delta_angle = Eigen::VectorXd::Zero(_manipulator.getDOF());     // Δq，n×1 的关节角增量；（暂设为 0 向量）

    // case1 迭代主循环：收敛成功
    for (int8_t count = 0; count < iteration; count++)
    {
        // step1 Forward kinematics solve --- 用当前关节做 FK，刷新位姿缓存
        solveForwardKinematics(&_manipulator);

        // step2 Get jacobian --- 计算当前雅可比 J(q)
        jacobian = this->jacobian(&_manipulator, tool_name);

        // step3 Pose Difference --- 计算 6×1 的末端位姿误差 Δx
        pose_changed = math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name),
                                            target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));

        // step4 Pose sovler success --- 收敛判定（很严格）
        // 若 ∥Δx∥ 的 2 - 范数 < 1e-6，就认为“够准了”（这个阈值非常小）
        if (pose_changed.norm() < 1E-6)
        {
            // 把当前这份局部副本的关节值拷到 goal_joint_value
            *goal_joint_value = _manipulator.getAllActiveJointValue();

            // 把速度/加速度/力矩字段清零
            for (int8_t index = 0; index < _manipulator.getDOF(); index++)
            {
                goal_joint_value->at(index).velocity = 0.0;
                goal_joint_value->at(index).acceleration = 0.0;
                goal_joint_value->at(index).effort = 0.0;
            }
            return true;
        }

        // step5 Get delta angle --- 线性方程最小二乘：求 Δq
        ColPivHouseholderQR<MatrixXd> dec(jacobian);
        delta_angle = lambda * dec.solve(pose_changed);

        // step6 Set changed angle --- 用 Δq 更新“局部”的关节角
        std::vector<double> changed_angle;
        for (int8_t index = 0; index < _manipulator.getDOF(); index++)
            changed_angle.push_back(_manipulator.getAllActiveJointPosition().at(index) + delta_angle(index));
        _manipulator.setAllActiveJointPosition(changed_angle);
    }

    // case2 循环失败：达到最大迭代次数仍未满足最小阈值限制
    *goal_joint_value = {};
    return false;
}

/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Jacobian
*****************************************************************************/
void SolverUsingCRAndSRJacobian::setOption(const void *arg) {}

Eigen::MatrixXd SolverUsingCRAndSRJacobian::jacobian(Manipulator *manipulator, Name tool_name)
{
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

    Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

    Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
    Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

    //////////////////////////////////////////////////////////////////////////////////

    int8_t index = 0;
    Name my_name = manipulator->getWorldChildName();

    for (int8_t size = 0; size < manipulator->getDOF(); size++)
    {
        Name parent_name = manipulator->getComponentParentName(my_name);
        if (parent_name == manipulator->getWorldName())
        {
            joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
        }
        else
        {
            joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
        }

        position_changed = math::skewSymmetricMatrix(joint_axis) *
                           (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
        orientation_changed = joint_axis;

        pose_changed << position_changed(0),
            position_changed(1),
            position_changed(2),
            orientation_changed(0),
            orientation_changed(1),
            orientation_changed(2);

        jacobian.col(index) = pose_changed;
        index++;
        my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
    }
    return jacobian;
}

// ps 正运动学的对外接口--2
void SolverUsingCRAndSRJacobian::solveForwardKinematics(Manipulator *manipulator)
{
    forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

// ps 逆运动学的对外接口--2
bool SolverUsingCRAndSRJacobian::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
    return inverseSolverUsingSRJacobian(manipulator, tool_name, target_pose, goal_joint_value);
}

// * Private--2
void SolverUsingCRAndSRJacobian::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
    Name my_name = component_name;
    Name parent_name = manipulator->getComponentParentName(my_name);
    int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

    Pose parent_pose_value;
    Pose my_pose_value;

    // Get Parent Pose
    if (parent_name == manipulator->getWorldName())
    {
        parent_pose_value = manipulator->getWorldPose();
    }
    else
    {
        parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
    }

    // Position
    my_pose_value.kinematic.position = parent_pose_value.kinematic.position + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
    // Orientation
    my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * manipulator->getComponentRelativeOrientationFromParent(my_name) * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));
    // Linear velocity
    my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
    // Angular velocity
    my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
    // Linear acceleration
    my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
    // Angular acceleration
    my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

    manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

    for (int8_t index = 0; index < number_of_child; index++)
    {
        Name child_name = manipulator->getComponentChildName(my_name).at(index);
        forwardSolverUsingChainRule(manipulator, child_name);
    }
}

// NOTICE--2 private：使用 SR (Singularity Robust) Jacobian 方法求解机械臂的逆运动学问题
/**
 * @brief 使用 SR (Singularity Robust) Jacobian 方法求解机械臂的逆运动学问题。
 *
 * 该函数通过迭代优化的方式，利用加权的雅可比矩阵和正则化项来处理机械臂在奇异位形下的逆运动学求解。
 * 使用了 Levenberg-Marquardt 类型的优化策略，并在每次迭代中更新关节角度以最小化末端执行器位姿误差。
 *
 * @param manipulator 指向当前机械臂对象的指针，用于获取其状态并进行【正向运动学】计算。
 * @param tool_name 末端执行器的名称，用于确定目标位姿对应的坐标系。
 * @param target_pose 目标末端执行器位姿（位置和姿态）。
 * @param goal_joint_value 输出参数，用于存储求解得到的关节值。若成功求解，则填充该向量；否则清空。
 * @return bool 如果成功收敛到目标位姿则返回 true，否则返回 false。
 */
bool SolverUsingCRAndSRJacobian::inverseSolverUsingSRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
    // manipulator
    Manipulator _manipulator = *manipulator; // 拷贝一份局部 _manipulator，迭代只改副本；成功才把解回传回给原对象

    // solver parameter
    double lambda = 0.0;        // 本轮阻尼系数（每轮动态更新，见后文 lambda = pre_Ek + param）
    const double param = 0.002; // 阻尼的下限/偏置（即使误差很小，也保留少量阻尼）
    const int8_t iteration = 5; // 最多迭代 5 次（上一版是 10）

    const double gamma = 0.5; // rollback delta ：回退比例（若误差没变小，就回滚一半步长）

    // sr sovler parameter：对位置误差的敏感程度
    double wn_pos = 1 / 0.3;        // 位置权重 ~3.333…
    double wn_ang = 1 / (2 * M_PI); // 姿态权重 ~0.159…
    double pre_Ek = 0.0;            // 加权误差能量，用来判定好坏、更新阻尼与是否回退
    double new_Ek = 0.0;

    Eigen::MatrixXd We(6, 6); // 误差加权矩阵，是个 6x6 的矩阵，要用它构造目标函数 E = Δx^⊤ We ​Δx
    We << wn_pos, 0, 0, 0, 0, 0,
        0, wn_pos, 0, 0, 0, 0,
        0, 0, wn_pos, 0, 0, 0,
        0, 0, 0, wn_ang, 0, 0,
        0, 0, 0, 0, wn_ang, 0,
        0, 0, 0, 0, 0, wn_ang;

    Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF()); // 关节空间权重矩阵 Wn，初始化为 DOFxDOF 的单位矩阵

    // jacobian
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());                        // 占位，随后每轮重新计算
    Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF()); // 占位，随后用作正规方程的左侧矩阵

    // delta parameter
    Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);                      // Δx
    Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF()); // delta angle (dq，Δq)
    Eigen::VectorXd gerr(_manipulator.getDOF());                                  // ? 这是啥?

    // angle parameter
    std::vector<double> present_angle; // 当前关节角度 (q)
    std::vector<double> set_angle;     // 更新后的关节数组 (q + dq)

    //////////////////////////// solving //////////////////////////////////

    solveForwardKinematics(&_manipulator); // 初始一次 FK、误差与能量

    ////////////// checking dx ///////////////
    // Δx：目标与当前末端位姿误差（6×1）
    pose_changed = math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
    // ps 初始加权误差能量 Ek​ = Δx⊤ We ​Δx，用于动态阻尼与“是否更好”的判断
    pre_Ek = pose_changed.transpose() * We * pose_changed;
    ///////////////////////////////////////

    //////////////////////////// debug //////////////////////////////////
#if defined(KINEMATICS_DEBUG)
    Eigen::Vector3d target_orientation_rpy = math::convertRotationToRPY(target_pose.orientation);
    Eigen::VectorXd debug_target_pose(6);
    for (int t = 0; t < 3; t++)
        debug_target_pose(t) = target_pose.position(t);
    for (int t = 0; t < 3; t++)
        debug_target_pose(t + 3) = target_orientation_rpy(t);

    Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    Eigen::MatrixXd present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    Eigen::Vector3d present_orientation_rpy = math::convertRotationToRPY(present_orientation);
    Eigen::VectorXd debug_present_pose(6);
    for (int t = 0; t < 3; t++)
        debug_present_pose(t) = present_position(t);
    for (int t = 0; t < 3; t++)
        debug_present_pose(t + 3) = present_orientation_rpy(t);

    log::println("------------------------------------");
    log::warn("iter : first");
    log::warn("Ek : ", pre_Ek * 1000000000000);
    log::println("tar_pose");
    log::println_VECTOR(debug_target_pose, 16);
    log::println("pre_pose");
    log::println_VECTOR(debug_present_pose, 16);
    log::println("delta_pose");
    log::println_VECTOR(debug_target_pose - debug_present_pose, 16);
#endif
    //////////////////////////// debug //////////////////////////////////

    ////////////////////////// solving loop ///////////////////////////////
    // case1 迭代主循环：收敛成功
    for (int8_t count = 0; count < iteration; count++)
    {
        ////////// solve using jacobian //////////
        jacobian = this->jacobian(&_manipulator, tool_name); // 计算当前 J(q)
        lambda = pre_Ek + param;                             // 阻尼自适应

        sr_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn); // calculate sr_jacobian (J^T*we*J + lamda*Wn)
        gerr = jacobian.transpose() * We * pose_changed;                      // calculate gerr (J^T*we) dx

        // 用列主元 Householder-QR 解线性方程，得到 Δq
        ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian); // solving (get dq)
        angle_changed = dec.solve(gerr);                       // (J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

        present_angle = _manipulator.getAllActiveJointPosition();
        set_angle.clear();
        for (int8_t index = 0; index < _manipulator.getDOF(); index++)
            set_angle.push_back(present_angle.at(index) + angle_changed(index));
        _manipulator.setAllActiveJointPosition(set_angle);

        solveForwardKinematics(&_manipulator); // 再做一次 FK，用更新后的 q 刷新末端位姿
        ////////////////////////////////////////

        ////////////// checking dx ///////////////
        // 重新计算误差 Δx 与新的能量 Ek
        pose_changed = math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
        new_Ek = pose_changed.transpose() * We * pose_changed;
        ////////////////////////////////////////

        //////////////////////////// debug //////////////////////////////////
#if defined(KINEMATICS_DEBUG)
        present_position = _manipulator.getComponentPositionFromWorld(tool_name);
        present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
        present_orientation_rpy = math::convertRotationToRPY(present_orientation);
        for (int t = 0; t < 3; t++)
            debug_present_pose(t) = present_position(t);
        for (int t = 0; t < 3; t++)
            debug_present_pose(t + 3) = present_orientation_rpy(t);
        log::warn("iter : ", count, 0);
        log::warn("Ek : ", new_Ek * 1000000000000);
        log::println("tar_pose");
        log::println_VECTOR(debug_target_pose, 16);
        log::println("pre_pose");
        log::println_VECTOR(debug_present_pose, 16);
        log::println("delta_pose");
        log::println_VECTOR(debug_target_pose - debug_present_pose, 16);
#endif
        //////////////////////////// debug //////////////////////////////////

        // * 收敛判据：加权能量 Ek ​< 1e−12（非常严格）
        if (new_Ek < 1E-12)
        {
            ///////////////////////////// debug /////////////////////////////////
#if defined(KINEMATICS_DEBUG)
            log::warn("iter : ", count, 0);
            log::warn("Ek : ", new_Ek * 1000000000000);
            log::error("Success");
            log::println("------------------------------------");
#endif
            ////////////////////////// debug //////////////////////////////////

            *goal_joint_value = _manipulator.getAllActiveJointValue();
            for (int8_t index = 0; index < _manipulator.getDOF(); index++)
            {
                goal_joint_value->at(index).velocity = 0.0;
                goal_joint_value->at(index).acceleration = 0.0;
                goal_joint_value->at(index).effort = 0.0;
            }
            return true;
        }
        else if (new_Ek < pre_Ek)
        {
            pre_Ek = new_Ek;
        }
        else
        {
            present_angle = _manipulator.getAllActiveJointPosition();
            for (int8_t index = 0; index < _manipulator.getDOF(); index++)
                set_angle.push_back(present_angle.at(index) - (gamma * angle_changed(index)));
            _manipulator.setAllActiveJointPosition(set_angle);

            solveForwardKinematics(&_manipulator);
        }
    }

    // case2 循环失败：达到最大迭代次数仍未满足最小阈值限制
    log::error("[sr]fail to solve inverse kinematics (please change the solver)");
    *goal_joint_value = {}; // 清空输出
    return false;           // 返回失败
}

/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Position Only Jacobian
*****************************************************************************/
void SolverUsingCRAndSRPositionOnlyJacobian::setOption(const void *arg) {}

Eigen::MatrixXd SolverUsingCRAndSRPositionOnlyJacobian::jacobian(Manipulator *manipulator, Name tool_name)
{
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

    Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

    Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
    Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

    //////////////////////////////////////////////////////////////////////////////////

    int8_t index = 0;
    Name my_name = manipulator->getWorldChildName();

    for (int8_t size = 0; size < manipulator->getDOF(); size++)
    {
        Name parent_name = manipulator->getComponentParentName(my_name);
        if (parent_name == manipulator->getWorldName())
        {
            joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
        }
        else
        {
            joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
        }

        position_changed = math::skewSymmetricMatrix(joint_axis) *
                           (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
        orientation_changed = joint_axis;

        pose_changed << position_changed(0),
            position_changed(1),
            position_changed(2),
            orientation_changed(0),
            orientation_changed(1),
            orientation_changed(2);

        jacobian.col(index) = pose_changed;
        index++;
        my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
    }
    return jacobian;
}

// ps 正运动学的对外接口--3
void SolverUsingCRAndSRPositionOnlyJacobian::solveForwardKinematics(Manipulator *manipulator)
{
    forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

// ps 逆运动学的对外接口--3
bool SolverUsingCRAndSRPositionOnlyJacobian::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
    return inverseSolverUsingPositionOnlySRJacobian(manipulator, tool_name, target_pose, goal_joint_value);
}

// * private--3
void SolverUsingCRAndSRPositionOnlyJacobian::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
    Name my_name = component_name;
    Name parent_name = manipulator->getComponentParentName(my_name);
    int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

    Pose parent_pose_value;
    Pose my_pose_value;

    // Get Parent Pose
    if (parent_name == manipulator->getWorldName())
    {
        parent_pose_value = manipulator->getWorldPose();
    }
    else
    {
        parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
    }

    // position
    my_pose_value.kinematic.position = parent_pose_value.kinematic.position + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
    // orientation
    my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * manipulator->getComponentRelativeOrientationFromParent(my_name) * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));
    // linear velocity
    my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
    // angular velocity
    my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
    // linear acceleration
    my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
    // angular acceleration
    my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

    manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

    for (int8_t index = 0; index < number_of_child; index++)
    {
        Name child_name = manipulator->getComponentChildName(my_name).at(index);
        forwardSolverUsingChainRule(manipulator, child_name);
    }
}

// NOTICE--3 private：使用位置_only的SR（Selective Resolution）雅可比矩阵求解机械臂逆运动学问题
/**
 * @brief 使用位置_only的SR（Selective Resolution）雅可比矩阵求解机械臂逆运动学问题。
 *
 * 该函数通过迭代方式，使用带阻尼的最小二乘法（Selective Resolution Motion Control）来求解机械臂末端执行器达到目标位姿时各关节的角度。
 * 仅考虑位置误差，忽略姿态误差。
 *
 * @param manipulator 指向当前机械臂对象的指针，用于获取结构信息和进行正运动学计算。
 * @param tool_name 末端执行器的名称，用于获取其在世界坐标系下的位置。
 * @param target_pose 目标位姿，包含期望的位置信息。
 * @param goal_joint_value 输出参数，指向存储求解得到的关节值的向量。
 * @return bool 如果成功求解则返回true，否则返回false。
 */
bool SolverUsingCRAndSRPositionOnlyJacobian::inverseSolverUsingPositionOnlySRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
    // manipulator
    Manipulator _manipulator = *manipulator;

    // solver parameter
    double lambda = 0.0;
    const double param = 0.002;
    const int8_t iteration = 10;

    const double gamma = 0.5; // rollback delta

    // jacobian
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());
    Eigen::MatrixXd position_jacobian = Eigen::MatrixXd::Identity(3, _manipulator.getDOF());
    Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

    // delta parameter
    Eigen::Vector3d position_changed = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF()); // delta angle (dq)
    Eigen::VectorXd gerr(_manipulator.getDOF());

    // sr sovler parameter
    double wn_pos = 1 / 0.3;
    double pre_Ek = 0.0;
    double new_Ek = 0.0;

    Eigen::MatrixXd We(3, 3);
    We << wn_pos, 0, 0,
        0, wn_pos, 0,
        0, 0, wn_pos;

    Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

    // angle parameter
    std::vector<double> present_angle; // angle (q)
    std::vector<double> set_angle;     // set angle (q + dq)

    ////////////////////////////solving//////////////////////////////////

    solveForwardKinematics(&_manipulator);
    //////////////checking dx///////////////
    position_changed = math::positionDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name));
    pre_Ek = position_changed.transpose() * We * position_changed;
///////////////////////////////////////

/////////////////////////////debug/////////////////////////////////
#if defined(KINEMATICS_DEBUG)
    Eigen::Vector3d target_orientation_rpy = math::convertRotationToRPY(target_pose.orientation);
    Eigen::VectorXd debug_target_pose(6);
    for (int t = 0; t < 3; t++)
        debug_target_pose(t) = target_pose.position(t);
    for (int t = 0; t < 3; t++)
        debug_target_pose(t + 3) = target_orientation_rpy(t);

    Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    Eigen::MatrixXd present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    Eigen::Vector3d present_orientation_rpy = math::convertRotationToRPY(present_orientation);
    Eigen::VectorXd debug_present_pose(6);
    for (int t = 0; t < 3; t++)
        debug_present_pose(t) = present_position(t);
    for (int t = 0; t < 3; t++)
        debug_present_pose(t + 3) = present_orientation_rpy(t);

    log::println("------------------------------------");
    log::warn("iter : first");
    log::warn("Ek : ", pre_Ek * 1000000000000);
    log::println("tar_pose");
    log::println_VECTOR(debug_target_pose, 16);
    log::println("pre_pose");
    log::println_VECTOR(debug_present_pose, 16);
    log::println("delta_pose");
    log::println_VECTOR(debug_target_pose - debug_present_pose, 16);
#endif
    ////////////////////////////debug//////////////////////////////////

    //////////////////////////solving loop///////////////////////////////
    // case1 迭代主循环：收敛成功
    for (int8_t count = 0; count < iteration; count++)
    {
        //////////solve using jacobian//////////
        jacobian = this->jacobian(&_manipulator, tool_name);
        position_jacobian.row(0) = jacobian.row(0);
        position_jacobian.row(1) = jacobian.row(1);
        position_jacobian.row(2) = jacobian.row(2);
        lambda = pre_Ek + param;

        sr_jacobian = (position_jacobian.transpose() * We * position_jacobian) + (lambda * Wn); // calculate sr_jacobian (J^T*we*J + lamda*Wn)
        gerr = position_jacobian.transpose() * We * position_changed;                           // calculate gerr (J^T*we) dx

        ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian); // solving (get dq)
        angle_changed = dec.solve(gerr);                       //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

        present_angle = _manipulator.getAllActiveJointPosition();
        set_angle.clear();
        for (int8_t index = 0; index < _manipulator.getDOF(); index++)
            set_angle.push_back(_manipulator.getAllActiveJointPosition().at(index) + angle_changed(index));
        _manipulator.setAllActiveJointPosition(set_angle);
        solveForwardKinematics(&_manipulator);
        ////////////////////////////////////////

        //////////////checking dx///////////////
        position_changed = math::positionDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name));
        new_Ek = position_changed.transpose() * We * position_changed;
        ////////////////////////////////////////

        /////////////////////////////debug/////////////////////////////////
#if defined(KINEMATICS_DEBUG)
        present_position = _manipulator.getComponentPositionFromWorld(tool_name);
        present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
        present_orientation_rpy = math::convertRotationToRPY(present_orientation);
        for (int t = 0; t < 3; t++)
            debug_present_pose(t) = present_position(t);
        for (int t = 0; t < 3; t++)
            debug_present_pose(t + 3) = present_orientation_rpy(t);
        log::warn("iter : ", count, 0);
        log::warn("Ek : ", new_Ek * 1000000000000);
        log::println("tar_pose");
        log::println_VECTOR(debug_target_pose, 16);
        log::println("pre_pose");
        log::println_VECTOR(debug_present_pose, 16);
        log::println("delta_pose");
        log::println_VECTOR(debug_target_pose - debug_present_pose, 16);
#endif
        ////////////////////////////debug//////////////////////////////////

        if (new_Ek < 1E-12)
        {
            /////////////////////////////debug/////////////////////////////////
#if defined(KINEMATICS_DEBUG)
            log::warn("iter : ", count, 0);
            log::warn("Ek : ", new_Ek * 1000000000000);
            log::error("IK Success");
            log::println("------------------------------------");
#endif
            //////////////////////////debug//////////////////////////////////
            *goal_joint_value = _manipulator.getAllActiveJointValue();
            for (int8_t index = 0; index < _manipulator.getDOF(); index++)
            {
                goal_joint_value->at(index).velocity = 0.0;
                goal_joint_value->at(index).acceleration = 0.0;
                goal_joint_value->at(index).effort = 0.0;
            }
            return true;
        }
        else if (new_Ek < pre_Ek)
        {
            pre_Ek = new_Ek;
        }
        else
        {
            present_angle = _manipulator.getAllActiveJointPosition();
            for (int8_t index = 0; index < _manipulator.getDOF(); index++)
                set_angle.push_back(_manipulator.getAllActiveJointPosition().at(index) - (gamma * angle_changed(index)));
            _manipulator.setAllActiveJointPosition(set_angle);

            solveForwardKinematics(&_manipulator);
        }
    }

    // case2 循环失败：达到最大迭代次数仍未满足最小阈值限制
    log::error("[position_only]fail to solve inverse kinematics (please change the solver)");
    *goal_joint_value = {};
    return false;
}

/*****************************************************************************
** Kinematics Solver Customized for OpenManipulator Chain
*****************************************************************************/
void SolverCustomizedforOMChain::setOption(const void *arg) {}

Eigen::MatrixXd SolverCustomizedforOMChain::jacobian(Manipulator *manipulator, Name tool_name)
{
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

    Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

    Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
    Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

    //////////////////////////////////////////////////////////////////////////////////

    int8_t index = 0;
    Name my_name = manipulator->getWorldChildName();

    for (int8_t size = 0; size < manipulator->getDOF(); size++)
    {
        Name parent_name = manipulator->getComponentParentName(my_name);
        if (parent_name == manipulator->getWorldName())
        {
            joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
        }
        else
        {
            joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
        }

        position_changed = math::skewSymmetricMatrix(joint_axis) *
                           (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
        orientation_changed = joint_axis;

        pose_changed << position_changed(0),
            position_changed(1),
            position_changed(2),
            orientation_changed(0),
            orientation_changed(1),
            orientation_changed(2);

        jacobian.col(index) = pose_changed;
        index++;
        my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
    }
    return jacobian;
}

void SolverCustomizedforOMChain::solveForwardKinematics(Manipulator *manipulator)
{
    forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverCustomizedforOMChain::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
    return chainCustomInverseKinematics(manipulator, tool_name, target_pose, goal_joint_value);
}

// * private--4
void SolverCustomizedforOMChain::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
    Name my_name = component_name;
    Name parent_name = manipulator->getComponentParentName(my_name);
    int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

    Pose parent_pose_value;
    Pose my_pose_value;

    // Get Parent Pose
    if (parent_name == manipulator->getWorldName())
    {
        parent_pose_value = manipulator->getWorldPose();
    }
    else
    {
        parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
    }

    // position
    my_pose_value.kinematic.position = parent_pose_value.kinematic.position + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
    // orientation
    my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * manipulator->getComponentRelativeOrientationFromParent(my_name) * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));
    // linear velocity
    my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
    // angular velocity
    my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
    // linear acceleration
    my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
    // angular acceleration
    my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

    manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

    for (int8_t index = 0; index < number_of_child; index++)
    {
        Name child_name = manipulator->getComponentChildName(my_name).at(index);
        forwardSolverUsingChainRule(manipulator, child_name);
    }
}

// NOTICE--4 private：自定义逆运动学求解器，用于OpenManipulator 机械臂链结构
/**
 * @brief 自定义逆运动学求解器，用于OpenManipulator机械臂链结构。
 *
 * 此函数通过迭代优化方法（基于加权最小二乘与阻尼最小二乘）计算给定目标位姿下的关节角度。
 * 使用了自定义的目标方向调整策略以适应特定的机械臂结构约束，并在每次迭代中更新前向运动学模型，
 * 直到误差小于阈值或达到最大迭代次数为止。
 *
 * @param manipulator 指向当前操作对象的指针，表示要进行IK求解的机械臂。
 * @param tool_name 工具坐标系名称，指定末端执行器的位置和姿态参考点。
 * @param target_pose 目标位姿，包括位置和旋转信息。
 * @param goal_joint_value 输出参数，存储成功求解后的关节角及其速度、加速度等附加属性。
 * @return bool 成功求解返回true；否则返回false。
 */
bool SolverCustomizedforOMChain::chainCustomInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
    // manipulator
    Manipulator _manipulator = *manipulator;

    // solver parameter
    double lambda = 0.0;
    const double param = 0.002;
    const int8_t iteration = 10;

    const double gamma = 0.5; // rollback delta

    // sr sovler parameter
    double wn_pos = 1 / 0.3;
    double wn_ang = 1 / (2 * M_PI);
    double pre_Ek = 0.0;
    double new_Ek = 0.0;

    Eigen::MatrixXd We(6, 6);
    We << wn_pos, 0, 0, 0, 0, 0,
        0, wn_pos, 0, 0, 0, 0,
        0, 0, wn_pos, 0, 0, 0,
        0, 0, 0, wn_ang, 0, 0,
        0, 0, 0, 0, wn_ang, 0,
        0, 0, 0, 0, 0, wn_ang;

    Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

    // jacobian
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());
    Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

    // delta parameter
    Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF()); // delta angle (dq)
    Eigen::VectorXd gerr(_manipulator.getDOF());

    // angle parameter
    std::vector<double> present_angle; // angle (q)
    std::vector<double> set_angle;     // set angle (q + dq)

    ////////////////////////////solving//////////////////////////////////

    solveForwardKinematics(&_manipulator);

    //////////////make target ori//////////  //only OpenManipulator Chain
    Eigen::Matrix3d present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    Eigen::Vector3d present_orientation_rpy = math::convertRotationMatrixToRPYVector(present_orientation);
    Eigen::Matrix3d target_orientation = target_pose.kinematic.orientation;
    Eigen::Vector3d target_orientation_rpy = math::convertRotationMatrixToRPYVector(target_orientation);

    Eigen::Vector3d joint1_rlative_position = _manipulator.getComponentRelativePositionFromParent(_manipulator.getWorldChildName());
    Eigen::Vector3d target_position_from_joint1 = target_pose.kinematic.position - joint1_rlative_position;

    target_orientation_rpy(0) = present_orientation_rpy(0);
    target_orientation_rpy(1) = target_orientation_rpy(1);
    target_orientation_rpy(2) = atan2(target_position_from_joint1(1), target_position_from_joint1(0));

    target_pose.kinematic.orientation = math::convertRPYToRotationMatrix(target_orientation_rpy(0), target_orientation_rpy(1), target_orientation_rpy(2));
    ///////////////////////////////////////

    //////////////checking dx///////////////
    pose_changed = math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
    pre_Ek = pose_changed.transpose() * We * pose_changed;
///////////////////////////////////////

/////////////////////////////debug/////////////////////////////////
#if defined(KINEMATICS_DEBUG)
    Eigen::VectorXd debug_target_pose(6);
    for (int t = 0; t < 3; t++)
        debug_target_pose(t) = target_pose.position(t);
    for (int t = 0; t < 3; t++)
        debug_target_pose(t + 3) = target_orientation_rpy(t);

    Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    Eigen::VectorXd debug_present_pose(6);
    for (int t = 0; t < 3; t++)
        debug_present_pose(t) = present_position(t);
    for (int t = 0; t < 3; t++)
        debug_present_pose(t + 3) = present_orientation_rpy(t);

    log::println("------------------------------------");
    log::warn("iter : first");
    log::warn("Ek : ", pre_Ek * 1000000000000);
    log::println("tar_pose");
    log::println_VECTOR(debug_target_pose, 16);
    log::println("pre_pose");
    log::println_VECTOR(debug_present_pose, 16);
    log::println("delta_pose");
    log::println_VECTOR(debug_target_pose - debug_present_pose, 16);
#endif
    ////////////////////////////debug//////////////////////////////////

    //////////////////////////solving loop///////////////////////////////
    for (int8_t count = 0; count < iteration; count++)
    {
        //////////solve using jacobian//////////
        jacobian = this->jacobian(&_manipulator, tool_name);
        lambda = pre_Ek + param;

        sr_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn); // calculate sr_jacobian (J^T*we*J + lamda*Wn)
        gerr = jacobian.transpose() * We * pose_changed;                      // calculate gerr (J^T*we) dx

        ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian); // solving (get dq)
        angle_changed = dec.solve(gerr);                       //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

        present_angle = _manipulator.getAllActiveJointPosition();
        set_angle.clear();
        for (int8_t index = 0; index < _manipulator.getDOF(); index++)
            set_angle.push_back(present_angle.at(index) + angle_changed(index));
        _manipulator.setAllActiveJointPosition(set_angle);
        solveForwardKinematics(&_manipulator);
        ////////////////////////////////////////

        //////////////checking dx///////////////
        pose_changed = math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
        new_Ek = pose_changed.transpose() * We * pose_changed;
////////////////////////////////////////

/////////////////////////////debug/////////////////////////////////
#if defined(KINEMATICS_DEBUG)
        present_position = _manipulator.getComponentPositionFromWorld(tool_name);
        present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
        present_orientation_rpy = math::convertRotationToRPY(present_orientation);
        for (int t = 0; t < 3; t++)
            debug_present_pose(t) = present_position(t);
        for (int t = 0; t < 3; t++)
            debug_present_pose(t + 3) = present_orientation_rpy(t);
        log::warn("iter : ", count, 0);
        log::warn("Ek : ", new_Ek * 1000000000000);
        log::println("tar_pose");
        log::println_VECTOR(debug_target_pose, 16);
        log::println("pre_pose");
        log::println_VECTOR(debug_present_pose, 16);
        log::println("delta_pose");
        log::println_VECTOR(debug_target_pose - debug_present_pose, 16);
#endif
        ////////////////////////////debug//////////////////////////////////

        if (new_Ek < 1E-12)
        {
/////////////////////////////debug/////////////////////////////////
#if defined(KINEMATICS_DEBUG)
            log::warn("iter : ", count, 0);
            log::warn("Ek : ", new_Ek * 1000000000000);
            log::error("Success");
            log::println("------------------------------------");
#endif
            //////////////////////////debug//////////////////////////////////
            *goal_joint_value = _manipulator.getAllActiveJointValue();
            for (int8_t index = 0; index < _manipulator.getDOF(); index++)
            {
                goal_joint_value->at(index).velocity = 0.0;
                goal_joint_value->at(index).acceleration = 0.0;
                goal_joint_value->at(index).effort = 0.0;
            }
            return true;
        }
        else if (new_Ek < pre_Ek)
        {
            pre_Ek = new_Ek;
        }
        else
        {
            present_angle = _manipulator.getAllActiveJointPosition();
            for (int8_t index = 0; index < _manipulator.getDOF(); index++)
                set_angle.push_back(present_angle.at(index) - (gamma * angle_changed(index)));
            _manipulator.setAllActiveJointPosition(set_angle);

            solveForwardKinematics(&_manipulator);
        }
    }
    log::error("[OpenManipulator Chain Custom]fail to solve inverse kinematics");
    *goal_joint_value = {};
    return false;
}

/*****************************************************************************
** Kinematics Solver Using Geometry Approach
*****************************************************************************/
void SolverUsingCRAndGeometry::setOption(const void *arg)
{
    with_gripper_ = arg;
}

Eigen::MatrixXd SolverUsingCRAndGeometry::jacobian(Manipulator *manipulator, Name tool_name)
{
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());
    return jacobian;
}

void SolverUsingCRAndGeometry::solveForwardKinematics(Manipulator *manipulator)
{
    forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverUsingCRAndGeometry::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
    return inverseSolverUsingGeometry(manipulator, tool_name, target_pose, goal_joint_value);
}

// * private--5
void SolverUsingCRAndGeometry::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
    Name my_name = component_name;
    Name parent_name = manipulator->getComponentParentName(my_name);
    int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

    Pose parent_pose_value;
    Pose my_pose_value;

    // Get Parent Pose
    if (parent_name == manipulator->getWorldName())
    {
        parent_pose_value = manipulator->getWorldPose();
    }
    else
    {
        parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
    }

    // position
    my_pose_value.kinematic.position = parent_pose_value.kinematic.position + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
    // orientation
    my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * manipulator->getComponentRelativeOrientationFromParent(my_name) * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));
    // linear velocity
    my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
    // angular velocity
    my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
    // linear acceleration
    my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
    // angular acceleration
    my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

    manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

    for (int8_t index = 0; index < number_of_child; index++)
    {
        Name child_name = manipulator->getComponentChildName(my_name).at(index);
        forwardSolverUsingChainRule(manipulator, child_name);
    }
}

// NOTICE--5 private：使用几何方法求解机械臂逆运动学问题
/**
 * @brief 使用几何方法求解机械臂逆运动学问题
 *
 * 该函数通过解析几何方法计算给定目标位姿下机械臂各关节的角度值。
 * 主要步骤包括：
 * 1. 根据目标位置计算关节1的角度
 * 2. 通过几何关系计算关节2和关节3的角度
 * 3. 根据目标姿态计算关节4、5、6的角度
 * 4. 对计算结果进行有效性检查
 *
 * @param manipulator 指向机械臂对象的指针，用于获取机械臂参数和当前状态
 * @param tool_name 工具名称，用于获取工具相对于末端执行器的位置
 * @param target_pose 目标位姿，包含位置和姿态信息
 * @param goal_joint_value 输出参数，存储计算得到的关节角度值
 * @return bool 返回true表示计算成功，返回false表示计算失败（如出现NaN或无穷大值）
 */
bool SolverUsingCRAndGeometry::inverseSolverUsingGeometry(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
    Manipulator _manipulator = *manipulator;
    JointValue target_angle[6];
    std::vector<JointValue> target_angle_vector;

    //// Position
    // Compute Joint 1 Angle
    Eigen::VectorXd position = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd orientation = Eigen::MatrixXd::Zero(3, 3);
    position = target_pose.kinematic.position;
    orientation = target_pose.kinematic.orientation;
    double d6 = 0.123;

    if (with_gripper_)
    {
        auto tool_length = _manipulator.getComponentRelativePositionFromParent(tool_name);
        d6 += tool_length(0);
    }
    Eigen::Vector3d position_2 = Eigen::VectorXd::Zero(3);
    position_2 << orientation(0, 0), orientation(1, 0), orientation(2, 0);
    Eigen::Vector3d position_3 = Eigen::VectorXd::Zero(3);
    position_3 = position - d6 * position_2;
    if (position_3(0) == 0)
        position_3(0) = 0.001;
    target_angle[0].position = atan(position_3(1) / position_3(0));

    // Compute Joint 3 Angle
    Eigen::VectorXd position3 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd position3_2 = Eigen::VectorXd::Zero(3);
    position3 << 0.0, 0.0, 0.126;
    position3_2 << 0.0, 0.0, 0.033;
    Eigen::MatrixXd orientation3 = math::convertRPYToRotationMatrix(0, 0, target_angle[0].position);
    Eigen::VectorXd position3_3 = Eigen::VectorXd::Zero(3);
    position3_3 = position3 + orientation3 * position3_2;
    Eigen::VectorXd position3_4 = Eigen::VectorXd::Zero(3);
    position3_4 = position_3 - position3_3;
    double l1 = sqrt(0.264 * 0.264 + 0.030 * 0.030);
    double l2 = sqrt(0.030 * 0.030 + 0.258 * 0.258);
    double phi = acos((l1 * l1 + l2 * l2 - position3_4.norm() * position3_4.norm()) / (2 * l1 * l2));
    double alpha1 = atan2(0.030, 0.264);
    double alpha2 = atan2(0.258, 0.030);
    target_angle[2].position = PI - (phi - alpha1) - alpha2;

    // Compute Joint 2 Angle
    Eigen::VectorXd position2 = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd orientation2 = math::convertRPYToRotationMatrix(0, 0, target_angle[0].position);
    position2 = orientation2.inverse() * position3_4;
    double beta1 = atan2(position2(2), position2(0));
    double beta2 = acos((l1 * l1 + position3_4.norm() * position3_4.norm() - l2 * l2) / (2 * l1 * position3_4.norm()));
    if (position3_4(2) > 0)
        target_angle[1].position = (PI / 2 - alpha1) - fabs(beta1) - beta2;
    else
        target_angle[1].position = (PI / 2 - alpha1) + fabs(beta1) - beta2;

    //// Orientation
    // Compute Joint 4, 5, 6 Angles
    Eigen::MatrixXd orientation_to_joint3 = math::convertRPYToRotationMatrix(0, 0, target_angle[0].position) * math::convertRPYToRotationMatrix(0, target_angle[1].position, 0) * math::convertRPYToRotationMatrix(0, target_angle[2].position, 0);
    Eigen::MatrixXd orientation_def = orientation_to_joint3.transpose() * orientation;

    if (orientation_def(0, 0) < 1.0)
    {
        if (orientation_def(0, 0) > -1.0)
        {
            double joint4_angle_present = _manipulator.getJointPosition("joint4");
            double joint4_angle_temp_1 = atan2(orientation_def(1, 0), -orientation_def(2, 0));
            double joint4_angle_temp_2 = atan2(-orientation_def(1, 0), orientation_def(2, 0));

            if (fabs(joint4_angle_temp_1 - joint4_angle_present) < fabs(joint4_angle_temp_2 - joint4_angle_present))
            {
                // log::println("joint4_angle_temp_1", fabs(joint4_angle_present-joint4_angle_temp_1));
                target_angle[3].position = joint4_angle_temp_1;
                target_angle[4].position = acos(orientation_def(0, 0));
                target_angle[5].position = atan2(orientation_def(0, 1), orientation_def(0, 2));
            }
            else
            {
                // log::println("joint4_angle_temp_2", fabs(joint4_angle_present-joint4_angle_temp_2));
                target_angle[3].position = joint4_angle_temp_2;
                target_angle[4].position = -acos(orientation_def(0, 0));
                target_angle[5].position = atan2(-orientation_def(0, 1), -orientation_def(0, 2));
            }
        }
        else // R(0,0) = -1
        {
            target_angle[3].position = _manipulator.getJointPosition("joint4");
            target_angle[4].position = PI;
            target_angle[5].position = atan2(-orientation_def(1, 2), orientation_def(1, 1)) + target_angle[3].position;
        }
    }
    else // R(0,0) = 1
    {
        target_angle[3].position = _manipulator.getJointPosition("joint4");
        target_angle[4].position = 0.0;
        target_angle[5].position = atan2(-orientation_def(1, 2), orientation_def(1, 1)) - target_angle[3].position;
    }

    // log::println("------------------------------------");
    // log::println("End-effector Pose : ");
    // log::println("position1: ", target_angle[0].position);
    // log::println("position2: ", target_angle[1].position);
    // log::println("position3: ", target_angle[2].position);
    // log::println("position5: ", target_angle[4].position);
    // log::println("position4: ", target_angle[3].position);
    // log::println("position6: ", target_angle[5].position);
    // log::println("------------------------------------");

    if (std::isnan(target_angle[0].position) ||
        std::isnan(target_angle[1].position) ||
        std::isnan(target_angle[2].position) ||
        std::isnan(target_angle[3].position) ||
        std::isnan(target_angle[4].position) ||
        std::isnan(target_angle[5].position))
    {
        log::error("Target angle value is NAN!!");
        return false;
    }

    if (std::isinf(target_angle[0].position) ||
        std::isinf(target_angle[1].position) ||
        std::isinf(target_angle[2].position) ||
        std::isinf(target_angle[3].position) ||
        std::isinf(target_angle[4].position) ||
        std::isinf(target_angle[5].position))
    {
        log::error("Target angle value is INF!!");
        return false;
    }

    target_angle_vector.push_back(target_angle[0]);
    target_angle_vector.push_back(target_angle[1]);
    target_angle_vector.push_back(target_angle[2]);
    target_angle_vector.push_back(target_angle[3]);
    target_angle_vector.push_back(target_angle[4]);
    target_angle_vector.push_back(target_angle[5]);

    *goal_joint_value = target_angle_vector;

    return true;
}
