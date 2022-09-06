/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_H
#define ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_H
// 这是一个纯C++库，学到了，非ROS节点可以作为库加到CMakeList里

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {

// Default values for the roll pitch yawrate thrust controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

class RollPitchYawrateThrustControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RollPitchYawrateThrustControllerParameters()
      : attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};

// 电机的控制分配
class RollPitchYawrateThrustController {
 public:
  RollPitchYawrateThrustController();
  ~RollPitchYawrateThrustController();
  void InitializeParameters();
  // 计算并赋值电机转速（返回void，从输入赋值）
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const; //常成员函数, 它不改变对象的成员变量，也不能调用类中任何非const成员函数。
  // 下面两个set函数作用是msg_=msg，可以忽略
  void SetOdometry(const EigenOdometry& odometry);
  void SetRollPitchYawrateThrust(//附带解锁电机命令
      const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust);

  RollPitchYawrateThrustControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  bool initialized_params_;
  bool controller_active_; //解锁电机

  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

  mav_msgs::EigenRollPitchYawrateThrust roll_pitch_yawrate_thrust_;
  EigenOdometry odometry_;
  // 计算并赋值角加速度
  void ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acceleration) const;
};
}

#endif // ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_H
