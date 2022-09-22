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


#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_H

#include <Eigen/Eigen>

// 定义了motor仿真话题框架、转速和参考转速
class MotorModel
{
  public:
    MotorModel()
        : motor_rot_vel_(0.0),
          ref_motor_rot_vel_(0.0),
          prev_sim_time_(0.0),
          sampling_time_(0.01) {}
    virtual ~MotorModel() {}
    void GetMotorVelocity(double &result) const {
      result = motor_rot_vel_;
    }
    void SetReferenceMotorVelocity(double ref_motor_rot_vel) {
      ref_motor_rot_vel_ = ref_motor_rot_vel;
    }

    /*
    纯虚函数是在声明虚函数时被“初始化”为0的函数。声明纯虚函数的一般形式是
    virtual 函数类型 函数名 (参数表列) =0;

    注意: ①纯虚函数没有函数体；②最后面的“=0”并不表示函数返回值为0，它只
    起形式上的作用，告诉编译系统“这是纯虚函数”; ③这是一个声明语句，最后应
    有分号。
    纯虚函数只有函数的名字而不具备函数的功能，不能被调用。它只是通知编译系
    统: “在这里声明一个虚函数，留待派生类中定义”。在派生类中对此函数提供定
    义后，它才能具备函数的功能，可被调用。 
    */

    // 纯虚函数初始化为0，在派生类中定义
    virtual void InitializeParams() = 0;
    // 纯虚函数初始化为0，在派生类中定义
    virtual void Publish() = 0;

  protected:
    double motor_rot_vel_;
    double ref_motor_rot_vel_;
    double prev_ref_motor_rot_vel_;
    double prev_sim_time_;
    double sampling_time_;


    virtual void UpdateForcesAndMoments() = 0;
};

#endif // ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_H
