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


#include "rotors_joy_interface/joy.h"

#include <mav_msgs/default_topics.h>

Joy::Joy() {
  ros::NodeHandle nh; // 全局命名空间
  ros::NodeHandle pnh("~"); // 局部命名空间（发布话题自带节点名）
  ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 10);

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;

  // 设置无人机类型
  pnh.param("is_fixed_wing", is_fixed_wing_, false);
  pnh.param("deadzone_range", deadzone_range, (float)0.02);

  // 设置摇杆键位（左01右23）
  pnh.param("axis_yaw_rate_", axes_.yaw_rate, 0);
  pnh.param("axis_thrust_", axes_.thrust, 1);
  pnh.param("axis_roll_", axes_.roll, 2);
  pnh.param("axis_pitch_", axes_.pitch, 3);
  
  // 设置摇杆反转
  pnh.param("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, 1);
  pnh.param("axis_direction_yaw_rate", axes_.yaw_rate_direction, 1);
  pnh.param("axis_direction_thrust", axes_.thrust_direction, 1);

  // 设置摇杆最大值
  pnh.param("max_v_xy", max_.v_xy, 1.0);  // [m/s]
  pnh.param("max_roll", max_.roll, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_pitch", max_.pitch, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_yaw_rate", max_.yaw_rate, 45.0 * M_PI / 180.0);  // [rad/s]
  pnh.param("max_thrust", max_.thrust, 30.0);  // [N]

  // 设置按键键位
  pnh.param("button_yaw_left_", buttons_.yaw_left, 6);
  pnh.param("button_yaw_right_", buttons_.yaw_right, 7);
  pnh.param("button_ctrl_enable_", buttons_.ctrl_enable, 3);
  pnh.param("button_ctrl_mode_", buttons_.ctrl_mode, 2);
  pnh.param("button_takeoff_", buttons_.takeoff, 8);
  pnh.param("button_land_", buttons_.land, 9);

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
}

void Joy::StopMav() {
  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
}

// 设置死区减小摇杆的微动误差
float deadzone(float value, float range){
  if (abs(value)>range){
    if (value>0){
      // 将摇杆的范围还原到[-1,1]（保证max值生效）
      return (value-range)/(1.0-range);
    }else{
      return (value+range)/(1.0-range);
    }
  }else{
    return 0.0;
  }
}

// 接收ROS自带joy话题后的callback
void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;

  // 横滚俯仰控制信号
  control_msg_.roll = deadzone(msg->axes[axes_.roll],deadzone_range) * max_.roll * axes_.roll_direction;
  control_msg_.pitch = deadzone(msg->axes[axes_.pitch],deadzone_range) * max_.pitch * axes_.pitch_direction;
  
  // 偏航控制信号（向左转为正，即z轴向上）
  if (msg->buttons[buttons_.yaw_left]) {
    control_msg_.yaw_rate  = max_.yaw_rate;
  }
  else if (msg->buttons[buttons_.yaw_right]) {
    control_msg_.yaw_rate  = -max_.yaw_rate;
  }
  else {
    control_msg_.yaw_rate = deadzone(msg->axes[axes_.yaw_rate],deadzone_range) * max_.yaw_rate * axes_.yaw_rate_direction;
  }

  // 机体类型设置（油门分xyz方向）
  if (is_fixed_wing_) {
    double thrust = deadzone(msg->axes[axes_.thrust],deadzone_range) * axes_.thrust_direction;
    control_msg_.thrust.x = (thrust >= 0.0) ? thrust : 0.0;
  }
  else {
    control_msg_.thrust.z = (deadzone(msg->axes[axes_.thrust],deadzone_range) + 1) * max_.thrust / 2.0 * axes_.thrust_direction;
  }

  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  control_msg_.header.frame_id = "rotors_joy_frame";
  Publish();
}

void Joy::Publish() {
  ctrl_pub_.publish(control_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface");
  Joy joy;


  ROS_INFO("Axes settings: [yaw_rate, thrust, roll, pitch] = [%d, %d, %d, %d].",
           joy.axes_.yaw_rate, joy.axes_.thrust, joy.axes_.roll , joy.axes_.pitch);

  ros::spin();

  return 0;
}
