//
// Created by luotinkai on 2022/7/15.
//

#ifndef SRC_DART_MANUAL_H
#define SRC_DART_MANUAL_H

#include "rm_manual/chassis_gimbal_manual.h"
#include <actionlib/client/simple_action_client.h>
#include <rm_common/decision/calibration_queue.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utility>

namespace rm_manual
{
class DartManual : public ManualBase
{
public:
  explicit DartManual(ros::NodeHandle& nh);

protected:
  void sendCommand(const ros::Time& time) override;
  void run() override;
  void checkReferee() override;
  void remoteControlTurnOn() override;
  void leftSwitchUpFall();
  void leftSwitchDownRise() override;
  void leftSwitchMidRise() override;
  void leftSwitchUpRise() override;
  void updateRc() override;
  void updatePc() override;
  void move(rm_common::JointPointCommandSender* joint, double ch);
  void scaleAdjust();
  void recordPosition();

  rm_common::JointPointCommandSender *trigger_sender_, *friction_left_sender_, *friction_right_sender_;
  rm_common::JointPointCommandSender *pitch_sender_, *yaw_sender_;
  rm_common::CalibrationQueue *trigger_calibration_, *gimbal_calibration_;
  double pitch_outpost_{}, pitch_base_{}, yaw_outpost_{}, yaw_base_{};
  double qd_, upward_vel_, scale_;
  bool if_stop_{ true };

  InputEvent chassis_power_on_event_, gimbal_power_on_event_;
};
}  // namespace rm_manual

#endif  // SRC_DART_MANUAL_H
