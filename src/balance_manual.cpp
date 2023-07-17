//
// Created by yuchen on 2023/4/3.
//

#include "rm_manual/balance_manual.h"

namespace rm_manual
{
BalanceManual::BalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalShooterCoverManual(nh, nh_referee)
{
  ros::NodeHandle balance_nh(nh, "balance");
  balance_cmd_sender_ = new rm_common::BalanceCommandSender(balance_nh);
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);

  nh.param("flank_frame", flank_frame_, std::string("flank_frame"));
  nh.param("reverse_frame", reverse_frame_, std::string("yaw_reverse_frame"));
  nh.param("balance_dangerous_angle", balance_dangerous_angle_, 0.3);

  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("chassis_calibration", rpc_value);
  chassis_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);

  is_balance_ = true;
  flank_ = true;

  x_event_.setRising(boost::bind(&BalanceManual::xPress, this));
  g_event_.setRising(boost::bind(&BalanceManual::gPress, this));
  v_event_.setRising(boost::bind(&BalanceManual::vPress, this));
  auto_fallen_event_.setActiveHigh(boost::bind(&BalanceManual::modeFallen, this, _1));
  auto_fallen_event_.setDelayTriggered(boost::bind(&BalanceManual::modeNormalize, this), 1.5, true);
  ctrl_x_event_.setRising(boost::bind(&BalanceManual::ctrlXPress, this));
}

void BalanceManual::run()
{
  ChassisGimbalShooterCoverManual::run();
  chassis_calibration_->update(ros::Time::now());
}

void BalanceManual::sendCommand(const ros::Time& time)
{
  if (flank_)
    chassis_cmd_sender_->getMsg()->follow_source_frame = flank_frame_;
  else if (reverse_)
    chassis_cmd_sender_->getMsg()->follow_source_frame = reverse_frame_;
  else
    chassis_cmd_sender_->getMsg()->follow_source_frame = "yaw";

  if (supply_)
  {
    cover_close_ = false;
    cover_command_sender_->on();
  }
  else
  {
    cover_close_ = true;
    cover_command_sender_->off();
  }

  ChassisGimbalShooterManual::sendCommand(time);
  cover_command_sender_->sendCommand(time);
  balance_cmd_sender_->sendCommand(time);
}

void BalanceManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
  v_event_.update(dbus_data->key_v && !dbus_data->key_ctrl);
  ctrl_x_event_.update(dbus_data->key_ctrl && dbus_data->key_x);
}

void BalanceManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::updateRc(dbus_data);
  if (std::abs(dbus_data->ch_r_x) > 0.5 && std::abs(dbus_data->ch_r_x) > std::abs(dbus_data->ch_r_y))
    flank_ = true;
  else if (std::abs(dbus_data->ch_r_y) > 0.5 && std::abs(dbus_data->ch_r_y) > std::abs(dbus_data->ch_r_x))
    flank_ = false;
}

void BalanceManual::rightSwitchDownRise()
{
  ChassisGimbalShooterCoverManual::rightSwitchDownRise();
  state_ = RC;
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FALLEN);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void BalanceManual::rightSwitchMidRise()
{
  ChassisGimbalShooterCoverManual::rightSwitchMidRise();
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void BalanceManual::ctrlZPress()
{
  ChassisGimbalShooterCoverManual::ctrlZPress();
  if (supply_)
  {
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FALLEN);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  }
  else
  {
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
}

void BalanceManual::shiftRelease()
{
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
}

void BalanceManual::shiftPress()
{
  ChassisGimbalShooterCoverManual::shiftPress();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::UP_SLOPE);
  chassis_cmd_sender_->updateSafetyPower(60);
}

void BalanceManual::vPress()
{
  chassis_cmd_sender_->updateSafetyPower(80);
}

void BalanceManual::bPress()
{
  ChassisGimbalShooterCoverManual::bPress();
  chassis_cmd_sender_->updateSafetyPower(100);
}

void BalanceManual::wPress()
{
  if (flank_)
    flank_ = !flank_;
  if (!supply_)
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  ChassisGimbalShooterCoverManual::wPress();
}

void BalanceManual::wPressing()
{
  if (flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::wPressing();
  if (supply_)
    vel_cmd_sender_->setLinearXVel(x_scale_ * 0.4);
}

void BalanceManual::sPress()
{
  if (flank_)
    flank_ = !flank_;
  if (!supply_)
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  ChassisGimbalShooterCoverManual::sPress();
}

void BalanceManual::sPressing()
{
  if (flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::sPressing();
  if (supply_)
    vel_cmd_sender_->setLinearXVel(x_scale_ * 0.4);
}

void BalanceManual::aPress()
{
  if (!flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::aPress();
}

void BalanceManual::aPressing()
{
  if (!flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::aPressing();
  if (supply_)
    vel_cmd_sender_->setLinearYVel(y_scale_ * 0.4);
}

void BalanceManual::dPress()
{
  if (!flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::dPress();
}

void BalanceManual::dPressing()
{
  if (!flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::dPressing();
  if (supply_)
    vel_cmd_sender_->setLinearYVel(y_scale_ * 0.4);
}

void BalanceManual::cPress()
{
  if (is_gyro_)
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.0);
    is_gyro_ = false;
  }
  else
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    is_gyro_ = true;
    if (x_scale_ != 0.0 || y_scale_ != 0.0)
      vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_);
    else
      vel_cmd_sender_->setAngularZVel(1.0);
  }
}

void BalanceManual::ctrlXPress()
{
  if (balance_cmd_sender_->getMsg()->data == rm_msgs::BalanceState::NORMAL)
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
  else
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
}

void BalanceManual::ctrlQPress()
{
  ChassisGimbalShooterCoverManual::ctrlQPress();
  chassis_calibration_->reset();
}

void BalanceManual::chassisOutputOn()
{
  ChassisGimbalShooterCoverManual::chassisOutputOn();
  chassis_calibration_->reset();
}

void BalanceManual::remoteControlTurnOn()
{
  ChassisGimbalShooterCoverManual::remoteControlTurnOn();
  chassis_calibration_->stopController();
}

void BalanceManual::remoteControlTurnOff()
{
  ChassisGimbalShooterCoverManual::remoteControlTurnOff();
  chassis_calibration_->stop();
}

void BalanceManual::modeNormalize()
{
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
  ROS_INFO("mode normalize");
}

void BalanceManual::modeFallen(ros::Duration duration)
{
  if (duration.toSec() > 0.3)
  {
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
    ROS_INFO("mode fallen");
  }
}
}  // namespace rm_manual
