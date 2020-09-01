#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "autolabor_keyboard_control/keyboard_control_node.h"

namespace autolabor_tool {

KeyboardControl::KeyboardControl():linear_state_(0), angular_state_(0), port_name_(""){
  ros::NodeHandle private_node("~");

  private_node.param("linear_min", linear_min_, 0.2);
  private_node.param("linear_max", linear_max_, 2.0);
  private_node.param("linear_step", linear_step_, 0.2);

  private_node.param("angular_min", angular_min_, 0.5);
  private_node.param("angular_max", angular_max_, 4.0);
  private_node.param("angular_step", angular_step_, 0.2);

  private_node.param("rate", rate_, 10.0);

  linear_scale_ = linear_min_;
  angular_scale_ = angular_min_;
  send_flag_ = true;
}

KeyboardControl::~KeyboardControl(){
  close(fd_);
}

void KeyboardControl::buttonTwistCheck(int value, int& state, int down, int up){
  if (value == 1){
    state += down;
  }else if (value == 0){
    state += up;
  }
}

void KeyboardControl::buttonScaleCheck(int value, double &scale, double step, double limit){
  if (value == 1){
    if (step > 0){
      scale = std::min(scale + step, limit);
    }else{
      scale = std::max(scale + step, limit);
    }
  }
}

void KeyboardControl::parseKeyboard(){
  while (true) {
    read(fd_, &ev_, sizeof(struct input_event));
    if (ev_.type == EV_KEY){
      ROS_DEBUG_STREAM("INFO: [key]: " << ev_.code << ", [value]: " << ev_.value);
      switch (ev_.code) {
      case KEYBOARD_UP:
        buttonTwistCheck(ev_.value, linear_state_, 1, -1);
        break;
      case KEYBOARD_DOWN:
        buttonTwistCheck(ev_.value, linear_state_, -1, 1);
        break;
      case KEYBOARD_LEFT:
        buttonTwistCheck(ev_.value, angular_state_, 1, -1);
        break;
      case KEYBOARD_RIGHT:
        buttonTwistCheck(ev_.value, angular_state_, -1, 1);
        break;
      case KEYBOARD_1:
        buttonScaleCheck(ev_.value, linear_scale_, linear_step_, linear_max_);
        break;
      case KEYBOARD_2:
        buttonScaleCheck(ev_.value, linear_scale_, -linear_step_, linear_min_);
        break;
      case KEYBOARD_3:
        buttonScaleCheck(ev_.value, angular_scale_, angular_step_, angular_max_);
        break;
      case KEYBOARD_4:
        buttonScaleCheck(ev_.value, angular_scale_, -angular_step_, angular_min_);
        break;
      case KEYBOARD_9:
        if (ev_.value == 1){
          send_flag_ = true;
        }
        break;
      case KEYBOARD_0:
        if (ev_.value == 1){
          send_flag_ = false;
        }
        break;
      default:
        break;
      }
    }
  }
}

void KeyboardControl::twistCallback(const ros::TimerEvent &){
  if (send_flag_){
    geometry_msgs::Twist twist;
    twist.linear.x = linear_state_ * linear_scale_;
    twist.angular.z = angular_state_ * angular_scale_;
    twist_pub_.publish(twist);
    ROS_DEBUG_STREAM("linear: " << twist.linear.x << " angular: " << twist.angular.z);
  }
}


bool KeyboardControl::init(){
  const char path[] = "/dev/input/by-path";
  DIR *dev_dir = opendir(path);
  struct dirent *entry;
  if (dev_dir == NULL){
    return false;
  }

  while ((entry = readdir(dev_dir)) != NULL){
    std::string dir_str = entry->d_name;
    if (dir_str.find("event-kbd") < dir_str.length()){
      port_name_ = std::string(path) + "/" + dir_str;
      ROS_INFO_STREAM("INFO: The keyboard port is :" << port_name_);
      break;
    }
  }
  closedir(dev_dir);

  if (port_name_ != ""){
    fd_ = open(port_name_.c_str(), O_RDONLY);
    if (fd_ < 0){
      ROS_ERROR_STREAM("ERROR: Can't Open The Port :" << port_name_);
      return false;
    }else{
      ROS_INFO_STREAM("INFO: Open The Port :" << port_name_);
      return true;
    }
  }else{
    return false;
  }
}

void KeyboardControl::run(){
  if (init()){
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    twist_pub_timer_ = nh_.createTimer(ros::Duration(1.0/rate_), &KeyboardControl::twistCallback, this);
    boost::thread parse_thread(boost::bind(&KeyboardControl::parseKeyboard, this));
    ros::spin();
  }
}
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "keyboard_control_node");
  autolabor_tool::KeyboardControl keyboard_control;
  keyboard_control.run();
  return 0;
}


































