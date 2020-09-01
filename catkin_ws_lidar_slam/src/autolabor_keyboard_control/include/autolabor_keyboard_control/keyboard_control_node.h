#ifndef KEYBOARD_CONTROL_NODE_H
#define KEYBOARD_CONTROL_NODE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define KEYBOARD_UP     103
#define KEYBOARD_DOWN   108
#define KEYBOARD_LEFT   105
#define KEYBOARD_RIGHT  106

#define KEYBOARD_1      2
#define KEYBOARD_2      3
#define KEYBOARD_3      4
#define KEYBOARD_4      5
#define KEYBOARD_9      10
#define KEYBOARD_0      11

namespace autolabor_tool {
class KeyboardControl
{
public:
  KeyboardControl();
  ~KeyboardControl();

  void run();
private:
  ros::NodeHandle nh_;
  ros::Publisher twist_pub_;
  ros::Timer twist_pub_timer_;

  int fd_;
  struct input_event ev_;

  int linear_state_, angular_state_;

  std::string port_name_;
  double rate_;
  double linear_scale_, angular_scale_;
  double linear_min_, linear_max_, linear_step_;
  double angular_min_, angular_max_, angular_step_;
  bool send_flag_;

  bool init();
  void parseKeyboard();
  void twistCallback(const ros::TimerEvent&);
  void buttonTwistCheck(int value, int& state, int down, int up);
  void buttonScaleCheck(int value, double& scale, double step, double limit);
};

}

#endif // KEYBOARD_CONTROL_NODE_H
