#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <string>
#include <vector>

#define HEAD_1 0x55;
#define HEAD_2 0xAA;
#define SPEED_MSG_ID 0x01
#define BATTERY_MSG_ID 0x02
#define CURRENT_MSG_ID 0X07
#define VOLTAGE_MSG_ID 0x08
#define ERROR_MSG_ID 0xFF

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

class ChassisDriver
{
private:
  bool init();

  void parse_msg();

  void twist_callback(const geometry_msgs::Twist::ConstPtr& msg);

  void send_speed_callback(const ros::TimerEvent&);
  void ask_battery_remainder_callback(const ros::TimerEvent&);
  void ask_voltage_callback(const ros::TimerEvent&);
  void ask_current_callback(const ros::TimerEvent&);

  void distribute_msg(uint8_t msg_type, uint8_t* buffer_data);

  void handle_speed_msg(uint8_t* buffer_data);
  void handle_battery_remainder_msg(uint8_t* buffer_data);
  void handle_voltage_msg(uint8_t* buffer_data);
  void handle_current_msg(uint8_t* buffer_data);
  void handle_error_msg();

  void cal_pulse(int& current, int& receive, int& delta);
  void check(uint8_t* data, size_t len, uint8_t& dest);
  std::string print_hex(uint8_t* data, int length);
public:
  ChassisDriver();
  ~ChassisDriver();

  void run();
private:
  bool parse_flag_;
  uint8_t msg_seq_;

  boost::mutex twist_mutex_;
  geometry_msgs::Twist current_twist_;
  ros::Time last_twist_time_;
  nav_msgs::Odometry odom_;

  boost::system::error_code ec_;
  boost::asio::io_service io_service_;
  serial_port_ptr port_;
  boost::mutex mutex_;

  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::TransformStamped transformStamped_;

  ros::Publisher odom_pub_, battery_pub_, current_pub_, voltage_pub_;
  ros::Time last_time_, now_;

  std::string port_name_;
  int baud_rate_;

  std::string odom_frame_, base_frame_;

  int control_rate_, sensor_rate_;

  double maximum_encoding_;
  double pulse_per_cycle_, encoder_resolution_, reduction_ratio_, pid_rate_;
  double model_param_cw_, model_param_acw_, wheel_diameter_;

  bool start_flag_;
  double delta_time_;
  double accumulation_x_, accumulation_y_, accumulation_th_;
  int cur_left_, cur_right_, rev_left_, rev_right_, delta_left_, delta_right_;
};

ChassisDriver::ChassisDriver():msg_seq_(0), start_flag_(true),cur_left_(0),cur_right_(0){}

ChassisDriver::~ChassisDriver(){
  boost::mutex::scoped_lock look(mutex_);
  parse_flag_ = false;
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();
}

bool ChassisDriver::init(){
  if (port_) {
    ROS_ERROR("error : port is already opened...");
    return false;
  }
  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, ec_);
  if (ec_) {
    ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
    return false;
  }
  // option settings...
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  port_->set_option(boost::asio::serial_port_base::character_size(8));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  return true;
}

/////////////////
/// 消息请求函数
/////////////////

void ChassisDriver::send_speed_callback(const ros::TimerEvent&){
  double left_d, right_d, radio;
  double model_param;
  short left, right;

  double linear_speed, angular_speed;
  if ((ros::Time::now() - last_twist_time_).toSec()<=1.0){
    linear_speed = current_twist_.linear.x;
    angular_speed = current_twist_.angular.z;
  }else{
    linear_speed = 0;
    angular_speed = 0;
  }

  if (angular_speed <= 0){
    model_param = model_param_cw_;
  }else{
    model_param = model_param_acw_;
  }

  left_d = (linear_speed - model_param/2 * angular_speed) * pulse_per_cycle_;
  right_d = (linear_speed + model_param/2 * angular_speed) * pulse_per_cycle_;

  radio = std::max(std::max(std::abs(left_d), std::abs(right_d)) / maximum_encoding_, 1.0);

  left = static_cast<short>(left_d / radio);
  right = static_cast<short>(right_d / radio);

  uint8_t data[14] = {0x55, 0xAA, 0x09, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  data[3] = msg_seq_++;
  data[5] = (left >> 8) & 0xff;
  data[6] = left & 0xff;
  data[7] = (right >> 8) & 0xff;
  data[8] = right & 0xff;
  check(data, 13, data[13]);
  boost::asio::write(*port_.get(), boost::asio::buffer(data, 14), ec_);
  ROS_DEBUG_STREAM("send -> left: " << left << "; right: " << right);
}

void ChassisDriver::ask_battery_remainder_callback(const ros::TimerEvent&){
  uint8_t data[7] = {0x55, 0xAA, 0x02, 0x00, 0x02, 0x00, 0x00};
  data[3] = msg_seq_++;
  check(data, 6, data[6]);
  boost::asio::write(*port_.get(), boost::asio::buffer(data, 7), ec_);
}

void ChassisDriver::ask_current_callback(const ros::TimerEvent&){
  uint8_t data[7] = {0x55, 0xAA, 0x02, 0x00, 0x07, 0x00, 0x00};
  data[3] = msg_seq_++;
  check(data, 6, data[6]);
  boost::asio::write(*port_.get(), boost::asio::buffer(data, 7), ec_);
}

void ChassisDriver::ask_voltage_callback(const ros::TimerEvent&){
  uint8_t data[7] = {0x55, 0xAA, 0x02, 0x00, 0x08, 0x00, 0x00};
  data[3] = msg_seq_++;
  check(data, 6, data[6]);
  boost::asio::write(*port_.get(), boost::asio::buffer(data, 7), ec_);
}

void ChassisDriver::twist_callback(const geometry_msgs::Twist::ConstPtr& msg){
  twist_mutex_.lock();
  last_twist_time_ = ros::Time::now();
  current_twist_ = *msg.get();
  twist_mutex_.unlock();
}

/////////////////
/// 消息分发函数
/////////////////

void ChassisDriver::distribute_msg(uint8_t msg_type, uint8_t* buffer_data){
  switch (msg_type) {
  case SPEED_MSG_ID:{
    handle_speed_msg(buffer_data);
    break;
  }case BATTERY_MSG_ID:{
    handle_battery_remainder_msg(buffer_data);
    break;
  }case CURRENT_MSG_ID:{
    handle_current_msg(buffer_data);
    break;
  }case VOLTAGE_MSG_ID:{
    handle_voltage_msg(buffer_data);
    break;
  }case ERROR_MSG_ID:{
    ROS_DEBUG_STREAM("RESET -> error_code " << (int)buffer_data[5]);
    uint8_t data[7] = {0x55, 0xAA, 0x02, 0x00, 0x05, 0x00, 0x00};
    data[3] = msg_seq_++;
    check(data, 6, data[6]);
    boost::asio::write(*port_.get(), boost::asio::buffer(data, 7), ec_);
    break;
  }default:{
    break;
  }
  }
}

/////////////////
/// 消息处理函数
/////////////////

void ChassisDriver::handle_current_msg(uint8_t *buffer_data){
  std_msgs::Float32 current;
  current.data = ((float)(buffer_data[5]*256 + buffer_data[6])) / 1000.0;
  current_pub_.publish(current);
}

void ChassisDriver::handle_voltage_msg(uint8_t *buffer_data){
  std_msgs::Float32 voltage;
  voltage.data = ((float)(buffer_data[5]*256 + buffer_data[6])) / 1000.0;
  voltage_pub_.publish(voltage);
}

void ChassisDriver::handle_battery_remainder_msg(uint8_t *buffer_data){
  std_msgs::Int32 battery;
  battery.data = (int)buffer_data[5];
  battery_pub_.publish(battery);
}

void ChassisDriver::handle_speed_msg(uint8_t* buffer_data){
  rev_left_ = buffer_data[5] * 256 + buffer_data[6];
  rev_right_ = buffer_data[7] * 256 + buffer_data[8];

  cal_pulse(cur_left_, rev_left_, delta_left_);
  cal_pulse(cur_right_, rev_right_, delta_right_);

  ROS_DEBUG_STREAM("receive -> left: " << delta_left_ << "(" << rev_left_ << ")" << "; right: " << delta_right_ << "(" << rev_right_ << ")");

  now_ = ros::Time::now();
  if (start_flag_){
    accumulation_x_ = accumulation_y_ = accumulation_th_ = 0.0;
    last_time_ = now_;
    start_flag_ = false;
    return;
  }
  delta_time_ = (now_ - last_time_).toSec();
  if (delta_time_ >= (0.5 / control_rate_)){
    double model_param;
    if (delta_right_ <= delta_left_){
      model_param = model_param_cw_;
    }else{
      model_param = model_param_acw_;
    }
    double delta_theta = (delta_right_ - delta_left_)/ (pulse_per_cycle_ * pid_rate_ * model_param);
    double v_theta = delta_theta / delta_time_;

    double delta_dis = (delta_right_ + delta_left_) / (pulse_per_cycle_ * pid_rate_ * 2.0);
    double v_dis = delta_dis / delta_time_;

    double delta_x, delta_y;
    if (delta_theta == 0){
      delta_x = delta_dis;
      delta_y = 0.0;
    }else{
      delta_x = delta_dis * (sin(delta_theta) / delta_theta);
      delta_y = delta_dis * ( (1 - cos(delta_theta)) / delta_theta );
    }

    accumulation_x_ += (cos(accumulation_th_) * delta_x - sin(accumulation_th_) * delta_y);
    accumulation_y_ += (sin(accumulation_th_) * delta_x + cos(accumulation_th_) * delta_y);
    accumulation_th_ += delta_theta;

    transformStamped_.header.stamp = ros::Time::now();
    transformStamped_.header.frame_id = odom_frame_;
    transformStamped_.child_frame_id = base_frame_;
    transformStamped_.transform.translation.x = accumulation_x_;
    transformStamped_.transform.translation.y = accumulation_y_;
    transformStamped_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, accumulation_th_);
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();

    br_.sendTransform(transformStamped_);

    odom_.header.frame_id = odom_frame_;
    odom_.child_frame_id = base_frame_;
    odom_.header.stamp = now_;
    odom_.pose.pose.position.x = accumulation_x_;
    odom_.pose.pose.position.y = accumulation_y_;
    odom_.pose.pose.position.z = 0;
    odom_.pose.pose.orientation.x = q.getX();
    odom_.pose.pose.orientation.y = q.getY();
    odom_.pose.pose.orientation.z = q.getZ();
    odom_.pose.pose.orientation.w = q.getW();
    odom_.twist.twist.linear.x = v_dis;
    odom_.twist.twist.linear.y = 0;
    odom_.twist.twist.angular.z = v_theta;

    odom_pub_.publish(odom_);

    ROS_DEBUG_STREAM("accumulation_x: " << accumulation_x_ << "; accumulation_y: " << accumulation_y_ <<"; accumulation_th: " << accumulation_th_);
  }
  last_time_ = now_;
}


/////////////////
/// 工具函数
/////////////////

std::string ChassisDriver::print_hex(uint8_t* data, int length){
  std::string output ="";
  static char hex[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
  for (int i=0; i<length; i++){
    output += hex[(*(data+i) >> 4) & 0xf] + hex[*(data+i) & 0xf] + " ";
  }
  return output;
}

void ChassisDriver::cal_pulse(int &current, int &receive, int &delta){
  if (receive > current){
    delta = (receive - current) < (current - receive + 65535) ? (receive - current) : (receive - current - 65535);
  }else{
    delta = (current - receive) < (receive - current + 65535) ? (receive - current) : (receive - current + 65535);
  }
  current = receive;
}

void ChassisDriver::parse_msg(){
  uint8_t msg_type, payload_length, state, check_num, buffer_data[255];
  parse_flag_ = true;
  while (parse_flag_){
    switch(state){
    case 0:{ // header 1
      check_num = 0x00;
      boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
      state = buffer_data[0] == 0x55 ? 1 : 0;
      if (state == 0){
        ROS_DEBUG_STREAM("parse error 1 : ->" << (int)buffer_data[0]);
      }
      break;
    }case 1:{ // header 2
      boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[1], 1), ec_);
      state = buffer_data[1] == 0xAA ? 2 : 0;
      if (state == 0){
        ROS_DEBUG_STREAM("parse error 2 : ->" << (int)buffer_data[1]);
      }
      break;
    }case 2:{ // length
      boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[2], 1), ec_);
      payload_length = buffer_data[2];
      state = 3;
      break;
    }case 3:{ // seq
      boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[3], 1), ec_);
      state = 4;
      break;
    }case 4:{ // payload
      boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[4], payload_length), ec_);
      msg_type = buffer_data[4];
      state = 5;
      break;
    }case 5:{ // check
      boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[4 + payload_length], 1), ec_);
      check(buffer_data, 4 + payload_length, check_num);
      state = buffer_data[4 + payload_length] == check_num ? 6 : 0;
      if (state == 0){
        ROS_DEBUG_STREAM("parse error 3 : ->" << print_hex(buffer_data, 5 + payload_length));
      }
      break;
    }case 6:{ // handle
      distribute_msg(msg_type, buffer_data);
      state = 0;
      break;
    }default:{
      state = 0;
      break;
    }
    }
  }
}

void ChassisDriver::check(uint8_t* data, size_t len, uint8_t& dest){
  dest = 0x00;
  for (int i=0; i<len; i++){
    dest = dest ^ *(data + i);
  }
}

/////////////////
/// 主执行函数
/////////////////

void ChassisDriver::run(){
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  private_node.param<std::string>("port_name", port_name_, std::string("/dev/ttyUSB0"));
  private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
  private_node.param<std::string>("base_frame", base_frame_, std::string("base_link"));

  private_node.param<int>("baud_rate", baud_rate_, 115200);
  private_node.param<int>("control_rate", control_rate_, 10);
  private_node.param<int>("sensor_rate", sensor_rate_, 5);

  private_node.param<double>("reduction_ratio", reduction_ratio_, 2.5);
  private_node.param<double>("encoder_resolution", encoder_resolution_, 1600.0);
  private_node.param<double>("wheel_diameter", wheel_diameter_, 0.15);
  private_node.param<double>("model_param_cw", model_param_cw_, 0.78);
  private_node.param<double>("model_param_acw", model_param_acw_, 0.78);
  private_node.param<double>("pid_rate", pid_rate_, 50.0);

  private_node.param<double>("maximum_encoding", maximum_encoding_, 32.0);

  // 速度1m/s的情况下每个控制周期的脉冲数
  pulse_per_cycle_ = reduction_ratio_ * encoder_resolution_ / (M_PI * wheel_diameter_ * pid_rate_);

  if (init()){
    odom_pub_ = node.advertise<nav_msgs::Odometry>("wheel_odom", 10);
    battery_pub_ = node.advertise<std_msgs::Int32>("remaining_battery", 1);
    current_pub_ = node.advertise<std_msgs::Float32>("current", 1);
    voltage_pub_ = node.advertise<std_msgs::Float32>("voltage",1);
    ros::Subscriber cmd_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &ChassisDriver::twist_callback, this);
    ros::Timer send_speed_timer = node.createTimer(ros::Duration(1.0/control_rate_), &ChassisDriver::send_speed_callback, this);
    ros::Timer ask_battery_remainder_timer = node.createTimer(ros::Duration(1.0/sensor_rate_), &ChassisDriver::ask_battery_remainder_callback, this);
    ros::Timer ask_current_timer = node.createTimer(ros::Duration(1.0/sensor_rate_), &ChassisDriver::ask_current_callback, this);
    ros::Timer ask_voltage_timer = node.createTimer(ros::Duration(1.0/sensor_rate_), &ChassisDriver::ask_voltage_callback, this);
    boost::thread parse_thread(boost::bind(&ChassisDriver::parse_msg, this));
    ros::spin();
    return;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "autolabor_driver");
  ChassisDriver driver;
  driver.run();
  return 0;
}
