#include "ros/ros.h"

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "autolabor_pro1_driver/Encode.h"
#include "dynamic_reconfigure/server.h"
#include "autolabor_pro1_driver/SimAutolaborDriverConfig.h"

#define HEAD_1 0x55;
#define HEAD_2 0xAA;
#define SPEED_MSG_ID 0x01
#define BATTERY_MSG_ID 0x02
#define ERROR_MSG_ID 0xFF

typedef autolabor_pro1_driver::SimAutolaborDriverConfig SimAutolaborDriverConfig;
typedef autolabor_pro1_driver::Encode Encode;

using namespace std;
namespace autolabor_driver {


typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

class SimpleDriver
{
public:
  SimpleDriver();
  ~SimpleDriver();
  bool init();
  void run();

private:
  void parse_msg();
  void cal_pulse(int& current, int& receive, int& delta);
  void distribute_msg(uint8_t msg_type, uint8_t* buffer_data);
  void check(uint8_t* data, size_t len, uint8_t& dest);
  void config_callback(SimAutolaborDriverConfig &config, uint32_t level);
  void send_callback(const ros::TimerEvent&);

  string port_name_;
  int baud_rate_;

  uint8_t msg_seq_;

  boost::system::error_code ec_;
  boost::asio::io_service io_service_;
  serial_port_ptr port_;

  bool run_flag_, parse_flag_;
  int control_rate_, pid_rate_;
  double radio_;
  int send_left_, send_right_, receive_left_, receive_right_, delta_left_, delta_right_, current_left_, current_right_;

  dynamic_reconfigure::Server<SimAutolaborDriverConfig> server_;
  dynamic_reconfigure::Server<SimAutolaborDriverConfig>::CallbackType f_;

  ros::Publisher receive_pub_, send_pub_;
  ros::Timer send_timer_;
};

SimpleDriver::SimpleDriver(){
  ros::NodeHandle private_node("~");
  private_node.param<std::string>("port_name", port_name_, std::string("/dev/ttyUSB0"));
  private_node.param<int>("baud_rate", baud_rate_, 9600);
  private_node.param<int>("control_rate", control_rate_, 10);
  private_node.param<int>("pid_rate", pid_rate_, 50);
  radio_ = 1.0 * pid_rate_ / control_rate_;
}

SimpleDriver::~SimpleDriver(){
  parse_flag_ = false;
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();
}

bool SimpleDriver::init(){
  if (port_) {
    cout << "error : port is already opened..." << endl;
    return false;
  }
  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, ec_);
  if (ec_) {
    cout << "error : port_->open() failed...com_port_name="
         << port_name_ << ", e=" << ec_.message().c_str() << endl;
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

void SimpleDriver::run(){
  ros::NodeHandle node;
  if (init()){
    f_ = boost::bind(&SimpleDriver::config_callback, this, _1, _2);
    server_.setCallback(f_);

    send_pub_ = node.advertise<Encode>("send_encode", 10);
    receive_pub_ = node.advertise<Encode>("receive_encode", 10);
    send_timer_ = node.createTimer(ros::Duration(1.0/10), &SimpleDriver::send_callback, this);
    boost::thread parse_thread(boost::bind(&SimpleDriver::parse_msg, this));
    ros::spin();
  }
}

void SimpleDriver::config_callback(SimAutolaborDriverConfig &config, uint32_t level) {
  send_left_ = config.left_wheel;
  send_right_ = config.right_wheel;
  run_flag_ = config.run_flag;
}

void SimpleDriver::check(uint8_t* data, size_t len, uint8_t& dest){
  dest = 0x00;
  for (int i=0; i<len; i++){
    dest = dest ^ *(data + i);
  }
}

void SimpleDriver::parse_msg(){
  uint8_t msg_type, payload_length, state, check_num, buffer_data[255];
  parse_flag_ = true;
  while (parse_flag_){
    switch(state){
    case 0:{ // header 1
      check_num = 0x00;
      boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
      state = buffer_data[0] == 0x55 ? 1 : 0;
      if (state == 0){
        std::cout << "parse error 1 : ->" << (int)buffer_data[0] << std::endl;
      }
      break;
    }case 1:{ // header 2
      boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[1], 1), ec_);
      state = buffer_data[1] == 0xAA ? 2 : 0;
      if (state == 0){
        std::cout << "parse error 2 : ->" << (int)buffer_data[1] << std::endl;
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

void SimpleDriver::cal_pulse(int &current, int &receive, int &delta){
  if (receive > current){
    delta = (receive - current) < (current - receive + 65535) ? (receive - current) : (receive - current - 65535);
  }else{
    delta = (current - receive) < (receive - current + 65535) ? (receive - current) : (receive - current + 65535);
  }
  current = receive;
}

void SimpleDriver::distribute_msg(uint8_t msg_type, uint8_t* buffer_data){
  switch (msg_type) {
  case SPEED_MSG_ID:{
    receive_left_ = buffer_data[5] * 256 + buffer_data[6];
    receive_right_ = buffer_data[7] * 256 + buffer_data[8];

    cal_pulse(current_left_, receive_left_, delta_left_);
    cal_pulse(current_right_, receive_right_, delta_right_);

    Encode receive;
    receive.left = (int)(delta_left_ / radio_);
    receive.right = (int)(delta_right_ / radio_);
    receive_pub_.publish(receive);
    break;
  }case ERROR_MSG_ID:{
    std::cout << "reset!" << std::endl;
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

void SimpleDriver::send_callback(const ros::TimerEvent&){
  if (run_flag_){
    uint8_t data[14] = {0x55, 0xAA, 0x09, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    data[3] = msg_seq_++;
    data[5] = (send_left_ >> 8) & 0xff;
    data[6] = send_left_ & 0xff;
    data[7] = (send_right_ >> 8) & 0xff;
    data[8] = send_right_ & 0xff;
    check(data, 13, data[13]);
    boost::asio::write(*port_.get(), boost::asio::buffer(data, 14), ec_);
    Encode encode;
    encode.left = send_left_;
    encode.right = send_right_;
    send_pub_.publish(encode);
  }else{
    uint8_t data[14] = {0x55, 0xAA, 0x09, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    data[3] = msg_seq_++;
    check(data, 13, data[13]);
    boost::asio::write(*port_.get(), boost::asio::buffer(data, 14), ec_);
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_autolabor_driver");
  ROS_INFO("Spinning node");
  autolabor_driver::SimpleDriver sim_driver;
  sim_driver.run();
  return 0;
}
