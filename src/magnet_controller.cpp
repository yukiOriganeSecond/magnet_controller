/******************************************************
* magnet_conroller.cpp
* This cpp makes "magnet_controller" node.
* The node send GPIO signal to the magnet controller.
*******************************************************/

#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <JetsonGPIO.h>
using std::placeholders::_1;

using namespace std::chrono_literals;

const int CONTROL_PIN = 16; // set magnet control pin as pin 16
const int FEEDBACK_PIN = 18; // set magnet feedback pin (read state of magnet controller) as pin 18
std::string message;

//#define USE_FEEDBACK_PIN  // Please remove this comment out if you receive feedback from magnet controller board.

class MagnetController : public rclcpp::Node
{
  public:
    MagnetController()
    : Node("magnet_controller")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "magnet_enable", 10, std::bind(&MagnetController::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::String>("magnet_feedback", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MagnetController::timer_callback, this));
    }

    ~MagnetController(){
      GPIO::output(CONTROL_PIN, GPIO::LOW); // when close, make magnet off to safety
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      if (msg->data=="ON"){
        GPIO::output(CONTROL_PIN, GPIO::HIGH);
        message = "ON";
      }else{
        GPIO::output(CONTROL_PIN, GPIO::LOW);
        message = "OFF";
      }
    }

    void timer_callback()
    {
      auto feedback_result = std_msgs::msg::String();
#ifdef USE_FEEDBACK_PIN
      if (GPIO::input(FEEDBACK_PIN) == GPIO::LOW){
        feedback_result.data = "ON";
      }else{
        feedback_result.data = "OFF";
      }
      RCLCPP_INFO(this->get_logger(), "Receive topic: '%s', Feedback result: '%s'", message.c_str(), feedback_result.data.c_str());
#else
      feedback_result.data = "none";
      RCLCPP_INFO(this->get_logger(), "Receive topic: '%s'", message.c_str());
#endif
      publisher_->publish(feedback_result);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //printf("here");
  GPIO::setmode(GPIO::BOARD); // refer to the pin number of the 40 pin GPIO header
  GPIO::setup(CONTROL_PIN, GPIO::OUT, GPIO::LOW);
#ifdef USE_FEEDBACK_PIN
  GPIO::setup(FEEDBACK_PIN, GPIO::IN);  // please off the comentout if you use feedback 
#endif
  rclcpp::spin(std::make_shared<MagnetController>());
  rclcpp::shutdown();
  return 0;
}