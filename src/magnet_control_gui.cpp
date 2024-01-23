#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <QWidget>
#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include <QCloseEvent>

using namespace std::chrono_literals;

class MagnetControlGUINode : public rclcpp::Node
{
  public:
    MagnetControlGUINode()
    : Node("magnet_control_gui"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("magnet_enable", 10);
    }
    void sendTopicON()
    {
      auto message = std_msgs::msg::String();
      message.data = "ON";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    void sendTopicOFF()
    {
      auto message = std_msgs::msg::String();
      message.data = "OFF";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

class GUIButtons : public QWidget {
  public:
    GUIButtons(QWidget *parent = nullptr) : 
    QWidget(parent)
    {
        auto *onBtn = new QPushButton("ON", this);
        auto *offBtn = new QPushButton("OFF", this);
        lbl = new QLabel("magnet_enable : OFF", this);
        auto *grid = new QGridLayout(this);
        grid->addWidget(onBtn, 1, 0);
        grid->addWidget(offBtn, 1, 1);
        grid->addWidget(lbl, 2, 0);
        setLayout(grid);
        connect(onBtn, &QPushButton::clicked, this, &GUIButtons::OnON);
        connect(offBtn, &QPushButton::clicked, this, &GUIButtons::OnOFF);
    }
    void closeEvent(QCloseEvent *event){
      if (executor_pointer_ == nullptr){
        return;
      }
      executor_pointer_->cancel();
      rclcpp::shutdown();
      event->accept();
    }
    void setROSExecutor(rclcpp::executors::SingleThreadedExecutor* pointer_){
      executor_pointer_ = pointer_;
    }
    int setROSNode(std::shared_ptr<MagnetControlGUINode> n_){
      if (n_ == nullptr){
        std::cout<<"node is nullptr"<<std::endl;
        return -1;
      }
      n_->sendTopicOFF();
      node_ = n_;
      return 0;
    }
  private slots:
    void OnON(){ 
      if (node_ == nullptr){
        std::cout<<"node is nullptr"<<std::endl;
        return;
      }
      lbl->setText("magnet_enable : ON");
      node_->sendTopicON();
    }
    void OnOFF(){
      if (node_ == nullptr){
        std::cout<<"node is nullptr"<<std::endl;
        return;
      }
      lbl->setText("magnet_enable : OFF");
      node_->sendTopicOFF();
    }
  private:
    QLabel *lbl;
    rclcpp::executors::SingleThreadedExecutor* executor_pointer_;
    std::shared_ptr<MagnetControlGUINode> node_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  
  GUIButtons window;
  window.setROSExecutor(&executor);
  window.resize(640,480);
  window.setWindowTitle("magnet_controller_gui");
  window.show();

  auto node = std::make_shared<MagnetControlGUINode>();
  window.setROSNode(node);
  executor.add_node(node);

  app.exec();
  executor.spin();
  rclcpp::shutdown();
  return 0;
}