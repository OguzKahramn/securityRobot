#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <map>
#include <vector>
#include <utility>
#include <iostream>
#include <fstream>
using std::placeholders::_1;
std::string full_path;

 
class getTargetPoints : public rclcpp::Node 
{
public:
    getTargetPoints() : Node("get_target_points") 
    {
        this->declare_parameter("txt_file_name");
        this->declare_parameter("target_number",3);
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>("clicked_point",10,
        std::bind(&getTargetPoints::topic_callback, this, _1));
        target_number = this->get_parameter("target_number").as_int();
        txt_path = this->get_parameter("txt_file_name").as_string();
        full_path =  main_path + txt_path + ".txt";
        RCLCPP_INFO(this->get_logger(),"Saving target points node has been started ");
    } 
private:
    void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
    {
      RCLCPP_INFO(this->get_logger(), "Your point is saved... X: '%f', Y: '%f'", msg->point.x, msg->point.y);
      targetPoints.push_back(msg->point.x);
      targetPoints.push_back(msg->point.y);

      if (targetPoints.size() == target_number * 2)
      {
        RCLCPP_INFO(this->get_logger(),"Target points are saving into %s..",full_path.c_str());
        std::ofstream myFile; 
        myFile.open(full_path);
        //std::cout<< full_path << std::endl;
        for (int i = 0; i < targetPoints.size(); i+=2) {
            myFile << targetPoints.at(i) << '/' << targetPoints.at(i+1) <<"+"<< "\n";
        }
        myFile.close();
        RCLCPP_INFO(this->get_logger(),"Target points have been saved successfully.");
        rclcpp::shutdown();
      }
      
    }
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    std::vector<double> targetPoints;
    int target_number;
    std::string txt_path;
    std::string main_path = "/home/oguzk/cleaning_ws/src/two_wheeled_robot/config/";
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<getTargetPoints>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
