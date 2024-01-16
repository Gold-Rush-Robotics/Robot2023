#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

class Robot : public rclcpp::Node{
    public:
        Robot() : Node("Robot") {
            this->start_light_publisher = this->create_publisher<std_msgs::msg::Bool>("/grr/start_led", 10);
            this->create_wall_timer(500ms, std::bind(&Robot::light_timer, this));
        }
    private:
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> start_light_publisher;
        void light_timer(){
            std_msgs::msg::Bool light_present = std_msgs::msg::Bool();
            this->start_light_publisher->publish(light_present);
        }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
}