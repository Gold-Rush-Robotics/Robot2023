#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "bcm2835.h"
#include "backcolorsensor.h"
#include "backcolorsensor.cpp"

using namespace std::chrono_literals;

class Robot : public rclcpp::Node{
    public:
        Robot() : Node("Robot") {
            if (bcm2835_init() != 1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to init BCM2835");
            }
            this->start_light_publisher = this->create_publisher<std_msgs::msg::Bool>("/grr/start_led", 10);
            start_timer = this->create_wall_timer(500ms, std::bind(&Robot::light_timer, this));
            back_color_sensor = std::make_shared<ColorSensor>();
        }
    private:
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> start_light_publisher;
        std::shared_ptr<ColorSensor> back_color_sensor;
        rclcpp::TimerBase::SharedPtr start_timer;
        void light_timer(){
            std_msgs::msg::Bool light_present = std_msgs::msg::Bool();
            RCLCPP_INFO(this->get_logger(), "before light");
            light_present.data = back_color_sensor->is_start_light();
            RCLCPP_INFO(this->get_logger(), "after light");
            RCLCPP_INFO(this->get_logger(), light_present.data ? "Light Present" : "Light missing" );
            this->start_light_publisher->publish(light_present);
        }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
}
