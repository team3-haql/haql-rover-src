#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/GotoSetpoint.hpp"

namespace px4
{

inline constexpr std::string_view px4_waypoint_topic = "";

class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher();
    void publish_waypoint(float x, float y, float z);
private:
};

} // px4 end