#ifndef TRAVERSE_LAYER_HPP_
#define TRAVERSE_LAYER_HPP_

#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <nav2_msgs/nav2_msgs/msg/costmap.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mutex>

namespace traverse_layer
{

class TraverseLayer : public rclcpp::Node
{
public:
    TraverseLayer(const rclcpp::NodeOptions& options);
    virtual ~TraverseLayer();

    bool readParameters();

    void callback(grid_map_msgs::msg::GridMap::SharedPtr msg);
    void update_downscaled_map(grid_map::GridMap& input_map);
    void update_map(grid_map::GridMap& input_map);

private:
    // Topic names
    std::string input_topic_;
    std::string local_map_topic_;
    std::string global_map_topic_;
    std::string costmap_topic_;

    // Costmap parameters
    std::string costmap_layer_;
    double costmap_min_;
    double costmap_max_;
    bool publish_costmap_;
    bool publish_latency_;

    // map parameters
    double map_resolution_;
    double map_local_length_;
    double movement_update_threshold_;
    double downscaled_map_resolution_;
    std::string map_frame_id_;
    bool map_initialized_;
    double global_map_publish_length_;

    // saved maps
    grid_map::GridMap map_;
    grid_map::GridMap downscaled_map_;
    std::mutex map_mutex_;
    std::mutex downscaled_map_mutex_;
    grid_map::Position last_position_;

    // Pub/Sub handles
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscriber_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr local_publisher_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr global_publisher_;
    rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_publisher_;

    // latency
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_publisher_;

    // Filter chain
    filters::FilterChain<grid_map::GridMap> filter_chain_;

    // Filter chain parameters name.
    std::string filter_chain_parameter_name_;
};

} // namespace traverse_layer

#endif