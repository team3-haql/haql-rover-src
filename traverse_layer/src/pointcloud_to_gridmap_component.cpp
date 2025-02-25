#include <memory>
#include <string>
#include <utility>
#include <string>

#include <traverse_layer/pointcloud_to_gridmap_component.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>  
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <chrono>

// This script heavily uses the GridMaps class.
// Documentation can be found here: https://github.com/ANYbotics/grid_map/tree/humble

namespace traverse_layer
{

PointcloudToGridmap::PointcloudToGridmap(const rclcpp::NodeOptions & options) : 
    Node("pointcloud_to_gridmap", options),
    raw_map_({
        "elevation", "variance", "time", "lowest_scan_point", 
        "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", 
        "sensor_z_at_lowest_scan"
    }),
    map_({"elevation", "lower_bound", "upper_bound"}),
    min_variance_(0.003 * 0.003),
    // max_variance_(0.03 * 0.03),
    max_variance_(100.0),
    mahalanobis_distance_threshold_(2.5),
    multi_height_noise_(0.00002),
    scanning_duration_(1.0),
    enable_visibility_cleanup_(true),
    visibility_cleanup_rate_(1.0)
{
    // Initialize node variables as well as class variables
    if (!read_parameters()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read parameters.");
        rclcpp::shutdown();
        return;
    }

    initial_time_ = this->get_clock()->now();

    // Set grid map parameters
    grid_map::Length mapLength(world_length_, world_width_);
    grid_map::Position mapPosition(0.0, 0.0);

    raw_map_.setBasicLayers({"elevation", "variance"});
    raw_map_.setGeometry(mapLength, resolution_, mapPosition);
    raw_map_.setFrameId(map_frame_id_);
    map_.setBasicLayers({"elevation", "lower_bound", "upper_bound"});
    map_.setGeometry(mapLength, resolution_, mapPosition);
    map_.setFrameId(map_frame_id_);

    RCLCPP_INFO(
        this->get_logger(),
        "Created map with size %f x %f m (%i x %i cells).",
        map_.getLength().x(), map_.getLength().y(),
        map_.getSize()(0), map_.getSize()(1)
    );

    std::chrono::duration<int> buffer_timeout(1);
    // TF Buffer and listener
    // Buffer stores cache and deletes it over time.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // Create time interface before call to wait for transform
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    tf_buffer_->setCreateTimerInterface(timer_interface);

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscriber_.subscribe(this, input_topic_);
    tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        subscriber_, 
        *tf_buffer_, 
        map_frame_id_, 
        100,
        this->get_node_logging_interface(),
        this->get_node_clock_interface(), 
        buffer_timeout
    );

    tf_filter_->registerCallback(&PointcloudToGridmap::add_sensor_data, this);

    // subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     input_topic_, rclcpp::SensorDataQoS(),
    //     std::bind(&PointcloudToGridmap::callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        output_topic_, // Topic defined elsewhere
        static_cast<rclcpp::QoS>(5)
    );

    // Calls publish pointcloud every x milliseconds.
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)), // Period
        std::bind(&PointcloudToGridmap::publish_pointcloud, this) // Calls publish pointcloud on this object.
    );

    if (enable_visibility_cleanup_) {
        RCLCPP_INFO(
            this->get_logger(), 
            "Starting visibility cleanup timer"
        );

        // Call visibility clean up every x seconds.
        visibility_cleanup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / visibility_cleanup_rate_)),
            std::bind(&PointcloudToGridmap::visibility_cleanup, this)
        );
    }

    latency_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "point_to_grid/latency", // Topic
        rclcpp::QoS(1).durability_volatile() // Can only hold one message in queue while publishing, volatile
    );
}

PointcloudToGridmap::~PointcloudToGridmap() {
    RCLCPP_INFO(this->get_logger(), "PointcloudToGridmap node has been stopped.");
}


bool PointcloudToGridmap::read_parameters() {
    // Sets parameters as well as default values for node object
    this->declare_parameter<std::string>("input_topic", "input_topic");
    this->declare_parameter<std::string>("output_topic", "output_topic");
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<std::string>("center_frame_id", "base_link");
    this->declare_parameter<double>("resolution", 0.03);
    this->declare_parameter<double>("world_size.length", 10.0);
    this->declare_parameter<double>("world_size.width", 10.0);
    this->declare_parameter<double>("publish_rate", 1.0);
    this->declare_parameter<bool>("publish_latency", false);
    this->declare_parameter<bool>("visibility_cleanup.enabled", false);
    this->declare_parameter<double>("visibility_cleanup.rate", 1.0);

    // Store input topic in input_topic_, if not there log error
    if (!this->get_parameter("input_topic", input_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get input_topic.");
        return false;
    }
    this->get_parameter("output_topic", output_topic_);

    // Log input and output topics
    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());

    // Move values defined above into designated variable
    if (!this->get_parameter("map_frame_id", map_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get map_frame_id.");
        return false;
    }

    if (!this->get_parameter("resolution", resolution_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get resolution.");
        return false;
    }

    if (!this->get_parameter("world_size.length", world_length_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get world_size.length.");
        return false;
    }

    if (!this->get_parameter("world_size.width", world_width_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get world_size.width.");
        return false;
    }

    if (!this->get_parameter("publish_rate", publish_rate_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get publish_rate.");
        return false;
    }

    if (!this->get_parameter("publish_latency", publish_latency_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get publish_latency.");
        return false;
    }

    if (!this->get_parameter("center_frame_id", center_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get center_frame_id.");
        return false;
    }
    
    this->get_parameter("visibility_cleanup.enabled", enable_visibility_cleanup_);
    this->get_parameter("visibility_cleanup.rate", visibility_cleanup_rate_);

    return true;
}

void PointcloudToGridmap::update_map_from_raw() {
    auto method_start = std::chrono::high_resolution_clock::now();

    for (grid_map::GridMapIterator iterator(raw_map_); !iterator.isPastEnd(); ++iterator) {
        grid_map::Index index = {0, 0};
        grid_map::Position position = {0.0f, 0.0f};

        // Position is an out
        if (!raw_map_.getPosition(*iterator, position)) {
            continue; // skip this point outside of map
        }

        // Index is an out
        if (!map_.getIndex(position, index)) {
            continue; // skip if outside of map
        }

        // Get elevation and variance of point
        double elevation = raw_map_.at("elevation", *iterator);
        double variance = raw_map_.at("variance", *iterator);
        double sigma = sqrt(variance);
        map_.at("elevation", index) = elevation;
        // Set range of what the elevation is
        map_.at("upper_bound", index) = elevation + 2 * sigma;
        map_.at("lower_bound", index) = elevation - 2 * sigma;
    }

    // Log time taken to compute
    std::chrono::duration<double> method_duration = std::chrono::high_resolution_clock::now() - method_start;
    RCLCPP_DEBUG(
        this->get_logger(), 
        "update_map_from_raw took %f ms", 
        method_duration.count() * 1000.0
    );
}

void PointcloudToGridmap::update_map_center() {
    try {
        geometry_msgs::msg::TransformStamped transform;
        // If this fails send warning
        transform = tf_buffer_->lookupTransform(
            map_.getFrameId(), 
            center_frame_id_,
            tf2::TimePointZero
        );
        grid_map::Position newCenter(
            transform.transform.translation.x,
            transform.transform.translation.y
        );
        // Move changes the position of the map frame as 
        // well as wipes out data that is out of bounds.
        map_.move(newCenter);
        raw_map_.move(newCenter);
    } 
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Failed to center map. Could not find transform from %s to %s: %s",
            map_.getFrameId().c_str(), 
            center_frame_id_.c_str(), 
            ex.what()
        );
        return;
    }
}

void PointcloudToGridmap::publish_pointcloud() {
    // Prevents other threads from accessing the raw map and the map
    std::lock_guard<std::mutex> raw_lock(raw_map_mutex_);
    std::lock_guard<std::mutex> map_lock(map_mutex_);
    // RCLCPP_INFO(this->get_logger(), "Publishing grid map.");
    update_map_from_raw();

    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map_);
    // Sends to subscribers
    publisher_->publish(std::move(message));

    update_map_center();
}

void PointcloudToGridmap::add_sensor_data(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert point cloud to grid map frame
    sensor_msgs::msg::PointCloud2 cloud;
    geometry_msgs::msg::TransformStamped transform;

    // Prevent other threads from accessing raw map
    std::lock_guard<std::mutex> lock(raw_map_mutex_);

    auto start = std::chrono::high_resolution_clock::now();

    // transform pointcloud to map frame
    try {
        tf_buffer_->transform(*msg, cloud, map_.getFrameId());
        transform = tf_buffer_->lookupTransform(
            map_.getFrameId(), 
            msg->header.frame_id,
            tf2::TimePointZero
        );
    } 
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Could not transform point cloud from frame %s to %s: %s",
            msg->header.frame_id.c_str(), 
            map_.getFrameId().c_str(), 
            ex.what()
        );
        return;
    }

    const rclcpp::Time current_time = this->get_clock()->now();
    // Time since class was created
    const double time_since_start = (current_time - initial_time_).seconds();

    auto& elevation_layer = raw_map_["elevation"];
    auto& variance_layer = raw_map_["variance"];
    auto& time_layer = raw_map_["time"];
    auto& lowest_scan_point_layer = raw_map_["lowest_scan_point"];
    auto& sensor_x_at_lowest_scan_layer = raw_map_["sensor_x_at_lowest_scan"];
    auto& sensor_y_at_lowest_scan_layer = raw_map_["sensor_y_at_lowest_scan"];
    auto& sensor_z_at_lowest_scan_layer = raw_map_["sensor_z_at_lowest_scan"];

    // create cloudpoint2 iterator
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    // iterate through point cloud
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        double px = *iter_x, py = *iter_y, pz = *iter_z;

        // Get in relation to curr position
        double dx = px - transform.transform.translation.x;
        double dy = py - transform.transform.translation.y;
        double dz = pz - transform.transform.translation.z;

        grid_map::Index index;
        grid_map::Position position(px, py); // Get position in map

        if (!raw_map_.getIndex(position, index)) { // Get index of map
            continue; // skip this point outside of map
        }

        double distance_squared = dx * dx + dy * dy;

        const double MIN_DIST = 0.40;
        if (distance_squared < (MIN_DIST * MIN_DIST)) {
            continue; // skip this point too close to sensor
        }

        double R = distance_squared * 0.006518; // sensor covariance model for zed 2i camera
        R = R * R; // variance

        // Get info about map at point
        auto& elevation = elevation_layer(index(0), index(1));
        auto& variance = variance_layer(index(0), index(1));
        auto& time = time_layer(index(0), index(1));
        auto& lowest_scan_point = lowest_scan_point_layer(index(0), index(1));
        auto& sensor_x_at_lowest_scan = sensor_x_at_lowest_scan_layer(index(0), index(1));
        auto& sensor_y_at_lowest_scan = sensor_y_at_lowest_scan_layer(index(0), index(1));
        auto& sensor_z_at_lowest_scan = sensor_z_at_lowest_scan_layer(index(0), index(1));

        // If no info about this position
        if (std::isnan(elevation)) {
            elevation = pz;
            variance = R;
            time = time_since_start;
            continue;
        }

        // mahalanobis_distance = how many standard deviations from the mean this is
        const double mahalanobis_distance = std::abs(pz - elevation) / sqrt(variance);
        if (mahalanobis_distance > mahalanobis_distance_threshold_) {
            // Only add points one second after grid point initialization
            if (time_since_start - time <= scanning_duration_) {
                // Takes the worst case scenerio for the elevation, this is the cause of the ceiling error
                if (pz > elevation) {
                    // set point to heigher elevation.
                    elevation = pz;
                    variance = R;
                }
            } 
            else {
                // Adding a small amount to variance when its past scanning time?
                // Idk why you would do this, wouldnt this cause issues the longer the rover stayed in one spot?
                // IE, increases spread of data by a constant amount everytime a point is scanned when past scanning time.
                // TODO: See if this should be removed in testing.
                variance += multi_height_noise_;
            }
            continue;
        }

        // If point is past threshold:
        // point + 3 sigma
        const double point_height_plus_uncertainty = pz + 3.0 * sqrt(variance);
        if (std::isnan(lowest_scan_point) || point_height_plus_uncertainty < lowest_scan_point) {
            lowest_scan_point = point_height_plus_uncertainty;
            sensor_x_at_lowest_scan = transform.transform.translation.x;
            sensor_y_at_lowest_scan = transform.transform.translation.y;
            sensor_z_at_lowest_scan = transform.transform.translation.z;
        }
        
        // fuse data into map
        elevation = (variance * pz + R * elevation) / (variance + R);
        variance = (variance * R) / (variance + R);
        time = time_since_start;
    }


    // clean map
    for (grid_map::GridMapIterator iterator(raw_map_); !iterator.isPastEnd(); ++iterator) {
        auto& variance = raw_map_.at("variance", *iterator);
        // Clamps variance for each point of the map
        if (!std::isnan(variance)) {
            variance = variance < min_variance_ ? min_variance_ : variance;
            variance = variance > max_variance_ ? max_variance_ : variance;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    RCLCPP_DEBUG(this->get_logger(), "add_sensor_data took %f ms", duration.count() / 1000.0);

    if (publish_latency_) {
        std_msgs::msg::Float64 latency;
        latency.data = duration.count() / 1000.0;
        latency_publisher_->publish(latency);
    }
}

void PointcloudToGridmap::visibility_cleanup() {
    const rclcpp::Time current_time = this->get_clock()->now();
    const double time_since_start = (current_time - initial_time_).seconds();

    std::lock_guard<std::mutex> lock(raw_map_mutex_);

    // max height used for cleanup.
    raw_map_.add("max_height");

    // create max height layer with ray tracing
    for (grid_map::GridMapIterator iterator(raw_map_); !iterator.isPastEnd(); ++iterator) {
        if (!raw_map_.isValid(*iterator)) {
            continue;
        }
        auto& lowest_scan_point = raw_map_.at("lowest_scan_point", *iterator);
        auto& sensor_x = raw_map_.at("sensor_x_at_lowest_scan", *iterator);
        auto& sensor_y = raw_map_.at("sensor_y_at_lowest_scan", *iterator);
        auto& sensor_z = raw_map_.at("sensor_z_at_lowest_scan", *iterator);

        if (std::isnan(lowest_scan_point)) {
            continue;
        }

        grid_map::Index index_at_sensor;
        if (!raw_map_.getIndex(grid_map::Position(sensor_x, sensor_y), /* out */ index_at_sensor)) {
            continue;
        }

        grid_map::Position point;
        raw_map_.getPosition(*iterator, /* out */ point);
        // Difference between the camera and the point scanned.
        double point_dx = point.x() - sensor_x;
        double point_dy = point.y() - sensor_y;
        double point_distance = sqrt(point_dx * point_dx + point_dy * point_dy);

        if (point_distance > 0.0) {
            // ray trace to find max height
            for (grid_map::LineIterator iterator(raw_map_, index_at_sensor, *iterator); !iterator.isPastEnd(); ++iterator) {
                grid_map::Position cell_position;
                raw_map_.getPosition(*iterator, cell_position);

                // Difference between camera position and the cell position
                double cell_dx = cell_position.x() - sensor_x;
                double cell_dy = cell_position.y() - sensor_y;
                double cell_distance = point_distance - sqrt(cell_dx * cell_dx + cell_dy * cell_dy);

                // TODO: What does this equation do?
                double max_height = lowest_scan_point + (sensor_z - lowest_scan_point) / point_distance * cell_distance;
                auto& cell_max_height = raw_map_.at("max_height", *iterator);
                if (std::isnan(cell_max_height) || cell_max_height > max_height) {
                    cell_max_height = max_height;
                }
            }
        }
    }

    // clean map
    // Removes all variables that have a lower max height than the elevation minus 3 standard deviations
    std::vector<grid_map::Position> cells_to_remove;
    for (grid_map::GridMapIterator iterator(raw_map_); !iterator.isPastEnd(); ++iterator) {
        if (!raw_map_.isValid(*iterator)) {
            continue;
        }
        const auto& time = raw_map_.at("time", *iterator);
        if (time_since_start - time > scanning_duration_) {
            // Only remove cells that have not been updated during the last scan duration.
            // This prevents a.o. removal of overhanging objects.
            const auto& elevation = raw_map_.at("elevation", *iterator);
            const auto& variance = raw_map_.at("variance", *iterator);
            const auto& max_height = raw_map_.at("max_height", *iterator);

            double standard_deviation = sqrt(variance);
            if (!std::isnan(max_height) && elevation - 3.0 * standard_deviation > max_height) {
                grid_map::Position position;
                raw_map_.getPosition(*iterator, position);
                cells_to_remove.push_back(position);
            }
        }
    }

    // Sets all valid cells in cells_to_remove to NAN
    for (auto& cell : cells_to_remove) {
        grid_map::Index index;
        if (!raw_map_.getIndex(cell, index)) {
            continue; // If not in map skip
        }
        if (raw_map_.isValid(index)) {
            raw_map_.at("elevation", index) = NAN;
        }
    }

    // remove layers
    raw_map_.clear("lowest_scan_point");
    raw_map_.clear("sensor_x_at_lowest_scan");
    raw_map_.clear("sensor_y_at_lowest_scan");
    raw_map_.clear("sensor_z_at_lowest_scan");

    // remove layer used for cleanup.
    raw_map_.erase("max_height");
}


} // namespace traverse_layer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traverse_layer::PointcloudToGridmap)
