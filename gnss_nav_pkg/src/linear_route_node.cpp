#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>
#include <vector>
#include <random>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <swri_transform_util/local_xy_util.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std;
using namespace rclcpp;
using namespace sensor_msgs::msg;
using namespace swri_transform_util;


class ApplyNoise 
{
public:
    ApplyNoise(double stddev) : stddev(stddev), generator(std::random_device{}()), distribution(0.0, stddev) {}

    void update(NavSatFix& fix) {
        double noise_lat = distribution(generator)/5000.0;
        double noise_lon = distribution(generator)/500.0;
        fix.latitude += noise_lat;
        fix.longitude += noise_lon;

        cout << "Noise at (" << noise_lat << ", " << noise_lon << ")" << endl;
    }

private:
    double stddev;
    default_random_engine generator;
    normal_distribution<double> distribution;
};


class LinearTranslate {
public:
    LinearTranslate(NavSatFix waypoint1, NavSatFix waypoint2) : waypoint1(waypoint1), waypoint2(waypoint2) {
        total_distance = calculate_distance();
        noiseApplier = std::make_shared<ApplyNoise>(0.001); 
        speed = 2.0;
        alpha = 0.0;
    }

    double calculate_distance() {
        double lat1 = waypoint1.latitude;
        double lon1 = waypoint1.longitude;
        double lat2 = waypoint2.latitude;
        double lon2 = waypoint2.longitude;
        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;
        return sqrt(dlat * dlat + dlon * dlon);
    }

    void update(NavSatFix& fix) {
        alpha += 0.001 * speed;
        if (alpha > 1.0) {
            alpha = 0.0;
        }

        fix.latitude = waypoint1.latitude + alpha * (waypoint2.latitude - waypoint1.latitude);
        fix.longitude = waypoint1.longitude + alpha * (waypoint2.longitude - waypoint1.longitude);
        noiseApplier->update(fix);
    }

private:
    NavSatFix waypoint1;
    NavSatFix waypoint2;
    double total_distance;
    double speed;
    double alpha;
    shared_ptr<ApplyNoise> noiseApplier; 
};


class PositionPublisher : public rclcpp::Node
{
public:
    PositionPublisher() : Node("publish_demo_data")
    {
        mPublisher = this->create_publisher<NavSatFix>("fix", 1);
        mTimer     = this->create_wall_timer(std::chrono::milliseconds(200), // Decrease the frequency to 200 ms
                                             std::bind(&PositionPublisher::timerCallback, this));

        mMarkerPublisher = this->create_publisher<visualization_msgs::msg::Marker>("marker_topic", 1);

        RCLCPP_INFO(this->get_logger(), "Gnss publisher has been started.");

        origin.latitude = 51.424;
        origin.longitude = 5.4943;
     
        waypoint1.latitude = 51.424069;
        waypoint1.longitude = 5.492310;
        
        waypoint2.latitude = 51.388781; 
        waypoint2.longitude = 5.510854;

        mUpdaters.push_back(std::make_shared<LinearTranslate>(waypoint1, waypoint2));
    }

private:
    void timerCallback()
    {
        auto fix = std::make_shared<NavSatFix>();
        Time t = this->get_clock()->now();
        fix->header.stamp = t;
        fix->header.frame_id = "gps_sensor";
        fix->latitude = origin.latitude;
        fix->longitude = origin.longitude;
        for (auto u : mUpdaters) {
            u->update(*fix);
        }

        cout << "Publishing NavSatFix at (" << fix->latitude << ", " << fix->longitude << ")" << endl;
        mPublisher->publish(*fix);

        visualization_msgs::msg::Marker marker_msg;

        marker_msg.header.frame_id = "gps_sensor";
        marker_msg.header.stamp = t;
        marker_msg.ns = "markers";
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.position.x = 0.0;
        marker_msg.pose.position.y = 0.0;
        marker_msg.pose.position.z = 1.0;
        marker_msg.pose.orientation.x = 0.0;
        marker_msg.pose.orientation.y = 0.0;
        marker_msg.pose.orientation.z = 0.0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 40.0;
        marker_msg.scale.y = 40.0;
        marker_msg.scale.z = 40.0;
        marker_msg.color.a = 1.0; // Don't forget to set the alpha!
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 0.5;
        marker_msg.color.b = 0.8;

        // Publish marker
        mMarkerPublisher->publish(marker_msg);
    }

    NavSatFix origin;
    NavSatFix waypoint1;
    NavSatFix waypoint2;

    vector<shared_ptr<LinearTranslate>> mUpdaters;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mMarkerPublisher;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr mPublisher;
    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Node::SharedPtr node_ptr;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto positionPublisherNode = std::make_shared<PositionPublisher>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(positionPublisherNode);

    // Spin the executor to run all nodes
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
