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


class LinearTranslate {
public:
    LinearTranslate(NavSatFix waypoint1, NavSatFix waypoint2) : waypoint1(waypoint1), waypoint2(waypoint2) {
        total_distance = calculate_distance();
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
    }

private:
    NavSatFix waypoint1;
    NavSatFix waypoint2;
    double total_distance;
    double speed;
    double alpha;
};

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("publish_demo_data")
    {
        mPublisher = this->create_publisher<NavSatFix>("fix", 1);
        mTimer     = this->create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&MyNode::timerCallback, this));

        mMarkerPublisher = this->create_publisher<visualization_msgs::msg::Marker>("marker_topic", 1);

        RCLCPP_INFO(this->get_logger(), "Gnss publisher has been started.");

        origin.latitude = 51.424;//stod(argv[1]);
        origin.longitude = 5.4943;//stod(argv[2]);
     
        waypoint1.latitude = 51.424069;//stod(argv[3]);
        waypoint1.longitude = 5.492310;//stod(argv[4]);
        
        waypoint2.latitude = 51.388781; //stod(argv[5]);
        waypoint2.longitude = 5.510854;//stod(argv[6]);

        
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
        cout << "Publishings NavSatFix at (" << fix->latitude << ", " << fix->longitude << ")" << endl;
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
        marker_msg.scale.x = 30.0;
        marker_msg.scale.y = 30.0;
        marker_msg.scale.z = 30.0;
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



class PointStampedSubscriber : public rclcpp::Node
{
public:
    PointStampedSubscriber() : Node("receive_point")
    {
        // Subscribe to the PointStamped topic
        point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&PointStampedSubscriber::pointCallback, this, std::placeholders::_1));

        cout << "clicked_point alindi." << endl;

        mEnlem = 0;
        mBoylam = 0;
    }

private:
    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Extract coordinates from the received PointStamped message
        double x = msg->point.x;
        double y = msg->point.y;
        double z = msg->point.z;

        Wgs84FromLocalXy(x, y, 51.424, 5.4943, mEnlem, mBoylam);
        cout << "Muhammed : (" << mEnlem << ", " << mBoylam << ")" << endl;

        // Use the coordinates as needed
        // For example, you can print them to the console
        RCLCPP_INFO(this->get_logger(), "Received point coordinates: (%f, %f, %f)", x, y, z);
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber_;

    double mEnlem;
    double mBoylam;
};



class ApplyNoise 
{
public:
    ApplyNoise(double stddev) : stddev(stddev) {}

    void update(NavSatFix& fix) {
        default_random_engine generator;
        normal_distribution<double> distribution(0.0, stddev);
        double noise_lat = distribution(generator);
        double noise_lon = distribution(generator);
        fix.latitude += noise_lat;
        fix.longitude += noise_lon;
    }

private:
    double stddev;
};

class LooseFix 
{
public:
    LooseFix(double phase, double period) : phase(phase), period(period) {}

    void update(Time t, NavSatFix& fix) {
        double t_sec = t.nanoseconds() / 1e9;
        double phase_0_to_1 = fmod(t_sec, period) / period;
        if (phase_0_to_1 < phase) {
            fix.status.status = NavSatStatus::STATUS_NO_FIX;
        } else {
            fix.status.status = NavSatStatus::STATUS_FIX;
        }
    }

private:
    double phase;
    double period;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<MyNode>();
    auto point_subscriber_node = std::make_shared<PointStampedSubscriber>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(my_node);
    executor.add_node(point_subscriber_node);

    // Spin the executor to run all nodes
    executor.spin();
    rclcpp::shutdown();

    return 0;
}