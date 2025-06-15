#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>
#include <limits>
#include <chrono>

using namespace std::chrono_literals;

class ObstacleAvoider : public rclcpp::Node
{
public:
    ObstacleAvoider()
        : Node("obstacle_avoider")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ObstacleAvoider::scan_callback, this, std::placeholders::_1));
        
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ObstacleAvoider::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(500ms, std::bind(&ObstacleAvoider::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Enhanced ObstacleAvoider node started.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int mid_idx = msg->ranges.size() / 2;
        int range_window = 10;
        obstacle_detected_ = false;

        for (int i = -range_window; i <= range_window; ++i)
        {
            int idx = mid_idx + i;
            if (idx >= 0 && idx < static_cast<int>(msg->ranges.size()))
            {
                float range = msg->ranges[idx];
                if (range < 0.2 && range > msg->range_min)
                {
                    obstacle_detected_ = true;
                    break;
                }
            }
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_x_ = msg->pose.pose.position.x;
        current_odom_y_ = msg->pose.pose.position.y;
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        double dx = current_odom_x_ - last_odom_x_;
        double dy = current_odom_y_ - last_odom_y_;
        double moved_distance = std::sqrt(dx * dx + dy * dy);

        # 
        bool is_stuck = moved_distance < 0.01;

        if (obstacle_detected_ || is_stuck)
        {
            if (is_stuck)
            {
                stuck_counter_++;
            }
            else
            {
                stuck_counter_ = 0;
            }

            if (stuck_counter_ > 10)
            {
                cmd.linear.x -= random() % 2 ? 0.2 : -0.2;
                cmd.angular.z += 1;
                RCLCPP_WARN(this->get_logger(), "Stuck! Trying to back off and turn right...");
            }
            else
            {
                cmd.linear.x = 0.0;
                cmd.angular.z -= 1;
                RCLCPP_INFO(this->get_logger(), "Obstacle detected, turning left...");
            }
        }
        else
        {
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Path clear, moving forward.");
            stuck_counter_ = 0;
        }

        last_odom_x_ = current_odom_x_;
        last_odom_y_ = current_odom_y_;
        cmd_pub_->publish(cmd);
    }

    // 成员变量
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool obstacle_detected_ = false;

    // 里程计
    double last_odom_x_, last_odom_y_;
    double current_odom_x_, current_odom_y_;
    int stuck_counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoider>());
    rclcpp::shutdown();
    return 0;
}


/*
LaserScan.msg

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
Twist.msg
Vector3  linear （x, y, z） 参考右手坐标系， x控制前后， y左右， z上下
Vector3  angular（x, y, z)  在二维平面上，z控制绕z轴旋转， 左转（+）/ 右转（-）单位：弧度/s
*/
