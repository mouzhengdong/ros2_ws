#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class Tf2ExampleNode : public rclcpp::Node
{
public:
    Tf2ExampleNode() : Node("tf2_example_node")
    {
        // 创建 TF2 缓存和监听器
        buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        // 创建 TF2 广播器
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 定时发布变换
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Tf2ExampleNode::on_timer, this)
        );
    }

private:
    void on_timer()
    {
        // 发布机器人与世界坐标系之间的变换
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = 1.0;  // 假设机器人在世界坐标系中的位置
        transformStamped.transform.translation.y = 2.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        broadcaster_->sendTransform(transformStamped);

        // 发布摄像头与机器人坐标系之间的静态变换
        geometry_msgs::msg::TransformStamped cameraTransform;
        cameraTransform.header.stamp = now();
        cameraTransform.header.frame_id = "base_link";
        cameraTransform.child_frame_id = "camera_link";
        cameraTransform.transform.translation.x = 0.5;  // 假设摄像头相对于机器人坐标系的位置
        cameraTransform.transform.translation.y = 0.0;
        cameraTransform.transform.translation.z = 1.0;
        cameraTransform.transform.rotation.x = 0.0;
        cameraTransform.transform.rotation.y = 0.0;
        cameraTransform.transform.rotation.z = 0.0;
        cameraTransform.transform.rotation.w = 1.0;

        broadcaster_->sendTransform(cameraTransform);

        // 查询 'camera_link' 到 'base_link' 的变换
        // try
        // {
        //     geometry_msgs::msg::TransformStamped transformStamped =
        //         buffer_->lookupTransform("base_link", "camera_link", rclcpp::Time(0));
            
        //     RCLCPP_INFO(this->get_logger(), "Transform from camera_link to base_link: [%f, %f, %f]",
        //                 transformStamped.transform.translation.x,
        //                 transformStamped.transform.translation.y,
        //                 transformStamped.transform.translation.z);
        // }
        // catch (const tf2::TransformException & ex)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        // }

        // 查询 'camera_link' 到 'world' 的变换
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                buffer_->lookupTransform("world", "camera_link", rclcpp::Time(0));
            
            RCLCPP_INFO(this->get_logger(), "Transform from camera_link to world: [%f, %f, %f]",
                        transformStamped.transform.translation.x,
                        transformStamped.transform.translation.y,
                        transformStamped.transform.translation.z);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tf2ExampleNode>());
    rclcpp::shutdown();
    return 0;
}