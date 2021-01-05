#include <functional>
#include <memory>
#include <chrono>
#include "rcutils/allocator.h"
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

// RGBD Imports
#include <sensor_msgs/msg/image.hpp>

// ORBSLAM3 imports
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include "../../../include/System.h" // whack import of ORB_SLAM3

using namespace std::chrono_literals;

class rgbdNode : public rclcpp::Node
{
public:
    rgbdNode()
        : Node("rgbd_node"), count_(0), ORBSLAM("~/automagic_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt", "~/automagic_ws/src/ORB_SLAM3/Examples/ROS/ORB_SLAM3/Asus.yaml",ORB_SLAM3::System::RGBD,true) // FIXME Properly add vocab/settings file
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        //timer_ = this->create_wall_timer(0.5s, std::bind(&rgbdNode::timer_callback, this));

        // RGB and Depth images are syncronised by Azure Kinect DK so no worries about timings.
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>("input_rgb", 10, std::bind(&rgbdNode::rgb_callback, this, std::placeholders::_1));
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>("input_depth", 10, std::bind(&rgbdNode::depth_callback, this, std::placeholders::_1));
    }

     ~rgbdNode() override
    {
        ORBSLAM.Shutdown();
    }

private:
    /** \brief Check whether both RGB and D channels are in sync
     * \returns Syncronisation state as Boolean
     */
    bool rgbd_is_sync()
    {
        if (rgb_->header.stamp.nanosec == depth_->header.stamp.nanosec)
        {
            RCLCPP_INFO(this->get_logger(), "RGBD is sync");
            return true;
        } else return false;
    }

    /** \brief Spin SLAM algorithm. Will only execute if RGB + D Images are in sync
     */
    void spin_slam()
    {
        // Poll RGBD sync state
        if (rgbd_is_sync())
        {
            // Spin SLAM
            RCLCPP_INFO(this->get_logger(), "Spin SLAM");
            try
            {
                cv_bridge::CvImagePtr cv_rgb_= cv_bridge::toCvCopy(rgb_, "CV_8U"); // gray-scale
                cv_bridge::CvImagePtr cv_depth_ = cv_bridge::toCvCopy(depth_, "CV_32F");
                cv::Mat cam_pose_ = ORBSLAM.TrackRGBD(cv_rgb_->image, cv_depth_->image,rgb_->header.stamp.sec);

                // Check for successful tracking
                if (cam_pose_.empty())
                {
                    RCLCPP_WARN(this->get_logger(), "ORB_SLAM tracking fail");
                    return;
                }

                std::cout << "Cam pose: " << std::endl << " "  << cam_pose_ << std::endl << std::endl;

                // Transform cam pose to Eigen Matrix type
                Eigen::Matrix<float,4,4> Ti;

                //cv::cv2eigen(cam_pose_, Ti);
            } 
            catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }


            pub_callback(); // XXX Include odom msg as fcn parameter?
        }
    }

    void pub_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Publish Odom");
        odom_->header.frame_id = "odom";
        odom_->child_frame_id = "base_link";
        odom_->header.stamp = rclcpp::Clock().now();

        odom_pub_->publish(std::move(*odom_));
    }

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rgb_ = msg;
        spin_slam();
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        depth_ = msg;
        spin_slam();
    }

    size_t count_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Image::SharedPtr rgb_;
    sensor_msgs::msg::Image::SharedPtr depth_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    ORB_SLAM3::System ORBSLAM;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rgbdNode>());
    rclcpp::shutdown();
    return 0;
}
