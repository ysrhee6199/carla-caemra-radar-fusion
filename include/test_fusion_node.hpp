//C++
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <thread>
#include <mutex>
#include <string>

#include <opencv2/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>



//ROS2
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

namespace test_fusion {

class test_fusion_node : public rclcpp::Node
{
public:
    explicit test_fusion_node();
    ~test_fusion_node();

private:

    void callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg);

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    cv::Mat frontCamImageCopy_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat transform_;
};

} // namespace yolo_object_detectio