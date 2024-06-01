#include "test_fusion_node.hpp"
#include <cmath>

namespace test_fusion
{

test_fusion_node::test_fusion_node()
    : Node("test_fusion_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)),
           image_sub_(this, "/truck1/front_camera", rclcpp::SensorDataQoS().get_rmw_qos_profile()),
      pointcloud_sub_(this, "/truck1/front_radar", rclcpp::SensorDataQoS().get_rmw_qos_profile())
{
    // QoS 설정
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.best_effort();

            // Subscriber 초기화

    // Synchronizer 초기화 (ExactTime 정책 사용)
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), image_sub_, pointcloud_sub_);
    sync_->registerCallback(std::bind(&test_fusion_node::callback, this, std::placeholders::_1, std::placeholders::_2));
    // 카메라 내부 파라미터 설정
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 320, 0, 320, 0, 320, 240, 0, 0, 1); // focal_length = 320, cx = 320, cy = 240
    dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F); // 렌즈 왜곡 없음

    // 카메라와 레이더 간 변환 매트릭스 설정
    double theta = -15.0 * M_PI / 180.0; // -15도 각도 라디안으로 변환
    cv::Mat rotation = (cv::Mat_<double>(3, 3) <<
                         cos(theta), 0, sin(theta),
                         0, 1, 0,
                         -sin(theta), 0, cos(theta));
    cv::Mat translation = (cv::Mat_<double>(3, 1) << 0, 0, -2.0); // z축 방향으로 2m 이동 (카메라가 레이더보다 위에 있음)

    // 변환 매트릭스를 4x4 동차 좌표계로 확장
    transform_ = cv::Mat::eye(4, 4, CV_64F);
    rotation.copyTo(transform_(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(transform_(cv::Rect(3, 0, 1, 3)));

}


test_fusion_node::~test_fusion_node()
{

}


void test_fusion_node::callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg)
{
    // 이미지 데이터를 OpenCV 형식으로 변환
    cv_bridge::CvImagePtr cam_image;
    try
    {
        cam_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception : %s", e.what());
        return;
    }

    cv::Mat image = cam_image->image;

    // 포인트 클라우드 데이터를 순회하며 변환 및 오버레이
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points_msg, "x"),
                                                      iter_y(*points_msg, "y"),
                                                      iter_z(*points_msg, "z");
         iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z)
    {

        if(*iter_x > 10) {
            continue;
        }
        // 레이더 좌표계를 카메라 좌표계로 변환 (레이더의 x -> 카메라의 z, 레이더의 y -> 카메라의 x, 레이더의 z -> 카메라의 y)
        cv::Mat radar_point = (cv::Mat_<double>(4, 1) << *iter_x, *iter_y, *iter_z, 1.0);
        cv::Mat cam_point = transform_ * radar_point;

        double cam_z = cam_point.at<double>(0, 0); // 레이더의 x -> 카메라의 z
        double cam_x = cam_point.at<double>(1, 0); // 레이더의 y -> 카메라의 x
        double cam_y = cam_point.at<double>(2, 0); // 레이더의 z -> 카메라의 y

        // 카메라 좌표를 이미지 좌표로 변환
        cv::Mat uvw = camera_matrix_ * (cv::Mat_<double>(3, 1) << -cam_x, -cam_y, cam_z); // cam_x 값을 음수로 변환하여 이미지 x 좌표의 방향을 맞춤
        int img_x = static_cast<int>(uvw.at<double>(0, 0) / uvw.at<double>(2, 0));
        int img_y = static_cast<int>(uvw.at<double>(1, 0) / uvw.at<double>(2, 0));

        // 이미지 범위 내에 있는지 확인
        if (img_x >= 0 && img_x < image.cols && img_y >= 0 && img_y < image.rows)
        {
            // 이미지에 포인트 오버레이
            cv::circle(image, cv::Point(img_x, img_y), 3, cv::Scalar(0, 0, 255), -1);
            RCLCPP_INFO(this->get_logger(), "img_x, img_y: %d, %d, %f", img_x, img_y, *iter_x);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "else");
        }

    }

    // 오버레이된 이미지를 표시
    cv::namedWindow("Fused Image");
    cv::imshow("Fused Image", image);
    cv::waitKey(10);
}


}