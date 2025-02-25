#include <LiDAR/Lidar_declare.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

// 이미지 크기 및 라이다 범위 설정
const int IMAGE_WIDTH = 600;
const int IMAGE_HEIGHT = 400;
const float LIDAR_RANGE = 10.0; // 라이다 범위 (단위: 미터)

// 직선을 이미지에 그리는 함수
void drawLine(cv::Mat& image, float slope, float y_intercept, cv::Scalar color) {
    float x1 = -LIDAR_RANGE;
    float y1 = slope * x1 + y_intercept;
    float x2 = LIDAR_RANGE;
    float y2 = slope * x2 + y_intercept;

    // 좌표를 이미지 좌표로 변환
    int img_x1 = static_cast<int>((x1 + LIDAR_RANGE) / (2 * LIDAR_RANGE) * IMAGE_WIDTH);
    int img_y1 = static_cast<int>((-y1 + LIDAR_RANGE) / (2 * LIDAR_RANGE) * IMAGE_HEIGHT);
    int img_x2 = static_cast<int>((x2 + LIDAR_RANGE) / (2 * LIDAR_RANGE) * IMAGE_WIDTH);
    int img_y2 = static_cast<int>((-y2 + LIDAR_RANGE) / (2 * LIDAR_RANGE) * IMAGE_HEIGHT);

    // 이미지에 직선 그리기
    cv::line(image, cv::Point(img_x1, img_y1), cv::Point(img_x2, img_y2), color, 2);
}

void coeffCallback(const LiDAR::coeff_arr::ConstPtr& msg, ros::Publisher& image_pub) {
    // 빈 이미지 생성 (흰색 배경)
    cv::Mat image = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));

    // 두 직선 그리기
    for (const auto& coeff : msg->coeff_data) {
        float slope = coeff.Slope;
        float y_intercept = coeff.Yintercept;
        drawLine(image, slope, y_intercept, cv::Scalar(0, 0, 255)); // 빨간색 직선
    }
    // 이미지를 90도 오른쪽으로 회전
    cv::Mat rotated_image;
    cv::rotate(image, rotated_image, cv::ROTATE_90_CLOCKWISE);

    // 이미지를 ROS 이미지 메시지로 변환
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rotated_image).toImageMsg();

    // 이미지 퍼블리시
    image_pub.publish(img_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_visualizer");
    ros::NodeHandle nh;

    // 이미지 퍼블리셔 설정
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("Line_visual", 1);

    // coeff_arr 메시지를 구독하고 콜백 함수 설정
    ros::Subscriber coeff_sub = nh.subscribe<LiDAR::coeff_arr>("RANSAC_coeff", 1, boost::bind(coeffCallback, _1, boost::ref(image_pub)));

    ros::spin();
    return 0;
}