#ifndef CAMERA_PUBLISHER_HPP_
#define CAMERA_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "libk4w2/libk4w2.h"
#include "libk4w2/decoder.h"
#include "libk4w2/registration.h"

#define OUTPUT(fmt, ...) do { fprintf(stderr, fmt "\n", ## __VA_ARGS__); } while(0)
#define ABORT(fmt, ...) do { OUTPUT(fmt, ## __VA_ARGS__ ); exit(EXIT_FAILURE); } while(0)
#define CHK_K4W2(exp) do { int res = exp; if (K4W2_SUCCESS != res) { OUTPUT(#exp " failed (error code is %d)", res); } } while(0)

enum {
    COLOR=0,
    DEPTH=1,
};

const void *last_ptr[2] = {0};
int last_len[2] = {0};

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher();
    ~CameraPublisher();

private:
    void timer_callback();
    static void color_cb(const void *buffer, int length, void *userdata);
    static void depth_cb(const void *buffer, int length, void *userdata);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    k4w2_t ctx_;
    k4w2_decoder_t decoder_[2] = {0};
    cv::Mat rgb8U3_ = cv::Mat(1080, 1920, CV_8UC3);
    float tmpbuf_[512 * 424 * 2];
    cv::Mat depth32F1_ = cv::Mat(2, std::vector<int>{424, 512}.data(), CV_32FC1, tmpbuf_);
    const bool is_rgb_colorspace_ = (K4W2_COLORSPACE_RGB == k4w2_decoder_get_colorspace(decoder_[COLOR]));
};

#endif // CAMERA_PUBLISHER_HPP_

