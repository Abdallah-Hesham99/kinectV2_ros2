/**
 * @original_file   live.cpp
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 01:24:41 2015
 * @edited_by Abdallah Hesham
 * @date 9-June-2024 
 * @brief  
 * 
 * 
 */










#include "libk4w2/libk4w2.h"
#include "libk4w2/decoder.h"
#include "libk4w2/registration.h"

#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#define OUTPUT(fmt, ...) do { fprintf(stderr, fmt "\n", ## __VA_ARGS__); } while(0)
#define ABORT(fmt, ...) do { OUTPUT(fmt, ## __VA_ARGS__ ); exit(EXIT_FAILURE); } while(0)
#define CHK_K4W2( exp ) do { int res = exp; if (K4W2_SUCCESS != res) { OUTPUT(#exp " failed (error code is %d)", res); } } while(0)

enum FrameType {
    COLOR = 0,
    DEPTH = 1,
};

class KinectApp : public rclcpp::Node {
public:
    KinectApp(int device_id = 0)
        : Node("kinect_app"), device_id(device_id), ctx(nullptr), shutdown(false), last_ptr{nullptr}, last_len{0} {
        // Initialize Kinect
        ctx = k4w2_open(device_id, 0);
        if (!ctx) {
            ABORT("failed to open kinect device #%d", device_id);
        }

        unsigned int options = 0;
        decoders[DEPTH] = k4w2_decoder_open(K4W2_DECODER_DEPTH | options, 1);
        decoders[COLOR] = k4w2_decoder_open(K4W2_DECODER_COLOR | options, 1);

        if (decoders[DEPTH]) {
            struct kinect2_color_camera_param colorparam;
            struct kinect2_depth_camera_param depthparam;
            struct kinect2_p0table p0table;
            CHK_K4W2(k4w2_read_color_camera_param(ctx, &colorparam));
            CHK_K4W2(k4w2_read_depth_camera_param(ctx, &depthparam));
            CHK_K4W2(k4w2_read_p0table(ctx, &p0table));
            CHK_K4W2(k4w2_decoder_set_params(decoders[DEPTH], &colorparam, &depthparam, &p0table));
        }

        k4w2_set_color_callback(ctx, color_cb_wrapper, this);
        k4w2_set_depth_callback(ctx, depth_cb_wrapper, this);

        CHK_K4W2(k4w2_start(ctx));

        // Initialize OpenCV matrices
        rgb8U3 = cv::Mat(1080, 1920, CV_8UC3);
        resized8U3 = cv::Mat(1080 / 4, 1920 / 4, CV_8UC3);

        tmpbuf = new float[512 * 424 * 2];
        int sizes[2] = {424, 512};
        depth32F1 = cv::Mat(2, sizes, CV_32FC1, tmpbuf);
        ir32F1 = cv::Mat(2, sizes, CV_32FC1, tmpbuf + 512 * 424);

        is_rgb_colorspace = (K4W2_COLORSPACE_RGB == k4w2_decoder_get_colorspace(decoders[COLOR]));

        // Initialize ROS2 publishers
        rgb_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect2/rgb_image", 10);
        depth_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect2/depth_image", 10);
    }

    ~KinectApp() {
        CHK_K4W2(k4w2_stop(ctx));
        k4w2_close(&ctx);
        k4w2_decoder_close(&decoders[COLOR]);
        k4w2_decoder_close(&decoders[DEPTH]);
        delete[] tmpbuf;
    }

    void run() {
        rclcpp::Rate rate(30); // 30 Hz
        while (rclcpp::ok() && !shutdown) {
            process_frames();
            handle_keypress();
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

private:
    static void color_cb_wrapper(const void *buffer, int length, void *userdata) {
        static_cast<KinectApp*>(userdata)->color_cb(buffer, length);
    }

    static void depth_cb_wrapper(const void *buffer, int length, void *userdata) {
        static_cast<KinectApp*>(userdata)->depth_cb(buffer, length);
    }

    void color_cb(const void *buffer, int length) {
        const struct kinect2_color_footer *f = KINECT2_GET_COLOR_FOOTER(buffer, length);

        if (length < 10000) {
            OUTPUT("bad color frame?");
            return;
        }

        fprintf(stderr, "color: sequence:%10d timestamp:%10d\n", f->sequence, f->timestamp);
        last_ptr[COLOR] = buffer;
        last_len[COLOR] = length;
    }

    void depth_cb(const void *buffer, int length) {
        const struct kinect2_depth_footer *f = KINECT2_GET_DEPTH_FOOTER(buffer);

        if (length != KINECT2_DEPTH_FRAME_SIZE * 10) {
            OUTPUT("bad depth frame?");
            return;
        }

        fprintf(stderr, "depth: sequence:%10d timestamp:%10d\n", f->sequence, f->timestamp);
        last_ptr[DEPTH] = buffer;
        last_len[DEPTH] = length;
    }

    void process_frames() {
        const int slot = 0;

        if (last_ptr[COLOR]) {
            k4w2_decoder_request(decoders[COLOR], slot, last_ptr[COLOR], last_len[COLOR]);
            k4w2_decoder_fetch(decoders[COLOR], slot, rgb8U3.data, 1920 * 1080 * 3);

            cv::resize(rgb8U3, resized8U3, cv::Size(), 0.5, 0.5);

            if (is_rgb_colorspace)
                cv::cvtColor(resized8U3, resized8U3, CV_RGB2BGR);
            cv::imshow("rgb", resized8U3);

            // Publish RGB image
            auto rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized8U3).toImageMsg();
            rgb_publisher_->publish(*rgb_msg);

            last_ptr[COLOR] = nullptr;
        }

        if (last_ptr[DEPTH]) {
            k4w2_decoder_request(decoders[DEPTH], slot, last_ptr[DEPTH], last_len[DEPTH]);
            k4w2_decoder_fetch(decoders[DEPTH], slot, tmpbuf, 512 * 424 * 2 * sizeof(float));

            cv::imshow("depth", depth32F1 / 4500.f);

            // Publish depth image
            auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth32F1).toImageMsg();
            depth_publisher_->publish(*depth_msg);

            last_ptr[DEPTH] = nullptr;
        }
    }

    void handle_keypress() {
        const int rawkey = cv::waitKey(1);
        switch (rawkey & 0x0ffff) {
        case 'Q':
        case 'q':
            shutdown = true;
            break;
        }
    }

    int device_id;
    k4w2_t ctx;
    k4w2_decoder_t decoders[2];
    bool shutdown;
    const void *last_ptr[2];
    int last_len[2];
    cv::Mat rgb8U3, resized8U3;
    float *tmpbuf;
    cv::Mat depth32F1, ir32F1;
    bool is_rgb_colorspace;

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    int device_id = 0;
    if (argc >= 2) {
        device_id = std::atoi(argv[1]);
    }

    auto app = std::make_shared<KinectApp>(device_id);
    app->run();

    rclcpp::shutdown();
    return 0;
}

