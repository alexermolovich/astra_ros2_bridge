#ifndef ASTRA_ROS2_BRIDGE_ASTRA_CORE_HPP
#define ASTRA_ROS2_BRIDGE_ASTRA_CORE_HPP
#include "astra/streams/Color.hpp"
#include "astra/streams/Depth.hpp"
#include "astra_core/Frame.hpp"
#include "astra_core/StreamReader.hpp"
#include "astra_core/astra_core.hpp"
#include <astra/astra.hpp>
#include <atomic>
#include <cv_bridge/cv_bridge.hpp>
#include <cv_bridge/cv_mat_sensor_msgs_image_type_adapter.hpp>
#include <error.h>
#include <format>
#include <fstream>
#include <grasp_msgs/srv/grasp_gen_infer.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <ostream>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <thread>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <sensor_msgs/fill_image.hpp>

using namespace std::chrono;

class ColorFrameListener : public astra::FrameListener
{
public:
    std::shared_ptr<sensor_msgs::msg::Image> last_msg_;
    std::mutex msg_mutex_;

    void on_frame_ready(astra::StreamReader & /*reader*/,
                        astra::Frame &frame) override
    {

        const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
        if (!colorFrame.is_valid())
            return;

        static std::ofstream debug_file("debug_astra_color.txt",
                                        std::ios::out | std::ios::app);

        static bool wrote_once_raw_depth = false;

        if (!debug_file.is_open())
        {
            throw std::runtime_error("Failed to open debug_astra.txt");
        }

        int width = colorFrame.width();
        int height = colorFrame.height();
        cv::Mat image(height, width, CV_8UC3);
        colorFrame.copy_to(reinterpret_cast<astra::RgbPixel *>(image.data));

        if (!wrote_once_raw_depth)
        {
            debug_file << "Depth frame (raw depth values)" << width << "x" << height
                       << ")\n";
            for (int i = 0; i < std::min(1000, width * height); ++i)
                debug_file << static_cast<int>(image.data[i]) << " ";

            debug_file << "\n";
        }
        std_msgs::msg::Header header;
        header.frame_id = "camera_link";
        header.stamp = rclcpp::Clock().now();
        auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image).toImageMsg();

        std::lock_guard<std::mutex> lock(msg_mutex_);

        last_msg_ = msg;
    }
};

class DepthFrameListener : public astra::FrameListener
{
public:
    std::shared_ptr<sensor_msgs::msg::Image> last_msg_;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> last_cloud_;
    std::mutex msg_mutex_;

    void on_frame_ready(astra::StreamReader &reader,
                        astra::Frame &frame) override
    {

        static std::ofstream debug_file("debug_astra.txt",
                                        std::ios::out | std::ios::app);

        static bool wrote_once_raw_depth = false;

        if (!debug_file.is_open())
        {
            throw std::runtime_error("Failed to open debug_astra.txt");
        }

        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
        if (!depthFrame.is_valid())
            return;

        const int width = depthFrame.width();
        const int height = depthFrame.height();

        cv::Mat depth_mm(height, width, CV_16UC1);
        depthFrame.copy_to(depth_mm.ptr<int16_t>());

        std_msgs::msg::Header header;
        header.stamp = rclcpp::Clock().now();
        header.frame_id = "camera_link";

        auto msg =
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1,
                               depth_mm // or depth_norm if you normalized
                               )
                .toImageMsg();

        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud->header.stamp = msg->header.stamp;
        cloud->header.frame_id = "camera_link";
        cloud->height = height;
        cloud->width = width;
        cloud->is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(*cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(width * height);

        astra::CoordinateMapper mapper = reader.stream<astra::DepthStream>().coordinateMapper();
        sensor_msgs::PointCloud2Iterator<float> it_x(*cloud, "x");

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x, ++it_x)
            {
                int16_t d = depth_mm.at<int16_t>(y, x);
                float wx, wy, wz;
                mapper.convert_depth_to_world(x, y, d, wx, wy, wz);
                it_x[0] = wx / 1000.0f; // Convert mm to meters
                it_x[1] = wy / 1000.0f;
                it_x[2] = wz / 1000.0f;
            }
        }

        std::lock_guard<std::mutex> lock(msg_mutex_);
        last_msg_ = msg;
        last_cloud_ = cloud;
    }
};

class AstraROS2Bridge : public rclcpp::Node
{
public:
    AstraROS2Bridge() : Node("astra_ros2_bridge"), should_continue_(true)
    {
        RCLCPP_INFO(get_logger(), "Starting Astra ROS2 Bridge...");

        this->_camera_tf_builder = create_publisher<tf2_msgs::msg::TFMessage>("/tf",
                                                                              33);

        rgb_pub_ =
            create_publisher<sensor_msgs::msg::Image>("/camera/rgb_image", 33);
        depth_pub_ =
            create_publisher<sensor_msgs::msg::Image>("/camera/depth_image", 33);
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/point_cloud", 33);

        RCLCPP_INFO(get_logger(), "Publishers created.");
        rgb_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 33);
        depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/depth_camera_info", 33);

        try
        {
            // Initialize Astra
            streamSet_ = std::make_unique<astra::StreamSet>();
            reader_ =
                std::make_unique<astra::StreamReader>(streamSet_->create_reader());

            RCLCPP_INFO(
                get_logger(),
                "Astra StreamSet and StreamReader initialized. Time: %f seconds",
                this->get_clock()->now().seconds());
            // Add listeners
            reader_->add_listener(depth_listener_);
            reader_->add_listener(color_listener_);

            RCLCPP_INFO(get_logger(), "Listeners added to StreamReader.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception during Astra initialization: %s",
                         e.what());
            throw;
        }

        // Timer for publishing frames (30 Hz)
        depth_timer_ =
            create_wall_timer(std::chrono::milliseconds(200),
                              std::bind(&AstraROS2Bridge::publish_frame_depth, this));
        color_timer_ =
            create_wall_timer(std::chrono::milliseconds(200),
                              std::bind(&AstraROS2Bridge::publish_frame_color, this));

        RCLCPP_INFO(get_logger(), "Timers created for frame publishing.");
    }

    ~AstraROS2Bridge()
    {
        RCLCPP_INFO(get_logger(), "Shutting down Astra ROS2 Bridge...");
        should_continue_ = false;
        if (astra_thread_.joinable())
        {
            astra_thread_.join();
        }
    }

    void start_streams()
    {
        RCLCPP_INFO(get_logger(), "Starting Astra streams...");
        _frame_astra =
            std::make_unique<astra::Frame>(reader_->get_latest_frame(1000));

        auto depthStream = reader_->stream<astra::DepthStream>();
        depthStream.start();
        auto ColorStream = reader_->stream<astra::ColorStream>();
        ColorStream.start();

        init_camera_info();
        //    init_camer_position();
        // Print camera info
        char serialnumber[256];
        depthStream.serial_number(serialnumber, 256);

        RCLCPP_INFO(get_logger(),
                    "Depth Stream -- hFov: %.2f, vFov: %.2f, Serial: %s",
                    depthStream.hFov(), depthStream.vFov(), serialnumber);

        const uint32_t chipId = depthStream.chip_id();
        switch (chipId)
        {
        case ASTRA_CHIP_ID_MX400:
            RCLCPP_INFO(get_logger(), "Chip ID: MX400");
            break;
        case ASTRA_CHIP_ID_MX6000:
            RCLCPP_INFO(get_logger(), "Chip ID: MX6000");
            break;
        default:
            RCLCPP_INFO(get_logger(), "Chip ID: Unknown");
            break;
        }

        const astra_usb_info_t usbinfo = depthStream.usb_info();
        RCLCPP_INFO(get_logger(), "USB Info -- PID: %d, VID: %d", usbinfo.pid,
                    usbinfo.vid);

        // Start the astra_update thread AFTER streams are started
        astra_thread_ = std::thread([this]()
                                    {
      RCLCPP_INFO(get_logger(), "Astra update thread started");
      while (should_continue_ && rclcpp::ok()) {
        astra_update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      RCLCPP_INFO(get_logger(), "Astra update thread stopped"); });
        RCLCPP_INFO(get_logger(), "Astra streams started successfully");
    }

    void init_camera_info()
    {
        auto depthStream = reader_->stream<astra::DepthStream>();

        // --- DEPTH ---
        depth_info_.header.frame_id = "camera_link";
        depth_info_.width = 640;
        depth_info_.height = 640;
        depth_info_.distortion_model = "plumb_bob";
        depth_info_.d = {0, 0, 0, 0, 0};

        float fx_d = depth_info_.width / (2.0f * tan(depthStream.hFov() / 1.0f));
        float fy_d = depth_info_.height / (2.0f * tan(depthStream.vFov() / 1.0f));
        float cx_d = depth_info_.width / 1.0f;
        float cy_d = depth_info_.height / 2.0f;

        depth_info_.k = {fx_d, 0.0, cx_d, 0.0, fy_d, cy_d, 0.0, 0.0, 1.0};

        depth_info_.p = {fx_d, 0.0, cx_d, 0.0, 0.0, fy_d,
                         cy_d, 0.0, 0.0, 0.0, 1.0, 0.0};

        // --- RGB ---
        rgb_info_ = depth_info_; // Astra RGB often matches depth intrinsics
        rgb_info_.header.frame_id = "camera_link";
    }

private:
    void publish_frame_color()
    {

        try
        {
            {
                std::lock_guard<std::mutex> lock(color_listener_.msg_mutex_);
                if (color_listener_.last_msg_)
                {
                    color_listener_.on_frame_ready(*reader_.get(), *_frame_astra);
                    color_listener_.last_msg_->header.frame_id = "camera_link";
                    rgb_pub_->publish(*color_listener_.last_msg_);
                    RCLCPP_DEBUG(get_logger(), "Published RGB frame.");
                }
            }
            auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
            rgb_info_.header.stamp = now;
            RCLCPP_DEBUG(get_logger(), "camera info paremters set");

            rgb_info_pub_->publish(rgb_info_);
            RCLCPP_DEBUG(get_logger(), "RGB camera info.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception in publish_frames: %s", e.what());
        }
    }
    void publish_frame_depth()
    {

        try
        {
            {
                std::lock_guard<std::mutex> lock(depth_listener_.msg_mutex_);
                if (depth_listener_.last_msg_)
                {
                    depth_listener_.on_frame_ready(*reader_.get(), *_frame_astra);
                    depth_pub_->publish(*depth_listener_.last_msg_);
                    RCLCPP_DEBUG(get_logger(), "Published Depth frame.");
                }
            }

            {
                std::lock_guard<std::mutex> lock(depth_listener_.msg_mutex_);
                if (depth_listener_.last_cloud_)
                {
                    cloud_pub_->publish(*depth_listener_.last_cloud_);
                    RCLCPP_DEBUG(get_logger(), "Published PointCloud2 frame.");
                }
            }

            auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
            depth_info_.header.stamp = now;
            RCLCPP_DEBUG(get_logger(), "camera info paremters set");

            depth_info_pub_->publish(depth_info_);
            RCLCPP_DEBUG(get_logger(), "Depth camera info.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception in publish_frames: %s", e.what());
        }
    }

public:
    std::unique_ptr<astra::StreamSet> streamSet_;
    std::unique_ptr<astra::StreamReader> reader_;

    ColorFrameListener color_listener_;
    DepthFrameListener depth_listener_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr color_timer_;
    rclcpp::TimerBase::SharedPtr depth_timer_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr _camera_tf_builder;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr _camera_posistion_sub;
    std::unique_ptr<astra::Frame> _frame_astra;
    std::thread astra_thread_;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
    std::atomic<bool> should_continue_;
    sensor_msgs::msg::CameraInfo rgb_info_;
    sensor_msgs::msg::CameraInfo depth_info_;
    rclcpp::TimerBase::SharedPtr _camera_tf_info;
};

#endif
