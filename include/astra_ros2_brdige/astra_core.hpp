#ifndef ASTRA_ROS2_BRIDGE_ASTRA_CORE_HPP
#define ASTRA_ROS2_BRIDGE_ASTRA_CORE_HPP

#include "astra/streams/Color.hpp"
#include "astra/streams/Depth.hpp"
#include "astra_core/Frame.hpp"
#include "astra_core/StreamReader.hpp"
#include "astra_core/astra_core.hpp"
#include <astra/astra.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_mat_sensor_msgs_image_type_adapter.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <grasp_msgs/srv/grasp_gen_infer.hpp>

class ColorFrameListener : public astra::FrameListener
{
public:
    std::shared_ptr<sensor_msgs::msg::Image> last_msg_;
    std::mutex msg_mutex_;

    void on_frame_ready(astra::StreamReader& /*reader*/, astra::Frame& frame) override
    {
        const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
        if (!colorFrame.is_valid()) return;

        int width = colorFrame.width();
        int height = colorFrame.height();
        cv::Mat image(height, width, CV_8UC3);
        colorFrame.copy_to(reinterpret_cast<astra::RgbPixel*>(image.data));

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image).toImageMsg();
        msg->header.stamp = rclcpp::Clock().now();
        
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

    void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
    {
        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
        if (!depthFrame.is_valid()) return;

        int width = depthFrame.width();
        int height = depthFrame.height();
        cv::Mat depth_image(height, width, CV_16UC1);
        depthFrame.copy_to(reinterpret_cast<int16_t*>(depth_image.data));

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_image).toImageMsg();
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "camera_depth_frame";

        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud->header.stamp = msg->header.stamp;
        cloud->header.frame_id = "map";
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
                int16_t d = depth_image.at<int16_t>(y, x);
                float wx, wy, wz;
                mapper.convert_depth_to_world(x, y, d, wx, wy, wz);
                it_x[0] = wx / 1000.0f;  // Convert mm to meters
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
    AstraROS2Bridge() 
        : Node("astra_ros2_bridge"), should_continue_(true)
    {
        RCLCPP_INFO(get_logger(), "Starting Astra ROS2 Bridge...");

        // Initialize publishers
        rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/rgb/image_raw", 10);
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/depth/image_raw", 10);
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("camera/point_cloud", 10);

        RCLCPP_INFO(get_logger(), "Publishers created.");

        try
        {
            // Initialize Astra
            streamSet_ = std::make_unique<astra::StreamSet>();
            reader_ = std::make_unique<astra::StreamReader>(streamSet_->create_reader());

            RCLCPP_INFO(get_logger(), "Astra StreamSet and StreamReader initialized.");

            // Add listeners
            reader_->add_listener(depth_listener_);
            reader_->add_listener(color_listener_);

            RCLCPP_INFO(get_logger(), "Listeners added to StreamReader.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception during Astra initialization: %s", e.what());
            throw;
        }

        // Timer for publishing frames (30 Hz)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&AstraROS2Bridge::publish_frames, this)
        );
        RCLCPP_INFO(get_logger(), "Timer created for frame publishing.");
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
        _frame_astra = std::make_unique<astra::Frame>(reader_->get_latest_frame(1000));

        auto depthStream = reader_->stream<astra::DepthStream>();
        depthStream.start();
        auto ColorStream= reader_->stream<astra::ColorStream>();
        ColorStream.start();
         
        // Print camera info
        char serialnumber[256];
        depthStream.serial_number(serialnumber, 256);

        RCLCPP_INFO(get_logger(), "Depth Stream -- hFov: %.2f, vFov: %.2f, Serial: %s",
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
        RCLCPP_INFO(get_logger(), "USB Info -- PID: %d, VID: %d", usbinfo.pid, usbinfo.vid);

        // Start the astra_update thread AFTER streams are started
        astra_thread_ = std::thread([this]() {
            RCLCPP_INFO(get_logger(), "Astra update thread started");
            while (should_continue_ && rclcpp::ok())
            {
                astra_update();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            RCLCPP_INFO(get_logger(), "Astra update thread stopped");
        });
        
        RCLCPP_INFO(get_logger(), "Astra streams started successfully");
    }

private:
    void publish_frames()
    {
        try
        {
            // Publish depth image
            {
                std::lock_guard<std::mutex> lock(depth_listener_.msg_mutex_);
                if (depth_listener_.last_msg_)
                {
                    depth_listener_.on_frame_ready(*reader_.get(), *_frame_astra);
                    depth_pub_->publish(*depth_listener_.last_msg_);
                    RCLCPP_DEBUG(get_logger(), "Published Depth frame.");
                }
            }
            
            // Publish point cloud
            {
                std::lock_guard<std::mutex> lock(depth_listener_.msg_mutex_);
                if (depth_listener_.last_cloud_)
                {
                    cloud_pub_->publish(*depth_listener_.last_cloud_);
                    RCLCPP_DEBUG(get_logger(), "Published PointCloud2 frame.");
                }
            }
            
            // Publish color image
            {
                std::lock_guard<std::mutex> lock(color_listener_.msg_mutex_);
                if (color_listener_.last_msg_)
                {
                    color_listener_.on_frame_ready(*reader_.get(), *_frame_astra );
                    rgb_pub_->publish(*color_listener_.last_msg_);
                    RCLCPP_DEBUG(get_logger(), "Published RGB frame.");
                }
            }
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
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<astra::Frame> _frame_astra;
    std::thread astra_thread_;
    std::atomic<bool> should_continue_;
};

#endif