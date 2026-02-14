#include "astra/streams/Color.hpp"
#include "astra_ros2_brdige/astra_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <astra/astra.hpp>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    std::cout << "Initializing Astra SDK..." << std::endl;
    
    try {
        astra::initialize();
        std::cout << "Astra initialized, waiting..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));  
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to initialize Astra: " << e.what() << std::endl;
        return 1;
    }
    

    rclcpp::init(argc, argv);
    std::cout << "ROS2 initialized" << std::endl;

    std::shared_ptr<AstraROS2Bridge> node;
 
    try
    {
        std::cout << "Creating ROS2 Node..." << std::endl;
        node = std::make_shared<AstraROS2Bridge>();
        std::cout << "Node created successfully" << std::endl;
        
        std::cout << "Starting streams..." << std::endl;
        node->start_streams();
        std::cout << "Streams started successfully" << std::endl;
        node->set_parameter(rclcpp::Parameter("use_sim_time", true));
 
        std::cout << "Spinning..." << std::endl;
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    // Cleanup
    std::cout << "Shutting down..." << std::endl;
    node.reset();
    astra::terminate();
    rclcpp::shutdown();
    
    return 0;
}
