#include "lidarStrat.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarStrat>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // Print a message to let the user know the script exited cleanly
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Obstacles Node exited cleanly");
    rclcpp::shutdown();
    return 0;
}
