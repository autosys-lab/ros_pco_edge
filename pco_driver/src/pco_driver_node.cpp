
#include <pco_driver/pco_driver.h>

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto camera_driver = std::make_shared<PCODriver>(options);

    exec.add_node(camera_driver);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}