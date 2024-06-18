#include <aprs_gz_sim/spawn_part.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);

  auto spawn_part_node = std::make_shared<SpawnPart>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(spawn_part_node);

  executor.spin();

  rclcpp::shutdown();
}