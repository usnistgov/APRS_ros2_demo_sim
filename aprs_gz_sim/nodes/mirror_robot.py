import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MirrorRobot(Node):
    def __init__(self):
        super().__init__('digital_twin')
        self.declare_parameter('robot_name', 'fanuc')
        self.robot_name = self.get_parameter('robot_name').value

        self.real_robot_joint_states_sub = self.create_subscription(
            JointState,
            f'/{self.robot_name}/joint_states',
            self.real_robot_joint_states_callback,
            10)

        self.simulated_robot_joint_traj_pub = self.create_publisher(
            JointState,
            f'/simulation/{self.robot_name}/joint_trajectory_controller/joint_trajectory',
            10)

    def real_robot_joint_states_callback(self, msg: JointState):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = msg.name
        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 1e8

        traj_msg.points.append(point)
        self.simulated_robot_joint_traj_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)

    digital_twin = MirrorRobot()

    try:
        rclpy.spin(digital_twin)
    except KeyboardInterrupt:
        digital_twin.get_logger().info('KeyboardInterrupt caught, shutting down')
    finally:
        digital_twin.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()