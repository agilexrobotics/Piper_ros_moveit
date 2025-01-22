import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateReorderNode(Node):
    def __init__(self):
        super().__init__('joint_state_reorder_node')
        
        # 订阅 /joint_states 主题
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1)
        
        # 发布排序后的消息到 /joint_states 主题
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            1)

        self.get_logger().info("JointStateReorderNode 已启动，等待 /joint_states 消息...")

    def joint_state_callback(self, msg):
        self.get_logger().info("收到新的 /joint_states 消息，正在排序...")

        # 获取关节名称并排序
        sorted_joint_names = sorted(msg.name)

        # 根据排序后的关节名称获取新的关节位置、速度和力矩
        sorted_positions = [msg.position[msg.name.index(joint)] for joint in sorted_joint_names]
        sorted_velocities = [msg.velocity[msg.name.index(joint)] for joint in sorted_joint_names]
        sorted_efforts = [msg.effort[msg.name.index(joint)] for joint in sorted_joint_names]

        # 创建新的 JointState 消息
        sorted_msg = JointState()
        sorted_msg.name = sorted_joint_names
        sorted_msg.position = sorted_positions
        sorted_msg.velocity = sorted_velocities
        sorted_msg.effort = sorted_efforts

        # 发布排序后的消息到原始话题 /joint_states
        self.joint_state_pub.publish(sorted_msg)
        
        self.get_logger().info("排序完成，已发布新的 /joint_states 消息。")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateReorderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
