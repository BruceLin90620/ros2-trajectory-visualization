import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point

class PathVisualizer(Node):

    def __init__(self):
        super().__init__('path_visualizer')
        self.pub_trajectory_path = self.create_publisher(Path, 'trajectory_path', 10)
        timer_period = 0.1

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        # self.timer = self.create_timer(timer_period, self.trajectory_path_callback)

        self.frame_id = 'map'
        self.threshold = 0.001
        self.max_poses = 1000

        self.previous_pose_position = Point()
        self.trajectory_path_msg = Path()

    def odom_callback(self, odom_msg):
        self.trajectory_path_callback(odom_msg.pose.pose.position)
        # self.get_logger().info('I heard: "%s"' % odom_msg.pose.pose.position)
        
    
    def trajectory_path_callback(self, position):
        if ((abs(self.previous_pose_position.x - position.x) > self.threshold)
		 or (abs(self.previous_pose_position.y - position.y) > self.threshold)
		 or (abs(self.previous_pose_position.z - position.z) > self.threshold)):
               
            

            self.trajectory_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.trajectory_path_msg.header.frame_id = self.frame_id

            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            # pose_stamped_msg.pose.position.x = position.x + 0.5493068695068359
            pose_stamped_msg.pose.position.x = position.y - 2.1805243492126465
            # pose_stamped_msg.pose.position.y = position.y - 2.1805243492126465
            pose_stamped_msg.pose.position.y = -position.x - 0.5493068695068359
            pose_stamped_msg.pose.position.z = position.z
            pose_stamped_msg.pose.orientation.w = 1.0

            if len(self.trajectory_path_msg.poses) < self.max_poses:
                self.trajectory_path_msg.poses.append(pose_stamped_msg)
            else :
                self.trajectory_path_msg.poses = self.trajectory_path_msg.poses[1:]
                self.trajectory_path_msg.poses.append(pose_stamped_msg)

            self.trajectory_path_msg.poses.append(pose_stamped_msg)

            self.previous_pose_position = pose_stamped_msg.pose.position
            self.pub_trajectory_path.publish(self.trajectory_path_msg)
        # self.get_logger().info('I heard: "%s"' % self.trajectory_path_msg.poses)


def main(args=None):
    rclpy.init(args=args)

    path_visualizer = PathVisualizer()

    rclpy.spin(path_visualizer)

    path_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()