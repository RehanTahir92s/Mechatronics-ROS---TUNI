"""
Author: Rehan Tahir Sabbih, Tampere University

Description:
This script scbscribes to particular topics and creates a rosbag 
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rosbag2_py


class RosbagRecorder(Node):
    def __init__(self):
        super().__init__('turtlebot3_rosbag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        # parameter declaration
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.linear_odom = 0.0
        self.angular_odom = 0.0
        
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='/home/bpresa/Mechatronics_and_Robot_Programming/Exercises/Exercise_03/ros_turtlebot3_ws/src/motion_control/turtlebot3_rosbag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/cmd_vel',
            type='geometry_msgs/msg/Twist',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.twist_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Replace with the actual Twist topic name
            self.cmd_vel_callback,
            10
        )

        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/odom',
            type='nav_msgs/msg/Odometry',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Replace with the actual Twist topic name
            self.odom_callback,
            10
        )

        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/error_motion',
            type='geometry_msgs/msg/Twist',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.twist_subscription
        self.odom_subscription 


    def cmd_vel_callback(self, msg):
            # Process the received Twist data
            linear = msg.linear
            angular = msg.angular

            self.linear_vel = msg.linear.x
            self.angular_vel = msg.angular.z

            # Your processing logic here
            # Example: Print the received Twist data
            self.get_logger().info(
                'Received Command Velocity Data: Linear=%s, Angular=%s' %
                (linear, angular)
            )

            # Write Twist data to the rosbag file
            self.writer.write(
                '/cmd_vel',
                serialize_message(msg),
                self.get_clock().now().nanoseconds)
            
            #-----------------------------------------
            # For error
            error = Twist() 
            error.linear.x = self.linear_vel - self.linear_odom
            error.angular.z = self.angular_vel - self.angular_odom

            # Write Twist data to the rosbag file
            self.writer.write(
                '/error_motion',
                serialize_message(error),
                self.get_clock().now().nanoseconds)
            

    def odom_callback(self, msg):
            # Process the received Odometry data
            pose = msg.pose
            twist = msg.twist

            self.linear_odom = twist.twist.linear.x
            self.angular_odom = twist.twist.angular.z

            # Your processing logic here
            # Example: Print the received Twist data
            self.get_logger().info(
                'Received OdometryData: Pose=%s, Twist=%s' %
                (pose, twist)
            )

            # Write Twist data to the rosbag file
            self.writer.write(
                '/odom',
                serialize_message(msg),
                self.get_clock().now().nanoseconds)
            

def main(args=None):
    rclpy.init(args=args)
    sbr = RosbagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

