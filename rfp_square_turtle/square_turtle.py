from math import radians
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose


class SquareTurtle(Node):
    '''
    Time-based open-loop velocity controller for a turtle that makes it move in a square.
    '''
    class State(Enum):
        START = auto()
        RIGHT = auto()
        TURN_RIGHT_UP = auto()
        UP = auto()
        TURN_UP_LEFT = auto()
        LEFT = auto()
        TURN_LEFT_DOWN = auto()
        DOWN = auto()
        END = auto()

    def __init__(self):
        super().__init__('square_turtle')
        self.create_subscription(TurtlePose, 'pose', self.pose_callback, 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.declare_parameter('linear_velocity', 1.0)
        self.declare_parameter('angular_velocity', radians(30))
        self.declare_parameter('side_length', 1.0)
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.side_length = self.get_parameter('side_length').value
        self.current_pose = None
        self.state = self.State.START
        self.current_twist = Twist()
        self.end_time = self.get_clock().now()

    def timer_callback(self):
        current_time = self.get_clock().now()
        remaining_time = (self.end_time - current_time).nanoseconds / 1e9

        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.side_length = self.get_parameter('side_length').value

        self.get_logger().info(f'Current pose: {self.current_pose}')
        self.get_logger().info(f'Motion time remaining: {remaining_time} s')

        if remaining_time <= self.timer_period / 2:
            self.current_twist = Twist()
            if (self.state == self.State.START or
                self.state == self.State.TURN_RIGHT_UP or
                self.state == self.State.TURN_UP_LEFT or
                    self.state == self.State.TURN_LEFT_DOWN):
                self.current_twist.linear.x = self.linear_velocity
                motion_duration = Duration(
                    seconds=self.side_length / self.linear_velocity)
                self.end_time = current_time + motion_duration
                self.state = self.State(self.state.value + 1)
                self.get_logger().info(
                    f'Moving {self.state.name} with twist {self.current_twist}')
            elif (self.state == self.State.RIGHT or
                  self.state == self.State.UP or
                  self.state == self.State.LEFT or
                  self.state == self.State.DOWN):
                self.current_twist.angular.z = self.angular_velocity
                motion_duration = Duration(
                    seconds=(radians(90) / self.angular_velocity))
                self.end_time = current_time + motion_duration
                self.state = self.State(self.state.value + 1)
                self.get_logger().info(
                    f'Turning {self.state.name} with twist {self.current_twist}')
            elif self.state == self.State.END:
                self.get_logger().info('Finished moving in a square')

        self.twist_pub.publish(self.current_twist)

    def pose_callback(self, msg):
        self.current_pose = msg


def main(args=None):
    rclpy.init(args=args)

    square_turtle = SquareTurtle()

    rclpy.spin(square_turtle)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
