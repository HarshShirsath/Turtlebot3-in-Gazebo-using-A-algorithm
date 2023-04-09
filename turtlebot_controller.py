#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

current_x = 0.0
current_y = 0.0
current_yaw = 0.0

# Define the points
def read_path(filename='./path.txt'):
    poses = []
    with open(filename, 'r') as f:
        lines = f.readlines()

    for line in lines:
        x, y = line.strip().split(',')
        poses.append([float(x), float(y)])

    return poses

def get_orientation(msg):
    global current_x, current_y, current_yaw
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    _, _, current_yaw = euler_from_quaternion(quaternion)


def move_turtlebot_to_goal(goal_x, goal_y):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    twist = Twist()
    reached_goal = False

    while not reached_goal and not rospy.is_shutdown():
        # Calculate the distance and angle to the goal point
        dx = goal_x - current_x
        dy = goal_y - current_y
        goal_angle = math.atan2(dy, dx)
        distance_to_goal = math.sqrt(dx**2 + dy**2)

        # Rotate the robot towards the goal point
        if abs(goal_angle - current_yaw) > 0.1:
            twist.linear.x = 0.0
            twist.angular.z = goal_angle - current_yaw
        # Move the robot towards the goal point
        else:
            twist.linear.x = min(0.2, distance_to_goal)
            twist.angular.z = 0.0

            if distance_to_goal < 0.05:
                reached_goal = True
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        pub.publish(twist)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('turtlebot_move')
    odom_sub = rospy.Subscriber('/odom', Odometry, get_orientation)
    poses = read_path()
    for point in poses:
        goal_x, goal_y = point
        move_turtlebot_to_goal(goal_x, goal_y)