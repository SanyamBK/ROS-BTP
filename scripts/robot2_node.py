#!/usr/bin/env python3
import rospy
import random
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String

belief = [[0.5 for _ in range(5)] for _ in range(5)]
pose = None
rospy.Subscriber('/comm_channel', String, lambda msg: rospy.loginfo(f"[COMM] {msg.data}"))


def pose_callback(msg):
    global pose
    pose = msg

def update_belief():
    x, y = random.randint(0, 4), random.randint(0, 4)
    belief[x][y] = min(belief[x][y] + 0.1, 1.0)
    rospy.loginfo(f"[R2] Updated belief[{x}][{y}] = {belief[x][y]:.2f}")

def find_min_uncertain_cell():
    min_val = 2.0  # greater than max possible belief
    min_cell = (0, 0)
    for i in range(5):
        for j in range(5):
            if belief[i][j] < min_val:
                min_val = belief[i][j]
                min_cell = (i, j)
    return min_cell
    
def predict_other_robot_target():
    # Dummy prediction: assumes robot1 uses max uncertainty
    from robot1_node import find_max_uncertain_cell
    return find_max_uncertain_cell()

def main():
    global pose
    rospy.init_node('robot2_controller')
    pub = rospy.Publisher('/robot2/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/robot2/turtle1/pose', Pose, pose_callback)
    comm_pub = rospy.Publisher('/comm_channel', String, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        update_belief()

        if pose is None:
            rate.sleep()
            continue

        # Move toward the least uncertain cell
        target = find_min_uncertain_cell()
        target_x = 2 + target[0] * 2
        target_y = 2 + target[1] * 2

        angle_to_target = math.atan2(target_y - pose.y, target_x - pose.x)
        angle_diff = angle_to_target - pose.theta

        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = angle_diff
        predicted_other = predict_other_robot_target()
        if predicted_other != target:
            comm_msg = String()
            comm_msg.data = "[R2] Disagreement detected, triggering communication!"
            comm_pub.publish(comm_msg)
            
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()

