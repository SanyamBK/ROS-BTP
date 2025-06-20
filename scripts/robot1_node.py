#!/usr/bin/env python3
import rospy
import random
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String


belief = [[0.5 for _ in range(5)] for _ in range(5)]
pose = None

def comm_callback(msg):
    rospy.loginfo(f"[COMM] {msg.data}")

rospy.Subscriber('/comm_channel', String, comm_callback)


def pose_callback(msg):
    global pose
    pose = msg

def update_belief():
    x, y = random.randint(0, 4), random.randint(0, 4)
    belief[x][y] = min(belief[x][y] + 0.1, 1.0)
    rospy.loginfo(f"Updated belief[{x}][{y}] to {belief[x][y]:.2f}")

def find_max_uncertain_cell():
    max_val = -1
    max_cell = (0, 0)
    for i in range(5):
        for j in range(5):
            if belief[i][j] > max_val:
                max_val = belief[i][j]
                max_cell = (i, j)
    return max_cell
    

def predict_other_robot_target():
    # Dummy prediction: assumes robot2 uses min uncertainty
    from robot2_node import find_min_uncertain_cell
    return find_min_uncertain_cell()


def main():
    global pose
    rospy.init_node('robot1_controller')
    pub = rospy.Publisher('/robot1/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/robot1/turtle1/pose', Pose, pose_callback)
    comm_pub = rospy.Publisher('/comm_channel', String, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        update_belief()

        if pose is None:
            rate.sleep()
            continue

        # Find most uncertain cell
        target = find_max_uncertain_cell()
        
        predicted_other = predict_other_robot_target()
        
        if predicted_other != target:
            comm_msg = String()
            comm_msg.data = "[R1] Detected disagreement, triggering communication!"
            comm_pub.publish(comm_msg)

        # Map grid (0–4) to turtlesim coordinates (approx. 1–10)
        target_x = 2 + target[0] * 2
        target_y = 2 + target[1] * 2

        # Compute angle to target
        angle_to_target = math.atan2(target_y - pose.y, target_x - pose.x)
        angle_diff = angle_to_target - pose.theta

        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = angle_diff  # steer toward the most uncertain cell
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()

