#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
from turtlesim.srv import SetPen
from multi_robot_bsp.msg import BeliefGrid

belief = [[0.5 for _ in range(5)] for _ in range(5)]
pose = None

# ------------------- Helper Functions -------------------

def belief_difference(other_grid):
    diff = 0.0
    for i in range(5):
        for j in range(5):
            idx = i * 5 + j
            diff += abs(belief[i][j] - other_grid[idx])
    return diff

def belief_callback(msg):
    global belief
    if msg.sender_id == "robot2":
        rospy.loginfo(f"[R1] Received belief from {msg.sender_id}")
        for i in range(5):
            for j in range(5):
                idx = i * 5 + j
                belief[i][j] = (belief[i][j] + msg.grid[idx]) / 2.0
        d = belief_difference(msg.grid)
        rospy.loginfo(f"[R1] Belief diff from {msg.sender_id}: {d:.4f}")

def pose_callback(msg):
    global pose
    pose = msg

def update_belief_based_on_position():
    global belief, pose
    if pose is None:
        return

    # 1️⃣ PREDICTION STEP — Diffuse uncertainty
    new_belief = [[0.0 for _ in range(5)] for _ in range(5)]
    for i in range(5):
        for j in range(5):
            # Keep 80% in place, spread 20% to neighbors
            main = 0.8 * belief[i][j]
            spread = 0.2 * belief[i][j]
            new_belief[i][j] += main
            neighbors = [
                (max(i-1,0), j), (min(i+1,4), j),
                (i, max(j-1,0)), (i, min(j+1,4))
            ]
            for ni, nj in neighbors:
                new_belief[ni][nj] += spread / len(neighbors)
    belief[:] = new_belief

    # 2️⃣ CORRECTION STEP — Bayesian + spill to neighbors
    i = min(max(int((pose.x - 2) // 2), 0), 4)
    j = min(max(int((pose.y - 2) // 2), 0), 4)

    P_clear_given_clear = 0.9
    P_clear_given_not_clear = 0.1

    prior = belief[i][j]
    likelihood = P_clear_given_clear * (1 - prior) + P_clear_given_not_clear * prior
    correction = prior * likelihood

    neighbor_fraction = 0.2
    main_fraction = 1.0 - neighbor_fraction

    neighbors = [
        (max(i-1,0), j), (min(i+1,4), j),
        (i, max(j-1,0)), (i, min(j+1,4))
    ]

    belief[i][j] = correction * main_fraction
    for ni, nj in neighbors:
        belief[ni][nj] += correction * neighbor_fraction / len(neighbors)

    # Normalize grid
    total = sum([sum(row) for row in belief])
    for ii in range(5):
        for jj in range(5):
            belief[ii][jj] /= total

    rospy.loginfo(f"[R1] Bayesian updated belief[{i}][{j}] = {belief[i][j]:.3f}")

def find_max_uncertain_cell():
    max_val = max(max(row) for row in belief)
    candidates = [(i, j) for i in range(5) for j in range(5) if belief[i][j] == max_val]
    return sorted(candidates)[0]

def predict_other_robot_target():
    min_val = min(min(row) for row in belief)
    candidates = [(i, j) for i in range(5) for j in range(5) if belief[i][j] == min_val]
    return sorted(candidates)[0]

def is_belief_fully_reduced():
    return all(all(cell <= 0.01 for cell in row) for row in belief)

# ------------------- Main Execution -------------------

def main():
    global pose
    rospy.init_node('robot1_controller')

    rospy.wait_for_service('/robot1/turtle1/set_pen')
    try:
        set_pen = rospy.ServiceProxy('/robot1/turtle1/set_pen', SetPen)
        set_pen(255, 0, 0, 3, 0)
    except rospy.ServiceException as e:
        rospy.logwarn(f"[R1] Failed to set pen: {e}")

    pub = rospy.Publisher('/robot1/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/robot1/turtle1/pose', Pose, pose_callback)
    rospy.Subscriber('/comm_channel', String, lambda msg: rospy.loginfo(f"[COMM] {msg.data}"))
    rospy.Subscriber('/belief_channel', BeliefGrid, belief_callback)
    comm_pub = rospy.Publisher('/comm_channel', String, queue_size=10)
    belief_pub = rospy.Publisher('/belief_channel', BeliefGrid, queue_size=10)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        update_belief_based_on_position()

        if pose is None:
            rate.sleep()
            continue

        if is_belief_fully_reduced():
            rospy.loginfo("[R1] Belief fully reduced. Stopping.")
            stop_msg = Twist()
            pub.publish(stop_msg)
            rate.sleep()
            continue

        target = find_max_uncertain_cell()
        target_x = 2 + target[0] * 2
        target_y = 2 + target[1] * 2
        angle_to_target = math.atan2(target_y - pose.y, target_x - pose.x)
        angle_diff = angle_to_target - pose.theta

        i = min(max(int((pose.x - 2) // 2), 0), 4)
        j = min(max(int((pose.y - 2) // 2), 0), 4)
        local_belief = belief[i][j]

        motion_noise = np.random.uniform(-0.3, 0.3)

        msg = Twist()
        msg.linear.x = max(0.1, 0.5 + 1.5 * local_belief + motion_noise)
        msg.angular.z = angle_diff + np.random.uniform(-0.3, 0.3)

        predicted_other = predict_other_robot_target()
        if predicted_other != target:
            comm_msg = String()
            comm_msg.data = "[R1] Disagreement detected, triggering communication!"
            comm_pub.publish(comm_msg)
            flat = [val for row in belief for val in row]
            belief_msg = BeliefGrid(grid=flat, sender_id="robot1")
            belief_pub.publish(belief_msg)

        pub.publish(msg)
        rospy.loginfo(f"[R1] Moving towards target {target} | Local belief={local_belief:.2f} | "
                      f"Speed={msg.linear.x:.2f} | Motion noise={motion_noise:.3f}")
        rate.sleep()

if __name__ == '__main__':
    main()
