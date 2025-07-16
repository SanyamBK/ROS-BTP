#!/usr/bin/env python3
import rospy
import math
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
    if msg.sender_id == "robot1":
        rospy.loginfo(f"[R2] Received belief from {msg.sender_id}")
        for i in range(5):
            for j in range(5):
                idx = i * 5 + j
                belief[i][j] = (belief[i][j] + msg.grid[idx]) / 2.0
        d = belief_difference(msg.grid)
        rospy.loginfo(f"[R2] Belief diff from {msg.sender_id}: {d:.4f}")

def pose_callback(msg):
    global pose
    pose = msg

def update_belief_based_on_position():
    global belief, pose
    if pose is None:
        return

    # CORRECTION STEP — update the cell the robot is in
    i = min(max(int((pose.x - 2) // 2), 0), 4)
    j = min(max(int((pose.y - 2) // 2), 0), 4)

    # Example: sensor expects "unclear" if uncertain
    P_unclear_given_uncertain = 0.8
    P_unclear_given_certain = 0.2

    prior = belief[i][j]
    likelihood = P_unclear_given_uncertain * prior + P_unclear_given_certain * (1 - prior)
    belief[i][j] = prior * likelihood

    rospy.loginfo(f"[R2] Bayesian updated belief[{i}][{j}] = {belief[i][j]:.2f}")

def find_min_uncertain_cell():
    min_val = min(min(row) for row in belief)
    candidates = [(i, j) for i in range(5) for j in range(5) if belief[i][j] == min_val]
    return sorted(candidates)[0]

def predict_other_robot_target():
    max_val = max(max(row) for row in belief)
    candidates = [(i, j) for i in range(5) for j in range(5) if belief[i][j] == max_val]
    return sorted(candidates)[0]

def is_belief_fully_reduced():
    return all(all(cell <= 0.01 for cell in row) for row in belief)

# ------------------- Main Execution -------------------

def main():
    global pose
    rospy.init_node('robot2_controller')

    rospy.wait_for_service('/robot2/turtle1/set_pen')
    try:
        set_pen = rospy.ServiceProxy('/robot2/turtle1/set_pen', SetPen)
        set_pen(0, 255, 0, 3, 0)
    except rospy.ServiceException as e:
        rospy.logwarn(f"[R2] Failed to set pen: {e}")

    pub = rospy.Publisher('/robot2/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/robot2/turtle1/pose', Pose, pose_callback)
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
            rospy.loginfo("[R2] Belief fully reduced. Stopping.")
            stop_msg = Twist()
            pub.publish(stop_msg)
            rate.sleep()
            continue

        # === PLAN MOVEMENT ===
        target = find_min_uncertain_cell()
        target_x = 2 + target[0] * 2
        target_y = 2 + target[1] * 2
        angle_to_target = math.atan2(target_y - pose.y, target_x - pose.x)
        angle_diff = angle_to_target - pose.theta

        # Current cell belief
        i = min(max(int((pose.x - 2) // 2), 0), 4)
        j = min(max(int((pose.y - 2) // 2), 0), 4)
        local_belief = belief[i][j]

        msg = Twist()
        msg.linear.x = 0.5 + 1.5 * local_belief
        msg.angular.z = angle_diff

        # === PREDICTION STEP — add motion-based uncertainty ===
        motion_uncertainty = abs(msg.linear.x) * 0.05 + abs(msg.angular.z) * 0.02
        for ii in range(5):
            for jj in range(5):
                belief[ii][jj] = min(belief[ii][jj] + motion_uncertainty, 1.0)

        # OPTIONAL normalize
        total = sum([sum(row) for row in belief])
        for ii in range(5):
            for jj in range(5):
                belief[ii][jj] /= total

        predicted_other = predict_other_robot_target()
        if predicted_other != target:
            comm_msg = String()
            comm_msg.data = "[R2] Disagreement detected, triggering communication!"
            comm_pub.publish(comm_msg)
            flat = [val for row in belief for val in row]
            belief_msg = BeliefGrid(grid=flat, sender_id="robot2")
            belief_pub.publish(belief_msg)

        pub.publish(msg)
        rospy.loginfo(f"[R2] Moving towards target {target} | Local belief={local_belief:.2f} | Speed={msg.linear.x:.2f} | Added motion noise={motion_uncertainty:.4f}")
        rate.sleep()

if __name__ == '__main__':
    main()
