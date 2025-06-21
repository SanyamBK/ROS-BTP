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
    i = min(max(int((pose.x - 2) // 2), 0), 4)
    j = min(max(int((pose.y - 2) // 2), 0), 4)
    belief[i][j] = max(belief[i][j] - 0.2, 0.0)
    rospy.loginfo(f"[R1] Reduced belief[{i}][{j}] to {belief[i][j]:.2f} based on position.")

def find_max_uncertain_cell():
    max_val = max(max(row) for row in belief)
    candidates = [(i, j) for i in range(5) for j in range(5) if belief[i][j] == max_val]
    return sorted(candidates)[0]

def predict_other_robot_target():
    # Local replica of robot2's min-uncertainty target logic
    min_val = min(min(row) for row in belief)
    candidates = [(i, j) for i in range(5) for j in range(5) if belief[i][j] == min_val]
    return sorted(candidates)[0]

def is_belief_fully_reduced():
    return all(all(cell <= 0.01 for cell in row) for row in belief)

# ------------------- Main Execution -------------------

def main():
    global pose
    rospy.init_node('robot1_controller')

    # Set turtle pen color (red)
    rospy.wait_for_service('/robot1/turtle1/set_pen')
    try:
        set_pen = rospy.ServiceProxy('/robot1/turtle1/set_pen', SetPen)
        set_pen(255, 0, 0, 3, 0)
    except rospy.ServiceException as e:
        rospy.logwarn(f"[R1] Failed to set pen: {e}")

    # Publishers and Subscribers
    pub = rospy.Publisher('/robot1/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/robot1/turtle1/pose', Pose, pose_callback)
    rospy.Subscriber('/comm_channel', String, lambda msg: rospy.loginfo(f"[COMM] {msg.data}"))
    rospy.Subscriber('/belief_channel', BeliefGrid, belief_callback)
    comm_pub = rospy.Publisher('/comm_channel', String, queue_size=10)
    belief_pub = rospy.Publisher('/belief_channel', BeliefGrid, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz loop

    while not rospy.is_shutdown():
        update_belief_based_on_position()

        if pose is None:
            rate.sleep()
            continue
        
        if is_belief_fully_reduced():
            rospy.loginfo("[R1] Belief fully reduced. Stopping.")
            stop_msg = Twist()  # all zeros
            pub.publish(stop_msg)
            rate.sleep()
            continue

        # Plan movement
        target = find_max_uncertain_cell()
        target_x = 2 + target[0] * 2
        target_y = 2 + target[1] * 2
        angle_to_target = math.atan2(target_y - pose.y, target_x - pose.x)
        angle_diff = angle_to_target - pose.theta

        # Predict and compare with other robot
        predicted_other = predict_other_robot_target()
        if predicted_other != target:
            comm_msg = String()
            comm_msg.data = "[R1] Detected disagreement, triggering communication!"
            comm_pub.publish(comm_msg)
            flat = [val for row in belief for val in row]
            belief_msg = BeliefGrid(grid=flat, sender_id="robot1")
            belief_pub.publish(belief_msg)

        # Move toward target
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = angle_diff
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()
