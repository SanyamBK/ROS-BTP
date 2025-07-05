#!/usr/bin/env python3
import rospy
import math
import numpy as np
import random
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
from turtlesim.srv import SetPen
from multi_robot_bsp.msg import BeliefGrid

# Initialize belief as numpy array for easier computation
belief = np.full((5, 5), 0.2, dtype=np.float32)  # Start with low certainty
pose = None
previous_pose = None

# MR-AC Algorithm parameters
CONSENSUS_THRESHOLD = 0.1  # Threshold for consensus verification
COMMUNICATION_THRESHOLD = 0.15  # Threshold for triggering communication
AGREEMENT_WINDOW = 5  # Number of iterations to check for agreement
MAX_CONSENSUS_ITERATIONS = 10  # Maximum iterations for consensus

# Bayesian filter parameters
MOTION_NOISE = 0.1
SENSOR_NOISE = 0.05
OBSERVATION_CONFIDENCE = 0.8

# MR-AC state variables
consensus_state = "INITIAL"  # INITIAL, CONVERGING, CONVERGED, DISAGREEMENT
other_robot_belief = None
consensus_history = []
last_communication_time = 0
communication_trigger_count = 0

# ------------------- MR-AC Algorithm Implementation -------------------

def mr_ac_consensus():
    """
    Multi-Robot Asynchronous Consensus (MR-AC) main algorithm
    Implements distributed consensus with self-triggered communication
    """
    global consensus_state, other_robot_belief, consensus_history
    
    if other_robot_belief is None:
        rospy.loginfo("[R1] MR-AC: Waiting for initial communication")
        return False
    
    # Step 1: Calculate consensus metric
    consensus_error = calculate_consensus_error()
    consensus_history.append(consensus_error)
    
    # Keep only recent history
    if len(consensus_history) > AGREEMENT_WINDOW:
        consensus_history.pop(0)
    
    rospy.loginfo(f"[R1] MR-AC: Consensus error = {consensus_error:.4f}, "
                  f"State = {consensus_state}")
    
    # Step 2: Verify Agreement Condition (AC)
    agreement_verified = verify_ac()
    
    # Step 3: State machine for consensus
    if consensus_state == "INITIAL":
        if agreement_verified:
            consensus_state = "CONVERGED"
            rospy.loginfo("[R1] MR-AC: Initial consensus achieved")
            return True
        else:
            consensus_state = "CONVERGING"
    
    elif consensus_state == "CONVERGING":
        if agreement_verified:
            consensus_state = "CONVERGED"
            rospy.loginfo("[R1] MR-AC: Consensus converged")
            return True
        elif consensus_error > COMMUNICATION_THRESHOLD:
            consensus_state = "DISAGREEMENT"
            rospy.logwarn("[R1] MR-AC: Disagreement detected")
    
    elif consensus_state == "CONVERGED":
        if not agreement_verified:
            consensus_state = "DISAGREEMENT"
            rospy.logwarn("[R1] MR-AC: Consensus lost, disagreement detected")
    
    elif consensus_state == "DISAGREEMENT":
        # Step 4: Enforce Agreement Condition
        enforce_ac()
        if agreement_verified:
            consensus_state = "CONVERGING"
    
    return agreement_verified

def verify_ac():
    """
    Verify Agreement Condition (AC)
    Checks if robots have reached sufficient consensus
    """
    if len(consensus_history) < 3:
        return False
    
    # Check if consensus error is below threshold
    current_error = consensus_history[-1]
    if current_error > CONSENSUS_THRESHOLD:
        return False
    
    # Check if error is stable (not increasing)
    error_trend = np.diff(consensus_history[-3:])
    if np.mean(error_trend) > 0.01:  # Error is increasing
        return False
    
    rospy.loginfo(f"[R1] Verify AC: PASSED (error={current_error:.4f})")
    return True

def enforce_ac():
    """
    Enforce Agreement Condition (AC)
    Applies corrective measures to achieve consensus
    """
    global belief, consensus_state
    
    if other_robot_belief is None:
        trigger_communication("ENFORCE_AC")
        return
    
    rospy.loginfo("[R1] Enforce AC: Applying consensus correction")
    
    # Apply weighted consensus update
    consensus_weight = 0.3  # How much to adjust towards consensus
    
    # Calculate consensus target (weighted average)
    consensus_target = (belief + other_robot_belief) / 2.0
    
    # Apply gradual correction towards consensus
    belief = (1 - consensus_weight) * belief + consensus_weight * consensus_target
    
    # Ensure probability distribution remains valid
    belief = np.clip(belief, 0.01, 0.99)
    belief = belief / np.sum(belief)
    
    rospy.loginfo("[R1] Enforce AC: Consensus correction applied")

def calculate_consensus_error():
    """
    Calculate consensus error metric between robots
    """
    if other_robot_belief is None:
        return float('inf')
    
    # L2 norm of difference
    error = np.linalg.norm(belief - other_robot_belief)
    return error

def self_triggered_communication():
    """
    Self-triggered communication mechanism
    Decides when communication is necessary based on consensus state
    """
    global last_communication_time, communication_trigger_count
    
    current_time = time.time()
    time_since_last_comm = current_time - last_communication_time
    
    # Trigger conditions
    should_communicate = False
    reason = ""
    
    # 1. Disagreement detected
    if consensus_state == "DISAGREEMENT":
        should_communicate = True
        reason = "DISAGREEMENT"
    
    # 2. High consensus error
    elif len(consensus_history) > 0 and consensus_history[-1] > COMMUNICATION_THRESHOLD:
        should_communicate = True
        reason = "HIGH_ERROR"
    
    # 3. Periodic communication for consensus maintenance
    elif time_since_last_comm > 10.0 and consensus_state != "CONVERGED":
        should_communicate = True
        reason = "PERIODIC"
    
    # 4. Significant belief change
    elif detect_significant_belief_change():
        should_communicate = True
        reason = "BELIEF_CHANGE"
    
    if should_communicate:
        trigger_communication(reason)
        return True
    
    return False

def detect_significant_belief_change():
    """
    Detect if local belief has changed significantly
    """
    # This could be enhanced with belief history tracking
    # For now, use a simple heuristic based on entropy change
    current_entropy = calculate_entropy(belief)
    
    # If entropy is very high or very low, it indicates significant change
    if current_entropy > 3.5 or current_entropy < 1.0:
        return True
    
    return False

def trigger_communication(reason):
    """
    Trigger communication with specific reason
    """
    global last_communication_time, communication_trigger_count
    
    communication_trigger_count += 1
    last_communication_time = time.time()
    
    rospy.loginfo(f"[R1] COMMUNICATION TRIGGERED: {reason} "
                  f"(Count: {communication_trigger_count})")
    
    return True

# ------------------- Enhanced Bayesian Filter -------------------

def motion_model_prediction():
    """Enhanced motion model with better uncertainty propagation"""
    global belief, pose, previous_pose
    
    if pose is None or previous_pose is None:
        return
    
    dx = pose.x - previous_pose.x
    dy = pose.y - previous_pose.y
    motion_distance = math.sqrt(dx**2 + dy**2)
    
    if motion_distance > 0.1:
        # Apply motion blur with proper normalization
        new_belief = np.zeros_like(belief)
        
        for i in range(5):
            for j in range(5):
                prob = belief[i, j]
                
                # Spread probability based on motion uncertainty
                for di in range(-2, 3):
                    for dj in range(-2, 3):
                        ni, nj = i + di, j + dj
                        if 0 <= ni < 5 and 0 <= nj < 5:
                            distance = math.sqrt(di**2 + dj**2)
                            weight = math.exp(-distance**2 / (2 * MOTION_NOISE**2))
                            new_belief[ni, nj] += prob * weight
        
        belief = new_belief / np.sum(new_belief)
        rospy.loginfo(f"[R1] Motion model applied, distance: {motion_distance:.2f}")

def sensor_model_update():
    """Enhanced sensor model with exploration-based uncertainty reduction"""
    global belief, pose
    
    if pose is None:
        return
    
    grid_x = min(max(int((pose.x - 2) // 2), 0), 4)
    grid_y = min(max(int((pose.y - 2) // 2), 0), 4)
    
    # Simulate realistic sensor observation
    observed_probability = simulate_sensor_observation(grid_x, grid_y)
    
    # Create likelihood function
    likelihood = np.ones_like(belief)
    
    for i in range(5):
        for j in range(5):
            distance = math.sqrt((i - grid_x)**2 + (j - grid_y)**2)
            
            if distance == 0:
                # Direct observation with high confidence
                likelihood[i, j] = OBSERVATION_CONFIDENCE
            else:
                # Indirect evidence with distance decay
                likelihood[i, j] = (1 - OBSERVATION_CONFIDENCE) * \
                                   math.exp(-distance**2 / (2 * SENSOR_NOISE**2))
    
    # Bayesian update
    belief = belief * likelihood
    belief = belief / np.sum(belief)
    
    # Exploration effect - reduce uncertainty at current location
    exploration_factor = 0.1
    belief[grid_x, grid_y] = max(belief[grid_x, grid_y] - exploration_factor, 0.05)
    belief = belief / np.sum(belief)
    
    rospy.loginfo(f"[R1] Sensor update at [{grid_x}][{grid_y}], "
                  f"observed: {observed_probability:.3f}")

def simulate_sensor_observation(grid_x, grid_y):
    """Simulate noisy sensor observation"""
    true_prob = belief[grid_x, grid_y]
    noise = random.gauss(0, SENSOR_NOISE)
    observed = max(0.01, min(0.99, true_prob + noise))
    return observed

def environmental_belief_decay():
    """Model environmental dynamics"""
    global belief
    
    decay_rate = 0.005  # Slower decay
    belief = belief + decay_rate * (0.2 - belief)  # Drift towards low certainty
    belief = np.clip(belief, 0.01, 0.99)
    belief = belief / np.sum(belief)

# ------------------- Utility Functions -------------------

def calculate_entropy(grid):
    """Calculate Shannon entropy of belief grid"""
    grid_flat = grid.flatten()
    grid_flat = grid_flat[grid_flat > 0]  # Remove zeros
    entropy = -np.sum(grid_flat * np.log2(grid_flat))
    return entropy

def find_exploration_target():
    """Find target for exploration based on information gain"""
    best_target = (0, 0)
    best_score = -1
    
    for i in range(5):
        for j in range(5):
            # Score based on uncertainty and distance from current position
            uncertainty_score = belief[i, j]
            
            # Add distance factor to encourage exploration
            if pose is not None:
                current_grid_x = min(max(int((pose.x - 2) // 2), 0), 4)
                current_grid_y = min(max(int((pose.y - 2) // 2), 0), 4)
                distance = math.sqrt((i - current_grid_x)**2 + (j - current_grid_y)**2)
                distance_bonus = 0.1 * distance  # Encourage exploration of distant cells
            else:
                distance_bonus = 0
            
            total_score = uncertainty_score + distance_bonus
            
            if total_score > best_score:
                best_score = total_score
                best_target = (i, j)
    
    return best_target

# ------------------- Communication Handlers -------------------

def belief_callback(msg):
    """Enhanced belief fusion with MR-AC integration"""
    global belief, other_robot_belief
    
    if msg.sender_id == "robot2":
        rospy.loginfo(f"[R1] Received belief from {msg.sender_id}")
        
        # Store other robot's belief for consensus calculation
        other_robot_belief = np.array(msg.grid).reshape(5, 5)
        
        # Apply MR-AC consensus
        consensus_achieved = mr_ac_consensus()
        
        if consensus_achieved:
            rospy.loginfo("[R1] Consensus achieved through MR-AC")
        else:
            rospy.loginfo("[R1] MR-AC consensus in progress")

def pose_callback(msg):
    global pose, previous_pose
    previous_pose = pose
    pose = msg

def update_belief_with_bayesian_filter():
    """Complete Bayesian filter update with MR-AC integration"""
    global belief
    
    if pose is None:
        return
    
    # Step 1: Motion model prediction
    motion_model_prediction()
    
    # Step 2: Sensor model update
    sensor_model_update()
    
    # Step 3: Environmental dynamics
    environmental_belief_decay()
    
    # Step 4: Self-triggered communication check
    self_triggered_communication()
    
    # Log comprehensive state
    entropy = calculate_entropy(belief)
    max_belief = np.max(belief)
    min_belief = np.min(belief)
    
    rospy.loginfo(f"[R1] Belief updated - Entropy: {entropy:.3f}, "
                  f"Range: [{min_belief:.3f}, {max_belief:.3f}], "
                  f"Consensus: {consensus_state}")

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

    rate = rospy.Rate(1)  # 1 Hz

    rospy.loginfo("[R1] MR-AC Robot Controller Started")

    while not rospy.is_shutdown():
        # Apply Bayesian filter with MR-AC
        update_belief_with_bayesian_filter()

        if pose is None:
            rate.sleep()
            continue

        # Plan movement based on exploration strategy
        target = find_exploration_target()
        target_x = 2 + target[0] * 2
        target_y = 2 + target[1] * 2
        angle_to_target = math.atan2(target_y - pose.y, target_x - pose.x)
        angle_diff = angle_to_target - pose.theta

        # Check if communication should be triggered
        if self_triggered_communication():
            # Send belief update
            flat = belief.flatten().tolist()
            belief_msg = BeliefGrid(grid=flat, sender_id="robot1")
            belief_pub.publish(belief_msg)
            
            comm_msg = String()
            comm_msg.data = f"[R1] MR-AC Communication: {consensus_state}"
            comm_pub.publish(comm_msg)

        # Move toward target
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = angle_diff
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()