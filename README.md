# **Decentralized Multi-Robot Belief Space Planning (Turtlesim + ROS)**

This project demonstrates decentralized coordination between two robots (simulated turtles) to reduce uncertainty in a shared belief map through local movement and inter-robot communication. It is implemented in **ROS Noetic** using **Turtlesim** and simulates coordination through a belief-sharing protocol.

---

## 🐢 Project Features

- Two robots independently estimate a 5x5 belief grid
- Each robot reduces uncertainty (belief) based on physical location
- Robots predict each other’s behavior to avoid redundant work
- When predictions differ, they share beliefs and update accordingly
- Visual distinction via colored pen trails (red for R1, green for R2)

---

## 📁 Project Structure

```
multi_robot_bsp/
├── launch/
│   └── multi_robot.launch
├── msg/
│   └── BeliefGrid.msg
├── scripts/
│   ├── robot1_node.py
│   └── robot2_node.py
├── CMakeLists.txt
└── package.xml

```

---

## 🚀 How to Run

> ⚠️ Assumes `ROS Noetic` and `catkin_ws` already set up.

### 1. Build the workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
````

### 2. Launch the system

```bash
roslaunch multi_robot_bsp multi_robot.launch
```

This will:

* Start two turtlesim windows
* Assign red and green pens
* Launch control nodes for both robots

---

## 💡 How It Works

Each robot:

* Maintains a local `5x5` belief grid
* Reduces uncertainty by visiting grid cells
* Predicts the other robot’s movement
* If predictions don’t match, triggers communication
* Publishes its belief grid over a shared topic
* Updates its own belief using received data

---

## 📦 Custom Message

The `BeliefGrid.msg` format:

```plaintext
float32[] grid     # 25 floats (row-major order)
string sender_id   # "robot1" or "robot2"
```

---

## 🎯 Goals

* Demonstrate decentralized planning under partial information
* Simulate robot coordination without centralized control
* Test basic belief-space planning ideas with ROS

---

### 📈 System Architecture Diagram

```
     +---------------------+            +---------------------+
     |     Robot 1 (R1)    |            |     Robot 2 (R2)    |
     |---------------------|            |---------------------|
     |  Local Belief Grid  |            |  Local Belief Grid  |
     |  + Motion Planner   |            |  + Motion Planner   |
     |  + Target Predictor |            |  + Target Predictor |
     +---------|-----------+            +----------|----------+
               |                                   |
               |       Predicts Other's Target     |
               |<------------------------------->  |
               |                                   |
     +---------v-----------+            +----------v----------+
     | If Disagreement,    |            |  If Disagreement,   |
     | Send BeliefGrid.msg |            |  Send BeliefGrid.msg|
     +---------|-----------+            +----------|----------+
               |                                   |
               |      /belief_channel (topic)      |
               +-------------^---------------------+
                             |
                    +------------------+
                    | BeliefGrid.msg   |
                    |  - grid (25)     |
                    |  - sender_id     |
                    +------------------+
```

Turtlesim UI:

* R1 draws RED trail
* R2 draws GREEN trail

Movement:

* R1 seeks max uncertainty
* R2 seeks min uncertainty
* Grid cell = 2 + i \* 2 (in 1–10 range)

