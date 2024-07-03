# Pattern Matching and Reverse Docking
## - Working Sequence
  1. The Pattern Matching and Reverse Docking is intitiated when the initialization msg is published in a topic using the command "ros2 topic pub --once /initialize_charging std_msgs/msg/Bool data:\ true\ ".
  2. Now a goal pose (predetermined pose near the charging dock) is send to the Nav2 Action Server "navigate_to_pose".
  3. When the robot reach the Goal successfully, the Pattern Matching is started by sending an initialization msg to the action server created for Pattern Matching in the PatternMatcherNode.
  4. ### Steps involved in Pattern Matching:
    1. In the PatternMatcherNode, the Data from 2D-Lidar (in topic /scan) is saved in to a global variable in real-time.
    2. When the Pattern Matching is initialized, the Lidar data is filtered using the range and angle limit, for the sake of Computation Speed. 
    3. This Data is converted into corresponding PointCloud data using PCL (library) and trignometry.
    4. The Obtained PointCloud is then Clustered into Groups using the EuclideanClusterExtraction available in PCL
    5. Then each PointCloud cluster is checked for the pattern matching with the PCD (PointCloud Data) using IterativeClosestPoint in PCL
    6. Matched Pattern Positions are filtered by using a Threshold (FitnessScore < 0.00015)
    7. The Best fitted Pattern deatails are then published into a topic and saved into a variable.
    8. All these steps are iterated in a loop at the rate equivalent to the computation speed required for the Pattern Matching.
    9. By doing this, The average of Transforms Obtained in each iterations is send to the Action Client (RobotControlNode).
  5. After the Accurate potition data of the Matched Pattern is obtained, that details are send to the Nav2 Action Server as new Goal Pose to move.
  6. Upon Reaching the Goal(At the Transform of Matched Pattern), Reverse Docking is initialized.
  7. With the data obtained from encoders in the realtime, by subscribing to the topic /joint_states, a 180Deg turn is taken to face backwards towards the Charging Dock then takes 2 encoder steps back Performing Reverse Docking.

## - How to Run the Program:
  1. Clone the repo into a workspace/src.
  2. Colcon build the workspace.
  3. Open a terminal into the workspace directory and source it.
  4. Then run the command "ros2 launch pattern_matcher pattern_matcher.launch.py" to launch all the necessary programs.
  5. After the launching of Gazebo and successful Spawning of Robot in the World is completed, run the following command "ros2 topic pub --once /initialize_charging std_msgs/msg/Bool data:\ true\ "
