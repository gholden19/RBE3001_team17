# RBE 301 Repository

## Leb 1: System Architecture and Joint Control
- Understand the basics of the operating system, MATLAB, amd Git.
- Understand the architecture and design of the robot arm.
- Run test MATLAB code to validate the system.
- Develop MATLAB scripts and functions to actuate the arm, record data to a file, and plot outputs.
- Perform joint space control on the robot arm by sending/reading joint values of the robot arm.
- Characterize the timing of the system.
- Characterize joint-level motion trajectories

## Lab 2: Forward Kinematics
- Formulate the forward kinematics of the robot arm using DH convention and parameters.
- Create MATLAB scripts and functions to implement your FK calculations.
- Use your FK solution to compute the position of the end effector of the Hephastus Arm using the joint configuration received from the robot, integrating the previously generated code in Lab 1.
- Build a MATLAB stick model and use it for visualizing robot arm motion in real time.
- Implement point-to-point motion between setpoints in the joint space.
- Implement motion following an array of setpoints.
- Visualize and characterize task-space level motion trajectories. 

## Lab 3: Inverse Kinematics and Trajectory Generation
- Formulate the inverse kinematics of the Hephaestus arm.
- Create MATLAB scripts and functions to implement your IK calculations while interacting with the robot.
- Perform trajectory planning in joint space using cubic polynomials.
- Perform trajectory planning in task space using linear interpolation and cubic polynomials.
- Visualize and characterize joint-space and task-space level motion trajectories.

## Lab 4: Velocity Kinematics 
- Formulate the JAcobian matrix and calculate the forward velocity kinematics of the Hephaestus robotic arm.
- Create MATLAB scripts and functions to implement your velocity kinematics calculations while interacting with the robot.
- Implement a numerical approach to solving the inverse kinematics problem.
- Perform trajectory following using speed and direction commands.
- Visualize and characterize motion trajectories. 

## Labs 5-7: Robotic Pick and Place System (Final Project)
- Design a system architecture, workflow, and state machine for a robotic pick and place system.
- Calibrate a CV system.
- Identify objects in a camera frame and localize them in the robot coordinates.
- Control the robot's end-effector (gripper) to perform a task.
- Automatically movethe robot towards objects identified by our CV system, grab them, pick them up, and organize them.

