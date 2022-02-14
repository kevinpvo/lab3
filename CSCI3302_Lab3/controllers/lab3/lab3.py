"""lab3 controller."""
# Copyright University of Colorado Boulder 2021
# CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math

# TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 0.0001 # [rad/s]
MAX_SPEED_MS = 0.22 # [m/s]
AXLE_LENGTH = 0.160 # [m]



MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

# create the Robot instance.
robot = Robot()
MAX_SPEED = robot.getMaxVelocity()
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
# You should not use target_pos for storing waypoints. Leave it unmodified and 
# use your own variable to store waypoints leading up to the goal
target_pos = ('inf', 'inf') 
robot_parts = []

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

# Rotational Motor Velocity [rad/s]
vL = 0
vR = 0

# TODO
# Create you state and goals (waypoints) variable here
# You have to MANUALLY figure out the waypoints, one sample is provided for you in the instructions
goal = [(1.5, 1)]

i = 0

while robot.step(timestep) != -1:

    # STEP 2.1: Calculate sources of error
    # x_g, y_g are the desired locations(goal) and x_r, y_r are the current locaitons 
    x_g = goal[i][0]
    y_g = goal[i][1]
    theta_g = (7/12)*(math.pi) # guessing 
    rho = math.sqrt((pose_x - x_g)^2 + (pose_y - y_g)^2)
    alpha = math.atan((y_g - pose_y)/(x_g - pose_x)) - pose_theta
    eta = theta_g - pose_theta
#     print()
    pass   
    
    # STEP 2.2: Feedback Controller
    p_1 = 2
    p_2 = 2
    p_3 = 2
    x_dot = (p_1)*rho
    theta_dot = (p_2)*alpha + (p_3)*eta
    pass
    
    # testing values 
    print("rho = {}".format(rho))
    print("alpha = {}".format(alpha))
    print("eta = {}".format(eta))

    # STEP 1: Inverse Kinematics Equations (vL and vR as a function dX and dTheta)
    # Note that vL and vR in code is phi_l and phi_r on the slides/lecture
    # 
    time = SIM_TIMESTEP/1000 # time step in second
    d = AXLE_LENGTH # axle length in milimeters  

    x_dot_i = (pose_x/time) 
    x_dot_r = x_dot_i/math.acos(pose_theta)
    omega_dot_r = pose_theta/time
    phi_right = x_dot_r + omega_dot_r*d/2 #algebra manipulation of equations
    phi_left = x_dot_r + omega_dot_r*d/2
    vL = phi_left * MAX_SPEED/MAX_SPEED_MS
    vR = phi_right * MAX_SPEED/MAX_SPEED_MS
    pass
    
    # STEP 2.3: Proportional velocities
    vL = vL/MAX_SPEED # Left wheel velocity in rad/s
    vR = vR/MAX_SPEED # Right wheel velocity in rad/s
    pass

    # STEP 2.4: Clamp wheel speeds
    # we dont need this  
    pass


    
    # TODO
    # Use Your Lab 2 Odometry code after these 2 comments. We will supply you with our code next week 
    # after the Lab 2 deadline but you free to use your own code if you are sure about its correctness

    
    # NOTE that the odometry should ONLY be a function of 
    # (vL, vR, MAX_SPEED, MAX_SPEED_MS, timestep, AXLE_LENGTH, pose_x, pose_y, pose_theta)
    # Odometry code. Don't change speeds (vL and vR) after this line
    
    
    

    ########## End Odometry Code ##################
    
    ########## Do not change ######################
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28
    ###############################################

    # TODO
    # Set robot motors to the desired velocities if some stop criteria is met 

    cm_error = 0.03
    theta_error = (math.pi)/6

    if(eta <= theta_error and alpha < cm_error and rho <= cm_error): 
        robot_parts[MOTOR_LEFT].setVelocity(0)
        robot_parts[MOTOR_RIGHT].setVelocity(0)

    