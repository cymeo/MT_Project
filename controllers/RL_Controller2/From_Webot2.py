from controller import Supervisor
import pinocchio as pin
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

###### Control Robot and Arm Movements ######################################

#define Supervisor and basic time step
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Assign joints, motors, and sensors
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

motors = [supervisor.getDevice(name) for name in joint_names]
sensors = []

for i, motor in enumerate(motors): 
    sensor = motor.getPositionSensor()
    if sensor:
        sensor.enable(timestep)
        sensors.append(sensor)
    else:
        sensors.append(None)  # In case a sensor is missing

# Reset to starting pose 
reset_pose = np.array([0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0])

#get objects from Webots
table = supervisor.getFromDef('Table')
robot_node = supervisor.getFromDef('Robot')
box = supervisor.getFromDef('Box')
arm = supervisor.getFromDef('Arm')

# URDF Path for UR5e
robot_model = pin.buildModelFromUrdf('./ur5e.urdf')
robot_data = robot_model.createData()

# Some needed parameters
theta = np.zeros(6)
theta_dot = np.zeros(6)
p_arm = np.zeros(3)
p_ee = np.zeros(3)
v_ee = np.zeros(3)
ee_pose = np.eye(4)
max_ee_vel = np.zeros(3)
p_goal = np.zeros(3)
t_move = 0
position_control = False

d_min = 1 #distance for spereation, e.g 1.5meter

def get_Arm():  
    global p_arm
    p_arm = np.array(arm.getPosition())

def get_motor_info():
    global theta, theta_dot
    for i, motor in enumerate(motors):
        if sensors[i]:
            theta[i] = sensors[i].getValue()
        theta_dot[i] = motor.getVelocity() 
    return theta, theta_dot

def get_robot_info():
    global p_ee, v_ee, ee_pose
    pin.forwardKinematics(robot_model, robot_data, theta, theta_dot)
    pin.updateFramePlacements(robot_model, robot_data)    
    ee_id = robot_model.getFrameId("flange")  # Get end-effector
    ee_pose = robot_data.oMf[ee_id].homogeneous
    p_ee = ee_pose[:3, 3]  # Extract translation
    ee_velocity = pin.getFrameVelocity(robot_model, robot_data, ee_id, pin.ReferenceFrame.WORLD)
    v_ee = ee_velocity.linear  # Translation velocity
    print("v_ee: ", v_ee)

def check_crash():
    return (robot_node.getContactPoints(includeDescendants=True) or
            table.getContactPoints(includeDescendants=True) or
            arm.getContactPoints(includeDescendants=True))

def reset_sim():
    supervisor.simulationReset()
    supervisor.simulationResetPhysics()
    move_robot(reset_pose)
    for motor in motors:
        motor.setVelocity(np.pi)

def move_robot(angles):
    global t_move
    global position_control
    global motors
    global sensors
    global theta
    crashed = False     
    start_time = time.time()
    print("move_robot")
    while True:  
        supervisor.step(timestep)
        if check_crash():
            "crash = True"
            crashed = True
            break
        get_Arm()
        get_robot_info()
        max_theta = np.array(get_max_Vel())

        if np.all(np.abs(theta - angles) <= 0.03): 
            print("angles reached")
            break
        
        if position_control: 
            for i, motor in enumerate(motors):
                motor.setPosition(angles[i])
                theta[i] = sensors[i].getValue()        

        else: 

            for i, motor in enumerate(motors):
                theta[i] = sensors[i].getValue()
                if np.abs(theta[i]-angles[i]) <= 0.005:
                    motor.setVelocity(0)
                else: 
                    if (np.sign(max_theta[i]) == np.sign(angles[i]- theta[i])): 
                        motor.setPosition(float('inf'))  # Disable position control
                        motor.setVelocity(max_theta[i])
                    else: 
                        motor.setPosition(angles[i])
                    
                
                  
        time.sleep(0.001)
    print("moving ended")
    t_move = time.time() - start_time
    return crashed

def get_max_Vel():
    global max_ee_vel
    global d_arm
    global position_control 
    global d_min
    d_arm = np.linalg.norm(p_arm - p_ee)

    if d_arm > d_min:
        position_control = True
        print("position_control")
        pass

    else: 
        print("velocity control")
        position_control = False
        sigma_A = d_min/3         
        max_s = np.exp(-((d_arm - d_min)**2)/(2/sigma_A**2))  # Distance-based velocity limitation
        
        if max_s >=1: 
            max_s = 1
        
        max_ee_vel = max_s * (p_arm - p_ee)/d_arm
        print("max_s: ", max_s)
        print("max_ee_vel: ", max_ee_vel)
        
        #calculate max velocities 
        ee_id = robot_model.getFrameId("flange")
        J = pin.computeFrameJacobian(robot_model, robot_data, theta, ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        max_theta_dot = np.linalg.pinv(J[:3,:]) @ max_ee_vel
        max_theta_dot= np.clip(max_theta_dot, min= -np.pi, max = np.pi)
        current_maxvel = []
        # set maximal velocities 
        print("max_thate dot", max_theta_dot)
        return max_theta_dot
    

def get_distance():
    global p_ee, p_goal
    dist = np.linalg.norm(p_goal - p_ee)
    goal_rot = np.array([0, -1, 0])
    ee_rot_quat = R.from_matrix(ee_pose[:3, :3]).as_quat()
    ee_rot = R.from_quat(ee_rot_quat).as_matrix()
    y_axis = ee_rot[:, 1]  # Extract Y-axis direction
    rot_dist = np.dot(y_axis, goal_rot) / (np.linalg.norm(y_axis) * np.linalg.norm(goal_rot))
    print("gets_dist: ",dist, np.abs(rot_dist))
    return dist, np.abs(rot_dist)

def get_observation():    
    global v_ee
    global p_ee 
    global p_goal
    global theta
    global theta_dot
    global p_arm
    global ee_pose 
    global max_ee_vel 
    global t_move 
    
    p_ee = ee_pose[:3, 3]
    rot = R.from_matrix(ee_pose[:3, :3])
    rot_quat = rot.as_quat()
    pose_ee = np.concatenate((p_ee, rot_quat))
    dist, _ = get_distance()
    
    observation = {
        "pose_ee": pose_ee,
        "v_ee": v_ee,
        "theta": theta,
        "goal": p_goal,
        "d_goal_abs": np.array([dist]),
        "d_goal": np.subtract(p_goal, p_ee),
        "p_arm": p_arm,
        "d_arm_abs": np.array([np.linalg.norm(p_arm - p_ee)]),
        "d_arm": np.subtract(p_arm, p_ee),
        "vel_limit": max_ee_vel,
        "time": np.array([t_move])
    }
    return observation

def show_goal(position):
    global p_goal
    p_goal = np.array(position)
    translation = box.getField("translation")
    translation.setSFVec3f(position.tolist())

# Initialization
get_Arm()
get_motor_info()
get_robot_info()
get_max_Vel()