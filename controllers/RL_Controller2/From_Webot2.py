# """controller_test controller."""
# from controller import Supervisor
# import pinocchio as pin
# import numpy as np
# from scipy.spatial.transform import Rotation as R
# import time

# ###### Controll Robot and Arm Movements ######################################

# #define Supervisor and basic time step
# supervisor = Supervisor()
# timestep = int(supervisor.getBasicTimeStep())
# # assign joint, motor and sensors
# joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
#             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# motors = [supervisor.getDevice(name) for name in joint_names]
# print("motors: ", motors)
# sensors = []

# for i, motor in enumerate (motors): 
#     motor.setPosition(float('inf'))
#     sensors.append(motor.getPositionSensor())
#     sensors[i].enable(timestep)
 
# #reset to starting pose 
# reset_pose = np.array([0,-np.pi/2, np.pi/2, -np.pi/2,-np.pi/2,0])


# #get objects from webots
# robot_node = supervisor.getFromDef('Robot') 
# box = supervisor.getFromDef('Box')
# arm = supervisor.getFromDef('Arm')
# table = supervisor.getFromDef('Table')

# #URDF Path for UR5e
# robot_model = pin.buildModelFromUrdf('./ur5e.urdf')
# robot_data = robot_model.createData()
# d_arm =[]
# ### some needed parameters #############
# theta = np.zeros(6)
# theta_dot = np.zeros(6)
# p_arm = np.array([])
# p_ee = np.array([])
# v_ee = np.array([])
# ee_pose = np.array([])
# max_ee_vel = np.array([])
# p_goal = np.array([])
# t_move = 0




# ########Functions#####################
# def get_Arm():  
#     p_arm = arm.getPosition()
#     pass
        
# def get_motor_info():
#     for i, motor in enumerate(motors):
#         theta[i] = sensors[i].getValue()
#         theta_dot[i] = motor.getVelocity() 
#     return theta, theta_dot #returns theta, theta_dot

# def get_robot_info(): #retrune p_ee, v_ee
#     #get motor position and velocity from Webot 
#     # calculate endeffector pose and velocity using pinochio 
#     print("get robot info")
#     pin.forwardKinematics(robot_model, robot_data, theta, theta_dot)
#     pin.updateFramePlacements(robot_model, robot_data)    
#     ee_id = robot_model.getFrameId("flange") #get endeffector
#     print("ee_id")
#     ee_pose = robot_data.oMf[ee_id]
#     print("ee_pose", ee_pose)
#     p_ee = ee_pose.translation # only translation
#     ee_velocity = pin.getFrameVelocity(robot_model,robot_data,ee_id, pin.ReferenceFrame.WORLD)
#     v_ee =  ee_velocity.linear # translation velocity 
#     pass

# def check_crash():
#     crashed = False        
#     if  robot_node.getContactPoints(includeDescendants=True): 
#         crashed = True
#     if table.getContactPoints(includeDescendants = True): 
#         crashed = True
#     if arm.getContactPoints(includeDescendants=True):
#         crashed = True
        
#     return crashed
    
# def reset_sim():
#     supervisor.simulationReset()
#     supervisor.simulationResetPhysics()
#     move_robot(reset_pose)
#     for i , motor in enumerate(motors):
#         motor.setMinVelocity(-1)     
#         motor.setMaxVelocity(1)
#     pass

# def move_robot(angles):
#     crashed = False     
#     start_time = time.time()
#     while True:  
#         supervisor.step(timestep)
#         crashed = check_crash()
#         if crashed == True: 
#             break
#         if all (abs(theta-angles) <= 0.01): 
#             break
        
#         #set position and get actual position
#         for i, motor in enumerate(motors):
#             get_Arm()
#             get_robot_info()
#             set_max_Vel()
#             motor.setPosition(angles[n])
#             theta[i] = sensors[i].getValue()    
        
#         time.sleep(0.001)
        
#     t_move = time.time() - start_time
#     return crashed

# # def set_ee_vel(self, ee_velocity):
    
# #        # Compute Jacobian
# #     J = pin.computeFrameJacobian(self.model, self.data, self.theta, self.ee_id, pin.LOCAL_WORLD_ALIGNED)
    
# #     ee_velocity = np.concatenate([ee_velocity,0,0,0])
# #     theta_dot_command = np.linalg.pinv(J) @ ee_velocity
    
# #     for i, motor in motors: 
# #         if np.abs(self.theta[i]) <= (np.pi-0.01):
# #             motor.setVelocity(theta_dot_command(i))   
# #         else: 
# #             motor.setVelocity(0)
# #             return True, i 
# #     pass

# def set_max_Vel():
#     d_arm = np.norm(p_arm - p_ee )     
#     if d_arm <= 1:
#         max_s = (d_arm -1.6 - 0.1)/0.25 #  distance arm/robot in meters, -1.6m/s*1s human max speed*response time, 0.25s stoptime robot
#         max_ee_vel = max_s*(p_arm - p_ee)/d_arm ## how to choose max velocity? --> in direction of the arm ? 
#     else: 
#         max_ee_vel = np.array([1,1,1])
#     ee_id = robot_model.getFrameId("flange")
#     J = pin.computeFrameJacobian(robot_model, robot_data, theta, ee_id, pin.LOCAL_WORLD_ALIGNED)
#     max_theta_dot = np.linalg.pinv(J) @ max_ee_vel

#     for i , motor in enumerate(motors): 
#         if max_theta_dot[i]< 0: 
#             motor.setMinVelocity(max_theta_dot[i])
#             motor.setMaxVeloocity(1)
            
#         elif max_theta_dot[i] > 0:     
#             motor.setMaxVelocity(max_theta_dot[i])
#             motor.setMinVelocity(-1)
            
#         elif max_theta_dot[i] ==0: 
#             motor.setMaxVelocity(0)
#             motor.setMinVelocity(0)
#     pass


# def get_distance(self):
#     dist = np.linalg.norm(self.goal - self.p_ee[:3])
#     goal_rot = np.array([0,-1,0])
#     ee_rot_quat = R.from_quat(self.p_ee[3:])
#     ee_rot= ee_rot_quat.as_matrix()
#     # rot_dist = np.arccos((np.trace(np.dot(goal_rot.T,ee_rot))-1)/2)
#         #Compute the cosine similarity (dot product)
#     y_axis = ee_rot[:, 1]  # Assuming 3x3 rotation matrix
#     y_down = np.array([0,-1,0])  # downward direction for y 
#     rot_dist = np.dot(y_axis, y_down) / (np.linalg.norm(y_axis) * np.linalg.norm(y_down))
#     return dist, np.absolute(rot_dist)

# def get_observation():    
#     print("ee_pose: ", ee_pose)    
#     p_ee = np.array(ee_pose[:3,3])
#     rot= R.from_matrix(ee_pose[:3,:3])
#     rot_quat= np.array(rot.as_quat())
#     pose_ee = np.concatenate((p_ee,rot_quat))
    
#     dist,_ = get_distance()
    
#     observation = { 
#         "pose_ee": pose_ee, 
#         "v_ee": v_ee,
#         "theta": theta,
#         "goal": p_goal,
#         "d_goal_abs": dist,
#         "d_goal": np.subtract(p_goal, p_ee),
#         "p_arm": p_arm,
#         "d_arm_abs": d_arm,
#         "d_arm": np.subtract(p_arm, p_ee),
#         "vel_limit": max_ee_vel,
#         "time": t_move
#     }
    
#     return observation
    
# def show_goal(position):
#     p_goal = position
#     translation = box.getField("translation")
#     translation.setSFVec3f(position.tolist())
#     pass
    


# #init: 
# get_Arm()
# theta, theta_dot = get_motor_info()
# get_robot_info()


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
    crashed = False     
    start_time = time.time()
    
    while True:  
        supervisor.step(timestep)
        if check_crash():
            crashed = True
            break
        if np.all(np.abs(theta - angles) <= 0.01): 
            break
        
        for i, motor in enumerate(motors):
            get_Arm()
            get_robot_info()
            motor.setPosition(angles[i])
            set_max_Vel()        
        time.sleep(0.001)
    
    t_move = time.time() - start_time
    return crashed

def set_max_Vel():
    global max_ee_vel
    d_arm = np.linalg.norm(p_arm - p_ee)
    print("d_arm", d_arm)
    if d_arm <= 1:
        
        max_s = (d_arm - 1.6*0.3) / 0.25  # Distance-based velocity limitation
        print("max_S", max_s)
        
        
        if max_s >=1: 
            max_s = 1
        max_ee_vel = max_s * (p_arm - p_ee) / d_arm
    else:
        max_ee_vel = np.array([1, 1, 1])
    
    ee_id = robot_model.getFrameId("flange")
    J = pin.computeFrameJacobian(robot_model, robot_data, theta, ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    max_theta_dot = np.linalg.pinv(J[:3,:]) @ max_ee_vel
    max_theta_dot= np.clip(max_theta_dot, min= -np.pi, max = np.pi)
    for i, motor in enumerate(motors):
        motor.setVelocity(np.abs(max_theta_dot[i]))
    pass

def get_distance():
    global p_ee, p_goal
    dist = np.linalg.norm(p_goal - p_ee)
    goal_rot = np.array([0, -1, 0])
    ee_rot_quat = R.from_matrix(ee_pose[:3, :3]).as_quat()
    ee_rot = R.from_quat(ee_rot_quat).as_matrix()
    y_axis = ee_rot[:, 1]  # Extract Y-axis direction
    rot_dist = np.dot(y_axis, goal_rot) / (np.linalg.norm(y_axis) * np.linalg.norm(goal_rot))
    return dist, np.abs(rot_dist)

def get_observation():    
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
set_max_Vel()