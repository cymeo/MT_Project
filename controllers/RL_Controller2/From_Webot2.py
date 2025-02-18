"""controller_test controller."""
from controller import Supervisor
import pinocchio as pin
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

###### Controll Robot and Arm Movements ######################################

#define Supervisor and basic time step
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
# assign joint, motor and sensors
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

motors = [supervisor.getDevice(name) for name in joint_names]

for motor in motors: 
    motor.setPosition(float('inf'))
    sensor = motor.getPositionSensor()
    sensor.enable(timestep)
    
#reset to starting pose 
reset_pose = np.array([0,-np.pi/2, np.pi/2, -np.pi/2,-np.pi/2,0])

#get objects from webots
robot_node = supervisor.getFromDef('Robot') 
box = supervisor.getFromDef('Box')
arm = supervisor.getFromDef('Arm')
table = supervisor.getFromDef('Table')

#URDF Path for UR5e
robot_model = pin.buildModelFromUrdf('./ur5e.urdf')
robot_data = robot_model.createData()
theta = []
theta_dot = []
    
########Functions#####################

def get_Arm(): 
    arm_pos = arm.getPosition()
    return arm_pos       
        
def get_motor_pos():       
    sensors= []
    for n, motor in enumerate(motors):
        sensors.append(sensor.getValue())
    return np.array(sensors)

def get_robot_info(): 
    #get motor position and velocity from Webot 
    theta = get_motor_pos()
    theta_dot = np.array([motor.getVelocity() for motor in motors])
    # calculate endeffector pose and velocity using pinochio 
    pin.forwardKinematics(robot_model, robot_data, theta, theta_dot)
    pin.updateFramePlacements(robot_model, robot_data)    
    ee_id = robot_model.getFrameId("flange") #get endeffector
    ee_pose = robot_data.oMf[ee_id]
    p_ee = ee_pose.translation # only translation
    ee_velocity = pin.getFrameVelocity(robot_model,robot_data,ee_id, pin.ReferenceFrame.WORLD)
    v_ee =  ee_velocity.linear # translation velocity 
    return theta, theta_dot, p_ee, v_ee

def check_crash():
    crashed = False        
    if  robot_node.getContactPoints(includeDescendants=True): 
        crashed = True
    if table.getContactPoints(includeDescendants = True): 
        crashed = True
    if arm.getContactPoints(includeDescendants=True):
        crashed = True
        
    return crashed
    
def reset_sim():
    supervisor.simulationReset()
    supervisor.simulationResetPhysics()
    move_robot(reset_pose)
    for i , motor in enumerate(motors):
        motor.setMinVelocity(-1)     
        motor.setMaxVelocity(1)
    pass

def move_robot(angles):
    crashed = False
    sensors= np.zeros(6)        
    start_time = time.time()
    while True: 
        supervisor.step(timestep)
        current_pos = get_motor_pos()
        crashed = check_crash()
        if crashed == True: 
            break
        
        if all (abs(current_pos-angles) <= 0.01): 
            break
        
        for n, motor in enumerate(motors):
            motor.setPosition(angles[n])
            sensor = motor.getPositionSensor()
            sensor.enable(timestep)
            sensors[n] = sensor.getValue()
        
        
        # Check for timeout
        if time.time() - start_time > 5:
            print("Motor movement timeout, stucked!")        
            break
        time.sleep(0.001)
    
    return sensors, crashed

# def set_ee_vel(self, ee_velocity):
    
#        # Compute Jacobian
#     J = pin.computeFrameJacobian(self.model, self.data, self.theta, self.ee_id, pin.LOCAL_WORLD_ALIGNED)
    
#     ee_velocity = np.concatenate([ee_velocity,0,0,0])
#     theta_dot_command = np.linalg.pinv(J) @ ee_velocity
    
#     for i, motor in motors: 
#         if np.abs(self.theta[i]) <= (np.pi-0.01):
#             motor.setVelocity(theta_dot_command(i))   
#         else: 
#             motor.setVelocity(0)
#             return True, i 
#     pass

def set_max_Vel(d_arm,p_ee, p_arm):    
    if d_arm <= 1:
        max_s = (d_arm -1.6 - 0.1)/0.25 #  distance arm/robot in meters, -1.6m/s*1s human max speed*response time, 0.25s stoptime robot
        max_ee_vel = max_s*(p_arm - p_ee)/np.linalg.norm(p_arm -p_ee) ## how to choose max velocity? --> in direction of the arm ? 
    else: 
        max_ee_vel = np.array([1,1,1])
    ee_id = robot_model.getFrameId("flange")
    J = pin.computeFrameJacobian(robot_model, robot_data, theta, ee_id, pin.LOCAL_WORLD_ALIGNED)
    max_theta_dot = np.linalg.pinv(J) @ max_ee_vel

    for i , motor in enumerate(motors): 
        if max_theta_dot[i]<= 0: 
            motor.setMinVelocity(max_theta_dot[i])
        if max_theta_dot[i] >= 0:     
            motor.setMaxVelocity(max_theta_dot[i])
    return max_ee_vel   
    
def show_goal(position):
    translation = box.getField("translation")
    translation.setSFVec3f(position.tolist())
    pass
    


