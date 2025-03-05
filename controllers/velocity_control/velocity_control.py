import pinocchio as pin
import numpy as np

from scipy.spatial.transform import Rotation as R
from controller import Supervisor

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# URDF Path for UR5e
arm = supervisor.getFromDef('Arm')
robot_model = pin.buildModelFromUrdf('/home/cecily/MasterThesis_Cy/MT_Project/controllers/velocity_control/ur5e.urdf')
# Create Data for Kinematics
print("Model successfully loaded with", robot_model.njoints, "joints")
for frame in robot_model.frames:
    print(frame.name)
robot_data = robot_model.createData()

# assign joint, motor and sensors
motors = []
motors.append(supervisor.getDevice('shoulder_pan_joint'))
motors.append(supervisor.getDevice('shoulder_lift_joint'))
motors.append(supervisor.getDevice('elbow_joint'))
motors.append(supervisor.getDevice('wrist_1_joint'))
motors.append(supervisor.getDevice('wrist_2_joint'))
motors.append(supervisor.getDevice('wrist_3_joint'))

sensors = [motor.getPositionSensor() for motor in motors]
for i, motor in enumerate (motors): 
    motor.setPosition(float('inf'))
    sensors[i].enable(timestep)

# assigne and enable motor sensors sensors= []
def get_joint_angles(): 
    sensors= []
    for n, motor in enumerate(motors):
        sensor = motor.getPositionSensor()
        sensor.enable(timestep)
        sensors.append(sensor.getValue())
    return sensors


# End-effector name (matches last link in URDF)
end_effector_name = "tool0"
end_effector_id = robot_model.getFrameId(end_effector_name)

# Desired End-Effector Velocity (Linear + Angular)
xdot = np.array([0, 0.3 , 0, 0, 0, 0])  # Move along y-axis

p_prev = arm.getField("translation").getSFVec3f()
t_prev = supervisor.getTime()

def get_arm():
    global arm
    global p_prev,t_prev
    p_arm = arm.getField("translation").getSFVec3f()
    t_now = supervisor.getTime()
    dt = t_now - t_prev 
    if dt>0: 
        v_arm = [(p_arm[i] - p_prev[i]) / dt for i in range(3)]
    else: 
        v_arm = [0,0,0]
        
    p_prev = p_arm 
    t_prev = t_now
    
    return np.array(p_arm), np.array(v_arm[:3])  

def get_motor_pos(): 
    global motors
    global sensor
    theta = [0,0,0,0,0,0]
    theta_dot = [0,0,0,0,0,0]
    for n, motor in enumerate(motors):
        theta[n] = sensors[n].getValue()
        theta_dot[n]= motor.getVelocity()
    return np.array(theta), np.array(theta_dot)

def get_forward_kinematics(angles): 
    global motors
    global robot_model
    global robot_data
    theta_dot = np.array([motor.getVelocity() for motor in motors])
    pin.forwardKinematics(robot_model, robot_data, angles, theta_dot)
    pin.updateFramePlacements(robot_model, robot_data) 
    ee_id = robot_model.getFrameId("tool0")  # Get end-effector
    ee_pose = robot_data.oMf[ee_id].homogeneous

    return(ee_pose)

def get_max_speed(): 
    global robot_model
    p_arm, v_arm = get_arm()
    theta, theta_dot = get_motor_pos()
    pose_ee = get_forward_kinematics(theta)
    p_ee = pose_ee[:3,3]
    d_arm = np.linalg.norm(p_ee- p_arm)
    

    max_speed = [3.13,3.13,3.13,3.13,3.13,3.13]
    min_speed = [-3.13,-3.13,-3.13,-3.13,-3.13,-3.13]
    if d_arm <= 1: 
        # direction from endeffector to arm 
        dir_arm = [(p_arm[i]-p_ee[i])/np.linalg.norm(p_arm-p_ee) for i in range (3)]
        # velocity of arm towards endeffector 
        v_arm2ee =  np.dot(v_arm, ((p_ee -p_arm)/np.linalg.norm(p_ee-p_arm)))
        # maximal allowed speed for endeffector
        print("v_arm2ee", v_arm2ee)
        speed_m = (d_arm -0.2 - np.linalg.norm(v_arm2ee)*0.7) / 0.3
        print("max speed", speed_m)
        if speed_m <=0:
            return np.zeros(6),np.zeros(6)
        #transferred to maximal velocity towards the arm position      
        max_ee_vel = np.array(dir_arm)*speed_m
        
        # inversed kinematics/ jacobians to transfer to motor velocities 
        ee_id = robot_model.getFrameId("flange")
        J = pin.computeFrameJacobian(robot_model, robot_data, theta, ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        max_theta_dot = np.linalg.pinv(J[:3,:]) @ max_ee_vel
        max_theta_dot= np.clip(max_theta_dot, min= -3.13, max = 3.13)
        
        for i, theta_constr in enumerate (max_theta_dot): 
            if np.sign(theta_constr) == 0 : 
                max_speed[i] = 0
                min_speed[i] = 0 
            elif np.sign(theta_constr) <0: 
                max_speed[i] = 3.13
                min_speed[i] = theta_constr
            elif np.sign(theta_constr) >0: 
                max_speed[i] = theta_constr
                min_speed[i] = -3.13
        
    return max_speed, min_speed
           

while True: 
    supervisor.step(timestep)
    # Get Current Joint Positions
    q = np.array(get_joint_angles())
    qdot = np.zeros(len(q))  # Assume zero velocity initially

    # Compute Forward Kinematics
    pin.forwardKinematics(robot_model, robot_data, q, qdot)
    pin.updateFramePlacements(robot_model, robot_data)
    
    ee_id = robot_model.getFrameId("flange") 
    ee_pose = robot_data.oMf[ee_id]
    p_ee = ee_pose.translation 
    
    if np.abs(p_ee[1]) >= 0.3: 
        if (np.sign(p_ee[1])>0):
            xdot = [0,-0.2,0,0,0,0]
        else: 
            xdot = [0,0.2,0,0,0,0]
        
 
    # Compute Jacobian
    J = pin.computeFrameJacobian(robot_model, robot_data, q, end_effector_id, pin.LOCAL_WORLD_ALIGNED)

    # Compute Joint Velocities using Pseudoinverse
    qdot_command = np.linalg.pinv(J) @ xdot
    
    
    s_max, s_min = get_max_speed()
       

    # Apply Joint Velocities
    for i, motor in enumerate(motors):
        
        qdot_command[i] = np.clip(qdot_command[i], min = s_min[i], max = s_max[i])
        motor.setVelocity(qdot_command[i])

