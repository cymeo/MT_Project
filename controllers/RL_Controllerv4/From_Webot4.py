"""controller_test controller."""
from controller import Robot, Supervisor
import pinocchio as pin 
import numpy as np
from scipy.spatial.transform import Rotation as R

###### Controll Robot and Arm Movements ######################################

#-------------------------------------------------------------------
# define a Supervisor
supervisor = Supervisor()

# URDF Path for UR5e
robot_model = pin.buildModelFromUrdf('./ur5e2.urdf')
frame_names = [frame.name for frame in robot_model.frames]
print("All frames in Webots URDF:", frame_names)
robot_data = robot_model.createData()

#define basic time step
timestep = int(supervisor.getBasicTimeStep())
reset_pose = np.array([0,-np.pi/2, np.pi/2, -np.pi/2,-np.pi/2,0])

#get Robot 
robot_node = supervisor.getFromDef('Robot') 
box = supervisor.getFromDef('Box')
arm = supervisor.getFromDef('Arm')
table = supervisor.getFromDef('Table')
tip = supervisor.getFromDef('Tip')   

#get motors
motors = []
motors.append(supervisor.getDevice('shoulder_pan_joint'))
motors.append(supervisor.getDevice('shoulder_lift_joint'))
motors.append(supervisor.getDevice('elbow_joint'))
motors.append(supervisor.getDevice('wrist_1_joint'))
motors.append(supervisor.getDevice('wrist_2_joint'))
motors.append(supervisor.getDevice('wrist_3_joint'))

sensors = []  
for n, motor in enumerate(motors):
    sensors.append(motor.getPositionSensor())
    sensors[n].enable(timestep)
    motor.setPosition(float('inf'))


def move_robot(joint_velocites):
    global motors 
    theta_dot = np.zeros(6)
    
    for n, motor in enumerate(motors):
        motor.setPosition(float('inf'))
        motor.setVelocity(joint_velocites[n])
    supervisor.step(timestep)            
    pass

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
    global tip
    global robot_data
    theta_dot = np.array([motor.getVelocity() for motor in motors])
    pin.forwardKinematics(robot_model, robot_data, angles, theta_dot)
    pin.updateFramePlacements(robot_model, robot_data) 
    ee_id = robot_model.getFrameId("tool0")  # Get end-effector
    ee_pose = robot_data.oMf[ee_id].homogeneous
    
    p_ee = ee_pose[:3, 3]  # Extract translation
    
    #visualise calculated tip position
    tip_pos = tip.getField("translation")
    tip_pos.setSFVec3f(p_ee.tolist())
    quat =  R.from_matrix(ee_pose[:3,:3]).as_quat()
    # Convert quaternion to axis-angle (Webots format)
    angle = 2 * np.arccos(quat[3])
    axis = quat[:3] / np.linalg.norm(quat[:3])
    tip_rot = tip.getField("rotation")
    tip_rot.setSFRotation([axis[0], axis[1], axis[2], angle])
    return(ee_pose)

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

def check_crash():
    # Check for contact points
    global robot_node, table, arm
    contact_robot = []
    contact_table = []
    contact_arm = []
    contact_robot = robot_node.getContactPoints(includeDescendants=True)
    contact_table = table.getContactPoints(includeDescendants = True)
    contact_arm = arm.getContactPoints(includeDescendants = True)
    crashed = False

    if contact_table:
        crashed = True
        #print("table crash")
    if contact_arm: 
        #print("arm crash")
        crashed = True
    if contact_robot: 
        #print("robot crash")
        crashed = True    
    return crashed
    
def reset_sim():
    global supervisor
    global reset_pose
    global motors
    global robot_node, table, arm 
    supervisor.simulationReset()
    supervisor.simulationResetPhysics()
    arm.resetPhysics()
    table.resetPhysics()
    robot_node.resetPhysics()
    [motor.setVelocity(0) for motor in motors]
    supervisor.step(int(supervisor.getBasicTimeStep()))
    #print("contact after reset ",  check_crash() )
    pass

def show_goal(position, quat):
    global box
    translation = box.getField("translation")
    translation.setSFVec3f(position.tolist())
    
    # Convert quaternion to axis-angle (Webots format)
    angle = 2 * np.arccos(quat[3])
    axis = quat[:3] / np.linalg.norm(quat[:3])
    rot = tip.getField("rotation")
    rot.setSFRotation([axis[0], axis[1], axis[2], angle])
    pass

def get_max_speed(): 
    global robot_model
    p_arm, v_arm = get_arm()
    theta, _ = get_motor_pos()
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
        #print("v_arm2ee", v_arm2ee)
        speed_m = (d_arm -0.2 )/3 #- np.linalg.norm(v_arm2ee)*0.7) / 0.3
        #print("max speed", speed_m)
        if speed_m <=0:
            speed_m = 0         #transferred to maximal velocity towards the arm position      
        max_ee_vel = np.array(dir_arm)*speed_m
        
        # inversed kinematics/ jacobians to transfer to motor velocities 
        ee_id = robot_model.getFrameId("flange")
        J = pin.computeFrameJacobian(robot_model, robot_data, theta, ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        max_theta_dot = np.linalg.pinv(J[:3,:]) @ max_ee_vel
        max_theta_dot= np.clip(max_theta_dot,-3.13, 3.13)
        
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
    