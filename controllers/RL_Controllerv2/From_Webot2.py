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

def check_crash():
    # Check for contact points
    global robot_node
    global table
    contact_robot = []
    contact_table = []
    contact_robot = robot_node.getContactPoints(includeDescendants=True)
    contact_table = table.getContactPoints(includeDescendants = True)
    
    crashed = False
    
    if contact_table: 
        crashed = True
    if contact_robot: 
        crashed = True   
    return crashed
    
def reset_sim():
    global supervisor
    global reset_pose
    global motors
    global robot_node
    global table
    supervisor.simulationReset()
    supervisor.simulationResetPhysics()
    [motor.setVelocity(0) for motor in motors]

    supervisor.step(int(supervisor.getBasicTimeStep()))
    pass

def show_goal(position, quat):
    global box
    translation = box.getField("translation")
    translation.setSFVec3f(position.tolist())
    
    # Convert quaternion to axis-angle (Webots format)
    angle = 2 * np.arccos(quat[3])
    axis = quat[:3] / np.linalg.norm(quat[:3])
    rot = box.getField("rotation")
    rot.setSFRotation([axis[0], axis[1], axis[2], angle])
    pass
    


