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


def get_motor_pos(): 
    global motors
    global sensor
    theta = [0,0,0,0,0,0]
    for n, motor in enumerate(motors):
        theta[n] = sensors[n].getValue()
    return np.array(theta)

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
    #print("robot_tip translate webots", p_ee)
    
    #visualise calculated tip position
    tip_pos = tip.getField("translation")
    tip_pos.setSFVec3f(p_ee.tolist())
    return(ee_pose)

def check_crash():
    # Check for contact points
    global robot_node
    global table
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
    supervisor.simulationReset()
    supervisor.simulationResetPhysics()
    move_robot(reset_pose) 
    pass

def move_robot(angles):
    global motors 
    global sensors
    theta = np.zeros(6)
    # while True: 
    for n, motor in enumerate(motors):
        motor.setPosition(angles[n])
        #theta[n] = sensors[n].getValue()
    supervisor.step(timestep)
    # if (np.linalg.norm(theta - angles)<=0.01):
        #     break
    
    pass


def show_goal(position):
    global box
    translation = box.getField("translation")
    translation.setSFVec3f(position.tolist())
    pass
    


