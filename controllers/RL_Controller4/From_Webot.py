"""controller_test controller."""
from controller import Robot, Supervisor
from ikpy.chain import Chain
import pinocchio as pin 
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

###### Controll Robot and Arm Movements ######################################

#-------------------------------------------------------------------
# define a Supervisor
supervisor = Supervisor()
# Load the chain by specifying the elements you want to include

AL_mask = [False, True, True, True, True, True, True,False]
robot_chain = Chain.from_urdf_file("Robot.urdf", active_links_mask = AL_mask)

# URDF Path for UR5e
robot_model = pin.buildModelFromUrdf('./ur5e.urdf')

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

sensors= np.zeros(6)
for n, motor in enumerate(motors):
    sensor = motor.getPositionSensor()
    sensor.enable(timestep)


def get_motor_pos(): 
    for n, motor in enumerate(motors):
        sensors[n] = sensor.getValue()
    return sensors

def get_forward_kinematics(angles): 
    
    theta_dot = np.array([motor.getVelocity() for motor in motors])
    pin.forwardKinematics(robot_model, robot_data, angles, theta_dot)
    pin.updateFramePlacements(robot_model, robot_data) 
    ee_id = robot_model.getFrameId("tool0")  # Get end-effector
    ee_pose = robot_data.oMf[ee_id].homogeneous
    p_ee = ee_pose[:3, 3]  # Extract translation
    print("robot_tip translate webots", p_ee)
    
    translation = tip.getField("translation")
    #position = [position[0],position[1], position[2] ]
    #print('transl before:', translation.getSFVec3f() )
    translation.setSFVec3f(p_ee.tolist())
    pass
    
    
    
    return(ee_pose)

def check_crash():
        # Check for contact points
    robot_node = supervisor.getFromDef('Robot') 

    table = supervisor.getFromDef('Table')
    contact_robot = robot_node.getContactPoints(includeDescendants=True)
    contact_table = table.getContactPoints(includeDescendants = True)
    
    crashed = False
    
    if contact_table: 
        crashed = True
    if contact_robot: 
        crashed = True
        print("crashed")    
    return crashed
    

def reset_sim():
    supervisor.simulationReset()
    supervisor.simulationResetPhysics()
    move_robot(reset_pose) 
    robot_node = supervisor.getFromDef('Robot') 
    #print("contat in reset: ", contact_robot)
    pass

def move_robot(angles):
    crashed = False
    sensors= np.zeros(6)        
    start_time = time.time()
    
    supervisor.step(timestep)
    sensors = get_motor_pos()
    crashed = check_crash()
    if crashed == True: 
        return sensors, crashed
    
    for n, motor in enumerate(motors):
        motor.setPosition(angles[n])
        # sensor = motor.getPositionSensor()
        # sensor.enable(timestep)
        # sensors[n] = sensor.getValue()  
        
    return sensors, crashed


def show_goal(position):
    #print('pos =' , position.tolist())
    translation = box.getField("translation")
    #position = [position[0],position[1], position[2] ]
    #print('transl before:', translation.getSFVec3f() )
    translation.setSFVec3f(position.tolist())
    pass
    


