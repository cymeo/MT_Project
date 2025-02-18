"""controller_test controller."""
from controller import Supervisor
from ikpy.chain import Chain
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

###### Controll Robot and Arm Movements ######################################

#-------------------------------------------------------------------
# define a Supervisor
supervisor = Supervisor()
# Load the chain by specifying the elements you want to include
AL_mask = [False, False, True, True, True, True, True, True, False, False]
robot_chain = Chain.from_urdf_file("ur5e.urdf",active_links_mask = AL_mask)
#Print the chain to verify the correct links are included
#print(robot_chain)

#define basic time step
timestep = int(supervisor.getBasicTimeStep())
reset_pose = np.array([0,-np.pi/2, np.pi/2, -np.pi/2,-np.pi/2,0])

robot_tippose = np.array([0,0,0])

#get Robot 
robot_node = supervisor.getFromDef('Robot') 
box = supervisor.getFromDef('Box')
arm = supervisor.getFromDef('Arm')
table = supervisor.getFromDef('Table')



#get motors
motors = []
motors.append(supervisor.getDevice('shoulder_pan_joint'))
motors.append(supervisor.getDevice('shoulder_lift_joint'))
motors.append(supervisor.getDevice('elbow_joint'))
motors.append(supervisor.getDevice('wrist_1_joint'))
motors.append(supervisor.getDevice('wrist_2_joint'))
motors.append(supervisor.getDevice('wrist_3_joint'))

def get_Arm(): 
    arm_pos = arm.getPosition()
    return arm_pos       
        
def get_motor_pos(): 
    sensors= []
    for n, motor in enumerate(motors):
        sensor = motor.getPositionSensor()
        sensor.enable(timestep)
        sensors.append(sensor.getValue())
    return(np.array(sensors))

def get_forward_kinematics(angles): 
    
    angles_trans = np.concatenate((np.array([0]), np.array(angles), np.array([0,0,0])))
    #print(angles_trans)
    robotTipMatrix = robot_chain.forward_kinematics(angles_trans)
    #rotationrot = robotTipMatrix[:3,:3]
    robot_tippose = robotTipMatrix[3,:3]
    return(robotTipMatrix)

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


def show_goal(position):
    #print('pos =' , position.tolist())
    translation = box.getField("translation")
    #position = [position[0],position[1], position[2] ]
    #print('transl before:', translation.getSFVec3f() )
    translation.setSFVec3f(position.tolist())
    pass
    


