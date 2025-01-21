"""controller_test controller."""
from controller import Robot, Supervisor,Connector
from ikpy.chain import Chain
import numpy as np

from numpy import pi
from scipy.spatial.transform import Rotation as R

###### let Robot follow a Box ######################################

#-------------------------------------------------------------------
# define a Supervisor
supervisor = Supervisor()

# Load the chain by specifying the elements you want to include
robot_chain = Chain.from_urdf_file("ur5e.urdf")
# Print the chain to verify the correct links are included
print(robot_chain)

####define basic time step
timestep = int(supervisor.getBasicTimeStep())

motors = []
motors.append(supervisor.getDevice('shoulder_pan_joint'))
motors.append(supervisor.getDevice('shoulder_lift_joint'))
motors.append(supervisor.getDevice('elbow_joint'))
motors.append(supervisor.getDevice('wrist_1_joint'))
motors.append(supervisor.getDevice('wrist_2_joint'))
motors.append(supervisor.getDevice('wrist_3_joint'))
print("motors_length", np.size(motors))

box0 = supervisor.getFromDef("Box")

robot = supervisor.getFromDef("Robot")

while supervisor.step(timestep) != -1:
    
    box_position = box0.getPosition()
    box_rotation = box0.getOrientation() 
    box_rotationMatrix = np.array(box_rotation).reshape(3, 3)

    #print("box_rotation: ", box_rotationMatrix)
    r = R.from_matrix(box_rotationMatrix)
      
    ikAnglesD=robot_chain.inverse_kinematics(box_position,box_rotationMatrix, "all")
    
    reset_pose = np.array([0,-np.pi/2, np.pi/2, -np.pi/2,-np.pi/2,0])

    #print("ikangels",ikAnglesD)
    
    sensors = np.zeros(10)
    for n, motor in enumerate(motors):
        motor.setPosition(reset_pose[n])
        #motor.setPosition(ikAnglesD[n+2])
        sensor = motor.getPositionSensor()
        sensor.enable(timestep)
        sensors[n+1]= sensor.getValue()
    robotTipMatrix = robot_chain.forward_kinematics(sensors)
    robot_realrot = robotTipMatrix[:3,:3]
    #print("sensors: ",sensors)

    #print('robot rotation', robot_realrot )
