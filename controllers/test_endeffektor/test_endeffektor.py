
from controller import Robot, Supervisor,Connector
from ikpy.chain import Chain
from math import pi 


supervisor = Supervisor()

# Load the chain by specifying the elements you want to include
robot_chain = Chain.from_urdf_file("ur5e.urdf", active_links_mask=[False,False,True,True,True,True,True,True,False,False])

motors = []
motors.append(supervisor.getDevice('shoulder_pan_joint'))
motors.append(supervisor.getDevice('shoulder_lift_joint'))
motors.append(supervisor.getDevice('elbow_joint'))
motors.append(supervisor.getDevice('wrist_1_joint'))
motors.append(supervisor.getDevice('wrist_2_joint'))
motors.append(supervisor.getDevice('wrist_3_joint'))

timestep = int(supervisor.getBasicTimeStep())
while supervisor.step(timestep) != -1:
    motors[1].setPosition(-pi)
    

