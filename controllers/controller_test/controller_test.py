"""controller_test controller."""
from controller import Robot, Supervisor,Connector
from ikpy.chain import Chain
import numpy as np
from scipy.spatial.transform import Rotation as R

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


box0 = supervisor.getFromDef("Box")
box1 = supervisor.getFromDef("Box1")
box2 = supervisor.getFromDef("Box2")
box3 = supervisor.getFromDef("Box3")
box4 = supervisor.getFromDef("Box4")
box5 = supervisor.getFromDef("Box5")
box6 = supervisor.getFromDef("Box6")
box7 = supervisor.getFromDef("Box7")
box8 = supervisor.getFromDef("Box8")
boxes = [box1,box2,box3,box4,box5,box6,box7,box8]

boxpos = [-0.2, 0.5,0.1]

######Set Position of Boxes 
for i, box in enumerate(boxes):              
    tf = box.getField('translation') # translation field
    tf.setSFVec3f(boxpos)
    boxpos = (np.add(np.array(boxpos), np.array([0.15,0,0]))) # choose next location for the box
    boxpos = boxpos.tolist()
    if (i == 3):
        boxpos = [-0.2, 0.60,0.1]

i = 0 



robot = supervisor.getFromDef("Robot")

while supervisor.step(timestep) != -1:
    
    box_position = box0.getPosition()
    box_rotation = box0.getOrientation() 
    box_rotationMatrix = np.array(box_rotation).reshape(3, 3)

    print("box_rotation: ", box_rotationMatrix)
    r = R.from_matrix(box_rotationMatrix)
      
    ikAnglesD=robot_chain.inverse_kinematics(box_position,box_rotationMatrix, "all")
    
    sensors = np.zeros(10)
    for n, motor in enumerate(motors):
        motor.setPosition(ikAnglesD[n+2])
        sensor = motor.getPositionSensor()
        sensor.enable(timestep)
        sensors[n+1]= sensor.getValue()
        
    robotTipMatrix = robot_chain.forward_kinematics(sensors)
    robot_realrot = robotTipMatrix[:3,:3]
    print('robot rotation', robot_realrot )