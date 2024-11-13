"""controller_test controller."""
from controller import Robot, Supervisor, Connector
from ikpy.chain import Chain
import numpy as np
from math import pi 
import time 
import GetIK ##### extrascript for everything ur5e-control related

#-------------------------------------------------------------------
# define a Supervisor
supervisor = Supervisor()

### define basic time step
timestep = int(supervisor.getBasicTimeStep())

### get box/Button postions 
box = supervisor.getFromDef("Box")
box1 = supervisor.getFromDef("Box1")
box2 = supervisor.getFromDef("Box2")
box3 = supervisor.getFromDef("Box3")
box4 = supervisor.getFromDef("Box4")
box5 = supervisor.getFromDef("Box5")
box6 = supervisor.getFromDef("Box6")
box7 = supervisor.getFromDef("Box7")
box8 = supervisor.getFromDef("Box8")

boxes = [box1,box2,box3,box4,box5,box6,box7,box8]
boxpos = [-0.2,0.5,0.1]

# Set Position of Boxes 
for i, box in enumerate(boxes):              
    tf = box.getField('translation') # translation field
    tf.setSFVec3f(boxpos)
    boxpos = (np.add(np.array(boxpos), np.array([0.15,0,0]))) # choose next location for the box
    boxpos = boxpos.tolist()
    if (i == 3):
        boxpos = [-0.2, 0.60,0.1]
        
stoptime = 0 
i = 0 
while supervisor.step(timestep) != -1:
    # get Current box
    boxnumber = i% 8
    Ori_Box = boxes[boxnumber].getOrientation()
    Pos_Box = boxes[boxnumber].getPosition()

    # move to the box
    ikAngelsD = GetIK.getAngles(Pos_Box)
    #ikAngelsD = GetIK.getAnglesOptimised(Pos_Box)
    
    GetIK.moveMotors(ikAngelsD)
    sensors = GetIK.motorSensorInfos(timestep)
    #print("Sensors,",sensors)
    robotTipMatrix = GetIK.getTipPose(sensors)
    
    # calculate distance between robot tip and goal 
    Pos_tip = robotTipMatrix[:3,3] # get position from Pose 
    distance_Robot_Goal = np.linalg.norm(np.array(Pos_Box)-np.array(Pos_tip))
 
    # if time >= 3sec: loop and change to next box     
    if (distance_Robot_Goal < 0.01 ):
        if (stoptime == 0):
            goal_reached = True  # first time reached goal 

        if (goal_reached):
            starttime = time.time()
            stoptime = 0.01
            goal_reached = False 

        else:     
            stoptime = time.time() - starttime 
            #print("stoptime", stoptime)
        
        if (stoptime >= 3): 
            stoptime = 0
            i += 1 

    