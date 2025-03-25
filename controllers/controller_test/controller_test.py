"""controller_test controller."""
from controller import Robot, Supervisor,Connector
import pinocchio as pin 

import numpy as np

from numpy import pi
from scipy.spatial.transform import Rotation as R

###### let Robot follow a Box ######################################

#-------------------------------------------------------------------
# define a Supervisor
supervisor = Supervisor()


# load pin model for UR5e
robot_model = pin.buildModelFromUrdf('./ur5e.urdf')
frame_names = [frame.name for frame in robot_model.frames]# Print the chain to verify the correct links are included
robot_data = robot_model.createData()



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
    
    # box_position = box0.getPosition()
    # box_rotation = box0.getOrientation() 
    # box_rotationMatrix = np.array(box_rotation).reshape(3, 3)

    # #print("box_rotation: ", box_rotationMatrix)
    # r = R.from_matrix(box_rotationMatrix)
      
   
    reset_pose = np.array([0,-np.pi/2, np.pi/2, -np.pi/2,-np.pi/2,0])
    #reset_pose = np.array([0,0,0,0,0,1])

    
    sensors = np.zeros(6)
    theta_dot = np.zeros(6)
    for n, motor in enumerate(motors):
        motor.setPosition(reset_pose[n])
        sensor = motor.getPositionSensor()
        # sensor.enable(timestep)
        # sensors[n] = sensor.getValue()
        # theta_dot[n] = motor.getVelocity()
    
    # pin.forwardKinematics(robot_model, robot_data, sensors, theta_dot)
    # pin.updateFramePlacements(robot_model, robot_data) 
    # ee_id = robot_model.getFrameId("tool0")  # Get end-effector
    # ee_pose = robot_data.oMf[ee_id].homogeneous
    
    
    #print("sensors: ",sensors)
    # print('sensors:', sensors)

    # print('robot pose', ee_pose)
