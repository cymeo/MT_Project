"""velocity_control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from ikpy.chain import Chain
import numpy as np
from scipy.spatial.transform import Rotation as R

import sympy as sp 
import time 
# create the Robot instance.

supervisor = Supervisor()
# Load URDF of UR5e

#AL_mask = [False, False, True, True, True, True, True, True, False, False]
#robot_chain = Chain.from_urdf_file("ur5e.urdf",active_links_mask = AL_mask)
# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())


def get_Jacobian(joint_angles): 

    q = sp.symbols('q1:7')
    # UR5e DH Parameters: (theta, d, a, alpha)
    dh_params = [
        (q[0], 0.1625, 0, sp.pi/2), 
        (q[1], 0, -0.425, 0),
        (q[2], 0,-0.3922, 0),
        (q[3], 0.1333, 0, sp.pi/2)
        (q[4], 0.0997, 0, -sp.pi/2)
        (q[5], 0.996, 0, 0 ) 
    ]
    T = sp.eye(4)
    transforms = []
    
    for i, (theta, d, a, alpha) in enumerate(dh_params):
        T_i = sp.Matrix([
            [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha),  sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
            [sp.sin(theta),  sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
            [0,             sp.sin(alpha),                  sp.cos(alpha),                 d],
            [0,             0,                              0,                             1]
        ])
        T = T @ T_i  # Multiply transformations
        transforms.append(T)
     
    ee_position = T[:3,3]
    rotation_matrix = T[:3,:3]
    Jp = ee_position.jacobian(q)
    z_vectors = [sp.Matrix([0,0,1])]   
    for i in range (6):
        z_vectors.append(transforms[i][:3,2])
    
    Jr = sp.Matrix.hstack(*[z_vectors[i].cross(ee_position - transforms[i][:3,3]) for i in range (6)])
    J = sp.Matrix.vstack(Jp,Jr)   
    return J.simplify()

J_UR5e = get_Jacobian()
sp.pprint("jacobian",J_UR5e)

#get Robot 
robot_node = supervisor.getFromDef('Robot') 
box = supervisor.getFromDef('Box')
arm = supervisor.getFromDef('Arm')
table = supervisor.getFromDef('Table')


reset_pose = np.array([0,-np.pi/2, np.pi/2, -np.pi/2,-np.pi/2,0])

##get Motors 
# motors = []
# motors.append(supervisor.getDevice('shoulder_pan_joint'))
# motors.append(supervisor.getDevice('shoulder_lift_joint'))
# motors.append(supervisor.getDevice('elbow_joint'))
# motors.append(supervisor.getDevice('wrist_1_joint'))
# motors.append(supervisor.getDevice('wrist_2_joint'))
# motors.append(supervisor.getDevice('wrist_3_joint'))
# Get Webots motors and sensors
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

motors = [supervisor.getDevice(name) for name in joint_names]
sensors = [supervisor.getDevice(f"{name}_sensor") for name in joint_names]

# Enable position sensors
for sensor in sensors:
    sensor.enable(timestep)

# Get the end-effector frame ID
end_effector_name = "flange"  # Make sure it matches the URDF



def compute_jacobian(joint_angles):
    """ Compute the Jacobian matrix for UR5e """
    return J  

# Main loop:
# - perform simulation steps until Webots is stopping the controller
motor_vel = 0.01
while supervisor.step(timestep) != -1:
    
    joint_angles = np.array([sensor.getValue() for sensor in sensors])
    # Compute Jacobian
    J = get_Jacobian(joint_angles)
    print("Jacobian",J)
    
    motors[1].setVelocity(motor_vel)
    sensor = motors[1].getPositionSensor()
    motor_angle = sensor.getValue()

    if abs(motor_angle) > np.pi: 
        motor_vel= -1*motor_vel 

    
    pass

# Enter here exit cleanup code.


