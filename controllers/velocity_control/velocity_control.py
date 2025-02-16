import pinocchio as pin
import numpy as np
from controller import Supervisor

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())


# URDF Path for UR5e
model = pin.buildModelFromUrdf('/home/cecily/MasterThesis_Cy/MT_Project/controllers/velocity_control/ur5e.urdf')
# Create Data for Kinematics
print("Model successfully loaded with", model.njoints, "joints")
for frame in model.frames:
    print(frame.name)

data = model.createData()

# assign joint, motor and sensors
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

motors = [supervisor.getDevice(name) for name in joint_names]
for motor in motors: 
    motor.setPosition(float('inf'))

# assigne and enable motor sensors sensors= []
def get_joint_angles(): 
    sensors= []
    for n, motor in enumerate(motors):
        sensor = motor.getPositionSensor()
        sensor.enable(timestep)
        sensors.append(sensor.getValue())
    return sensors

# End-effector name (matches last link in URDF)
end_effector_name = "tool0"
end_effector_id = model.getFrameId(end_effector_name)

# Desired End-Effector Velocity (Linear + Angular)
xdot = np.array([0.1, 0.0, 0.0, 0, 0, 0])  # Move along X-axis

while True: 
    supervisor.step(timestep)
    # Get Current Joint Positions
    q = np.array(get_joint_angles())
    qdot = np.zeros(len(q))  # Assume zero velocity initially

    # Compute Forward Kinematics
    pin.forwardKinematics(model, data, q, qdot)
    pin.updateFramePlacements(model, data)
    
    ee_id = model.getFrameId("flange") 
    ee_pose = data.oMf[ee_id]
    
    print("ee_pos: ", ee_pose.translation)
    
    # Compute Jacobian
    J = pin.computeFrameJacobian(model, data, q, end_effector_id, pin.LOCAL_WORLD_ALIGNED)

    # Compute Joint Velocities using Pseudoinverse
    qdot_command = np.linalg.pinv(J) @ xdot

    # Apply Joint Velocities
    for i, motor in enumerate(motors):
        motor.setVelocity(qdot_command[i])

