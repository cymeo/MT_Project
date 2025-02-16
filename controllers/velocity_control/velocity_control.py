

import numpy as np
import sympy as sp
from ikpy.chain import Chain
from controller import Robot

# ✅ Load UR5e Model from URDF

AL_mask = [False, False, True, True, True, True, True, True, False, False]
ur5e_chain = Chain.from_urdf_file("ur5e.urdf",active_links_mask = AL_mask)
#ur5e_chain = Chain.from_urdf_file("ur5e.urdf")  # Replace with your URDF file

# ✅ Define Webots Robot Controller
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ✅ Get UR5e Joint Motors & Sensors
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

motors = [robot.getDevice(name) for name in joint_names]
for motor in motors:
    motor.setPosition(float('inf'))  # Disable position control
    motor.setVelocity(0)  # Start with zero velocity
    
sensors = [robot.getDevice(name + "_sensor") for name in joint_names]

for sensor in sensors:
    sensor.enable(timestep)

# ✅ Compute the Symbolic Jacobian Once
def get_ur5e_jacobian():
    print('calculate Jacobian')
    q = sp.symbols('q1:7')  # Joint variables

    dh_params = [
        (q[0],  0.1625,   0,        sp.pi/2),
        (q[1],  0,      -0.425,   0),
        (q[2],  0,      -0.3922,  0),
        (q[3],  0.1333,  0,        sp.pi/2),
        (q[4],  0.0997,  0,       -sp.pi/2),
        (q[5],  0.0996,  0,        0)
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
        T = T @ T_i
        transforms.append(T)

    ee_position = T[:3, 3]
    Jp = ee_position.jacobian(q)

    z_vectors = [sp.Matrix([0, 0, 1])]
    for i in range(6):
        z_vectors.append(transforms[i][:3, 2])

    Jr = sp.Matrix.hstack(*[z_vectors[i].cross(ee_position - transforms[i][:3, 3]) for i in range(6)])

    J = sp.Matrix.vstack(Jp, Jr)
    
    return sp.simplify(J)

# ✅ Convert the Symbolic Jacobian to a Fast NumPy Function
J_symbolic = get_ur5e_jacobian()
q_symbols = sp.symbols('q1:7')
J_fast = sp.lambdify(q_symbols, J_symbolic, modules="numpy")  # 🚀 Fast numerical function

# ✅ Function to Compute Forward Kinematics (Using IKPy)
def get_end_effector_position(joint_values):
    joint_values = np.concatenate((np.array([0,0]),joint_values, np.array([0,0])))
    fk_matrix = np.array(ur5e_chain.forward_kinematics(joint_values))
    return fk_matrix[:3, 3]  # Extract XYZ position

# ✅ Control Loop with 1 cm Accuracy
prev_position = None
update_jacobian = True  # Force initial Jacobian computation



while robot.step(timestep) != -1:
    # Read joint values
    joint_values = np.array([sensor.getValue() for sensor in sensors])

    # Compute end-effector position
    ee_position = get_end_effector_position(joint_values)

    # Check if movement exceeds 1 cm (0.01m)
    if prev_position is None or np.linalg.norm(ee_position - prev_position) > 0.01:
        update_jacobian = True
        prev_position = ee_position  # Update previous position

    # Compute Jacobian only if needed
    if update_jacobian:
        J_numeric = J_fast(*joint_values)  # 🚀 Fast evaluation
        update_jacobian = False

    # Define task-space velocity (example: move in X direction)
    v_task = np.array([0.05, 0, 0, 0, 0, 0])  # 5 cm/s in X direction

    # Compute joint velocities using Jacobian inverse
    q_dot = np.linalg.pinv(J_numeric) @ v_task

    # Apply velocity limits (optional, define q_dot_max based on robot specs)
    q_dot_max = np.array([1.5, 1.5, 2.0, 2.0, 2.5, 3.0])  # Max joint velocities
    
    scaling_factor = min(1, np.max(np.abs(q_dot / q_dot_max)))
    q_dot = q_dot * scaling_factor  # Ensure no joint exceeds max velocity
    print('q_dot: ', q_dot)
    # Send velocity commands to Webots motors
    for i, motor in enumerate(motors):
        motor.setVelocity(q_dot[i])

    print(f"EE Position: {ee_position}, Jacobian Updated: {update_jacobian}")
