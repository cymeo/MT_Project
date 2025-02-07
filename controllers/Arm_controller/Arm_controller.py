"""Arm_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor 

# create the Robot instance.
robot = Robot()
supervisor = Supervisor()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
arm = supervisor.getFromDef("Arm")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    pass

# Enter here exit cleanup code.
