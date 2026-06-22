import roboticstoolbox as rtb
robot = rtb.models.DH.Puma560() # built-in 6 axis arm models
print(robot) # prints DH table of model
T = robot.fkine(robot.qz) # FK for zero configuration
print(T) # Effector position as 4x4 matrix