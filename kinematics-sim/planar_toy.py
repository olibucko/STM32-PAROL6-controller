import roboticstoolbox as rtb
from math import pi

# FK for planar arm example. Each RevoluteDH row is a tranform operation (Ti).
arm = rtb.DHRobot([
    rtb.RevoluteDH(d=0, a=1, alpha=0),   # joint 1, link length 1
    rtb.RevoluteDH(d=0, a=1, alpha=0),   # joint 2, link length 1
])

# Perform the FK (T₁·T₂) with the following parameters 
T = arm.fkine([pi/2, 0])   # θ₁=90, θ₂=0
print(T)