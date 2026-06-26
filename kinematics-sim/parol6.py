import roboticstoolbox as rtb
from math import pi
from viz import show

# dimensions (m) from DH table
a1, a2, a3, a4, a5, a6, a7 = 0.1105, 0.02342, 0.180, 0.0435, 0.17635, 0.0628, 0.04525

# Set up the DH table (transforms T_i in each joints local frame)
# This relates each joint to the next in local terms. FK and IK both use this.
parol6 = rtb.DHRobot([
    rtb.RevoluteDH(d=a1,  a=a2,  alpha=-pi/2),
    rtb.RevoluteDH(d=0,   a=a3,  alpha=pi),
    rtb.RevoluteDH(d=0,   a=-a4, alpha=pi/2),
    rtb.RevoluteDH(d=-a5, a=0,   alpha=-pi/2),
    rtb.RevoluteDH(d=0,   a=0,   alpha=pi/2),
    rtb.RevoluteDH(d=-a6, a=-a7, alpha=pi),
], name="PAROL6")

# Print PAROL6 DH table
print(parol6)

# Define standby position in radians
from math import radians
standby = [radians(d) for d in [0, -90, 180, 0, 0, 180]]

# Run FK and print output (effector position and orientation)
fk = parol6.fkine(standby)
print(fk)

# Plot standby position
show(standby, parol6)

