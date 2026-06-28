import roboticstoolbox as rtb
from math import pi, radians
from viz import show
import numpy as np
import swift
from spatialgeometry import Sphere
from spatialmath import SE3
import copy

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

# Pick known joint configuration
q_true = [np.deg2rad(d) for d in [20,-60, 140, 10, 30, 90]]

# FK - Determine pose in this joint config
T_target = parol6.fkine(q_true)
print("Target Pose:\n", T_target)

# Run IK using target pose
#sol = parol6.ikine_LM(T_target)
sol = parol6.ikine_LM(T_target, q0=q_true) # Optional config seeding
print("\nSolver Result:", sol)

# Result?
print("\nSuccess?", sol.success)
print("Solved angles (deg):", np.round(np.rad2deg(sol.q), 1))
print("True angles (deg):  ", np.round(np.rad2deg(q_true), 1))
print("Position Error (m):", np.round(parol6.fkine(sol.q).t - T_target.t, 3))

show(q_true, parol6)
show(sol.q, parol6)

