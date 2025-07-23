from math import sqrt

ROBOT_LENGTH = 400 # [mm]
WHEEL_RADIUS = 35 # [mm]
WHEEL_DIAMETER = WHEEL_RADIUS * 2 # [mm]
WHEEL_BASE = ROBOT_LENGTH - WHEEL_RADIUS * 2 # [mm]

ROBOT_DIAGRAM = r"""
       B
    /  |  \
   M   |   \
  /  \ |    \
A------N------C
(back)      (front)
"""

"""
       B
    /  |  \
   M   |   \
  /  \ |    \
A------N------C
(back)      (front)

A, N, C are wheel locations
B is center axle of RBM
M is the rotation center of the back rocker assembly
WHEEL_BASE is A to C
Triangle BNC is a 45-45-90 triangle
NC == BN, BC = sqrt(2) * NC
AM == MN == BM, where M is the center of the back rocker assembly
AM^2 + MN^2 = AN^2
AM^2 + AM^2 = AN^2
2 * AM^2 = AN^2
AM^2 = 0.5 * AN^2
AM = sqrt(0.5) * AN
"""

BC = sqrt(2) * (WHEEL_BASE / 2) # [mm]
BM = sqrt(0.5) * (WHEEL_BASE / 2) # [mm]

print(
    f'Rover Rocker Bogie Assembly (RBM) parameters:\n'
    f'  Wheel Radius: {WHEEL_RADIUS} mm\n'
    f'  Wheel Base (A to C): {WHEEL_BASE} mm\n'
    f'  Center to Front Wheel (B to N): {BC} mm\n'
    f'  Center to Back Pivot (B to M): {BM} mm\n'
    f'  Rover Diagram:\n{ROBOT_DIAGRAM}'
)
