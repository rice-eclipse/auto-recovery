import numpy as np
import math
import matplotlib as mpl
from ahrs.filters import Madgwick

madgwick = Madgwick()


def quaternion_to_euler(q):

    rotation0 = 2 * (q[3] * q[0] + q[1] * q[2])
    rotation1 = 1 - 2 * (q[0] * q[0] + q[1] * q[1])
    roll_x = math.atan2(rotation0, rotation1)

    rotation2 = 2 * (q[3] * q[1] - q[2] * q[0])
    if rotation2 > 1:
        rotation2 = 1
    if rotation2 < -1:
        rotation2 = -1
    pitch_y = math.asin(rotation2)

    rotation3 = 2 * (q[3] * q[2] + q[0] * q[1])
    rotation4 = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    yaw_z = math.atan2(rotation3, rotation4)

    return (roll_x, pitch_y, yaw_z)  # in radians

vals = madgwick.updateIMU([0.7071, 0.0, 0.7071, 0.0],[-141,-53,-137],[-3306,-3978,15271])
val = quaternion_to_euler(vals)
print(val)