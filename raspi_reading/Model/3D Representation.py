#Problems: Slow, position calculations are shit, need to connect with rocket directly (change in collection of data).
import numpy as np
import matplotlib.pyplot as plt

vel = [0,0,0]
pos = [0,0,0]
blue = 1
red = 0

fig = plt.figure()
ax = plt.axes(projection ='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
l = 0

file = open(r"drop-test-data.txt", "r")

for each in file:
    line = each.split("> ")[1]
    sep = line.split(", ")[:9]
    acc = [int(sep[0])/10000, int(sep[1])/10000, int(sep[2])/10000]
    roVel = sep[3:6]
    magFi = sep[6:]
    vel[0] = vel[0] + 0.4*acc[0]
    vel[1] = vel[1] + 0.4*acc[1]
    vel[2] = vel[2] + 0.4*acc[2]
    pos[0] = pos[0] + 0.4*vel[0]
    pos[1] = pos[1] + 0.4*vel[1]
    pos[2] = pos[2] + 0.4*vel[2]

    if l % 3 == 0:
        ax.scatter3D(pos[0], pos[1], -pos[2], color=(red,0,blue))

        if red < 0.95:
            red = red + 0.02
            blue = blue - 0.02
        plt.draw()
        plt.pause(0.4)
    l = l + 1

plt.show()
file.close()