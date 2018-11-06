#!/usr/bin/env python
import matplotlib.pyplot as plt
from Relative2AbsolutePose import Relative2AbsolutePose
from Relative2AbsoluteXY import Relative2AbsoluteXY

# if __name__=="__main__":
#     robot_abs = []
#     u = []
#     contents = []
#     with open("measurement.txt", "r") as f:
#         for line in f:
#             robot_abs.append((line.split(",",1)[0]))

#             u.append((line.split(", ",1)[1]))
#         print(contents)

#     print(len(contents))
#     print(robot_abs)
#     print(u)

import csv
import numpy as np
robot_abs = []
u = []
robot_world_pos = np.asarray([[0, 0, 0]], dtype=np.float64)
robot_landmark_pos1 = np.asarray([[0]], dtype=np.float64)
robot_landmark_pos2 = np.asarray([[0]], dtype=np.float64)

with open("measurement.txt", "r") as f:
    csvreader = csv.reader(f, delimiter = ',', quoting = csv.QUOTE_MINIMAL)
    for row in csvreader:
        robot_abs = [[float(row[0])], [float(row[1])], [float(row[2])]]
        u = [[float(row[3])], [float(row[4])], [float(row[5])]]
        
        [next_robot_abs, H1, H2] = Relative2AbsolutePose(robot_abs, u)

        robot_world_pos = np.append(robot_world_pos, next_robot_abs.T, 0)

with open("trajectory_and_points.txt", "r") as f1:
    csvreader = csv.reader(f1, delimiter = ',', quoting = csv.QUOTE_MINIMAL)
    for row in csvreader:
	landmark_meas_xy = [float(row[0]), float(row[1])]
        [[landmark_abs1,landmark_abs2], HL1, HL2] = Relative2AbsoluteXY(robot_abs, landmark_meas_xy)

        robot_landmark_pos1 = np.append(robot_landmark_pos1, landmark_abs1)
        robot_landmark_pos2 = np.append(robot_landmark_pos2, landmark_abs2)
     
plt.plot(robot_world_pos[:,0], robot_world_pos[:,1],'r-',robot_landmark_pos1, robot_landmark_pos2, 'bo')
plt.show()
