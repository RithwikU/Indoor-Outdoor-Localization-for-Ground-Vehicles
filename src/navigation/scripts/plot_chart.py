#plot data from a csv file with coloumns of x and y data

import matplotlib.pyplot as plt
import csv

x = []
y = []

path = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/gtsam_results/result1.csv";

with open(path, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x.append(float(row[0]))
        y.append(float(row[1]))



#Load data from other csv file and plot on same graph
x1 = []
y1 = []

path1 = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/spline_ground_truth.csv";
with open(path1, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x1.append(float(row[0]))
        y1.append(float(row[1]))


#Load data from other csv file and plot on same graph
x2 = []
y2 = []

path2 = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/spline_lidar_inside.csv";

with open(path2, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x2.append(float(row[0]))
        y2.append(float(row[1]))


#Load data from other csv file and plot on same graph
x3 = []
y3 = []

path3 = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/spline_odom_drift.csv";

with open(path3, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x3.append(float(row[0]))
        y3.append(float(row[1]))

path4 = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/spline_gps.csv";


#Load data from other csv file and plot on same graph
x4 = []
y4 = []
with open(path4, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x4.append(float(row[0]))
        y4.append(float(row[1]))

#Plot all these in one graph
plt.plot(x, y, 'r-x', label='gtsam result')
plt.plot(x1, y1, 'b--x', alpha=0.5, label='ground truth')
plt.plot(x2, y2, 'g--x', alpha=0.5, label='lidar')
plt.plot(x3, y3, 'y--x', alpha=0.5, label='odom')
plt.plot(x4, y4, 'm--x', alpha=0.5, label='gps')

plt.legend()
plt.show()