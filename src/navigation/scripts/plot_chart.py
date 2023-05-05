#plot data from a csv file with coloumns of x and y data

import matplotlib.pyplot as plt
import csv

# File names
trial = "2";
filtered_file = "gtsam_filtered_" + trial;
smoothed_file = "gtsam_smoothed_" + trial;
gt_file = "spline_ground_truth_b";
lidar_file = "spline_lidar_mixed_b";
odom_file = "spline_odom_drift_b";
gps_file = "spline_gps_b";


data = "fst"  # o -odom, f - filtered result, s - smoothed result, g - gps, t - ground truth, l - lidar
title = "Comparision of Filtered Result to Smoothed Result"


# Smoothed results data
x_smoothed = []
y_smoothed = []

path_smoothed = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/gtsam_results/" + smoothed_file + ".csv";

with open(path_smoothed, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x_smoothed.append(float(row[0]))
        y_smoothed.append(float(row[1]))



# Filtered results data
x_filtered = []
y_filtered = []
path_filtered = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/gtsam_results/" + filtered_file + ".csv";

with open(path_filtered, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x_filtered.append(float(row[0]))
        y_filtered.append(float(row[1]))



# ground truth results data
x_gt = []
y_gt = []

path_gt = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/" + gt_file + ".csv";
with open(path_gt, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x_gt.append(float(row[0]))
        y_gt.append(float(row[1]))



# Lidar Data
x_lidar = []
y_lidar = []

path_lidar = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/" + lidar_file + ".csv";

with open(path_lidar, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x_lidar.append(float(row[0]))
        y_lidar.append(float(row[1]))


# odometry data
x_odom = []
y_odom = []

path_odom = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/" + odom_file + ".csv";
with open(path_odom, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x_odom.append(float(row[0]))
        y_odom.append(float(row[1]))



# gps data
path_gps = "/home/aadith/Desktop/ESE-650/ESE650-Final-Project/src/navigation/waypoints/" + gps_file + ".csv";
x_gps = []
y_gps = []
count = 0;
with open(path_gps, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        count = count + 1;
        if count > 13 and count < 27:
            x_gps.append(float(row[0]))
            y_gps.append(float(row[1]))




#Plot all these in one graph

#if data contains the letter "s" then it is smoothed data
if "t" in data:
    plt.plot(x_gt, y_gt, 'k-', linewidth=2, label='ground truth')

if "s" in data:
    plt.plot(x_smoothed, y_smoothed, 'g--', alpha=1, label='smoothed estimate')

if "f" in data:
    plt.plot(x_filtered, y_filtered, 'r--',alpha=1, label='filtered estimate')

if "l" in data:
    plt.plot(x_lidar, y_lidar, '--', color=(1,0.75,0), alpha=1, label='lidar localization')

if "o" in data:
    plt.plot(x_odom, y_odom, 'c--', alpha=1, label='odometery from robot')

if "g" in data:
    plt.plot(x_gps, y_gps, 'm--', alpha=1, label='gps localization')





plt.xlabel('X location (m)')
plt.ylabel('Y location (m)')
plt.title(title);

plt.legend()
plt.show()