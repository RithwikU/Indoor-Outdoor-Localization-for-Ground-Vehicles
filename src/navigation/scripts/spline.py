from scipy.interpolate import InterpolatedUnivariateSpline
import numpy as np
import matplotlib.pyplot as plt
import csv

# Read in the waypoints
x = []
y = []

path = "../waypoints/"

with open(path+'waypoints_big.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x.append(float(row[0]))
        y.append(float(row[1]))

axes = np.linspace(1, len(x), len(x))

# Interpolate the waypoints
x_spline = InterpolatedUnivariateSpline(axes, x)
y_spline = InterpolatedUnivariateSpline(axes, y)

# Sample the interpolated waypoints
number_of_samples = 50
axes_sampled = np.linspace(1, len(x), number_of_samples)
noise_sd = 0.0 # Standard deviation of the noise added to the spline

drift_x = np.zeros(number_of_samples)
drift_y = np.zeros(number_of_samples)
# drift_x[25:35] = 1.0 # Add a drift to the spline for some samples in the x direction

# drift_y[15:25] = 2.0 # Add a drift to the spline for some samples in the y direction
# drift_y[5:15] = -1.0 # Add a drift to the spline for some samples in the y direction
# drift_y[40:] = -1.0 # Add a drift to the spline for some samples in the y direction

x_sampled = x_spline(axes_sampled) + np.random.normal(0, noise_sd, number_of_samples) + drift_x
y_sampled = y_spline(axes_sampled) + np.random.normal(0, noise_sd, number_of_samples) + drift_y

# Save the interpolated waypoints
# with open(path+'dense_straight_only.csv', 'w') as csvfile:
with open(path+'spline_ground_truth_b.csv', 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')

    for i in range(len(x_sampled)):
        writer.writerow([x_sampled[i], y_sampled[i]])

# Plot the interpolated waypoints
plt.plot(x_sampled, y_sampled, 'r-x', label='spline')
plt.plot(x, y, 'b--x', alpha=0.5, label='waypoints')
plt.legend()
plt.show()



