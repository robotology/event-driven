import matplotlib.pyplot as plt
import numpy as np

fn_acc = '/home/aglover/local/imu/ajg9.acc.mat'
fn_gyr = '/home/aglover/local/imu/ajg9.gyr.mat'

acc_data = np.genfromtxt(fn_acc)
print("==Acceleration Statistics==")
print("Mean Sample Time:", np.mean(np.diff(acc_data[:, 0])), "seconds")
print("Start", acc_data[0, 0], "Stop", acc_data[-1, 0])
print("X: [", np.min(acc_data[:, 1]), np.max(acc_data[:, 1]), "]")
print("Y: [", np.min(acc_data[:, 2]), np.max(acc_data[:, 2]), "]")
print("Z: [", np.min(acc_data[:, 3]), np.max(acc_data[:, 3]), "]")

gyr_data = np.genfromtxt(fn_gyr)
print("==Gyroscope Statistics==")
print("Mean Sample Time:", np.mean(np.diff(gyr_data[:, 0])), "seconds")
print("Start", gyr_data[0, 0], "Stop", gyr_data[-1, 0])
print("X: [", np.min(gyr_data[:, 1]), np.max(gyr_data[:, 1]), "]")
print("Y: [", np.min(gyr_data[:, 2]), np.max(gyr_data[:, 2]), "]")
print("Z: [", np.min(gyr_data[:, 3]), np.max(gyr_data[:, 3]), "]")

plt.figure(1)
plt.plot(acc_data[:, 0], acc_data[:, 1])
plt.plot(acc_data[:, 0], acc_data[:, 2])
plt.plot(acc_data[:, 0], acc_data[:, 3])
plt.title('Accelerometer Data');
plt.draw()

plt.figure(2)
plt.plot(gyr_data[:, 0], gyr_data[:, 1])
plt.plot(gyr_data[:, 0], gyr_data[:, 2])
plt.plot(gyr_data[:, 0], gyr_data[:, 3])
plt.title('Gyroscope Data');
plt.draw()

plt.show()