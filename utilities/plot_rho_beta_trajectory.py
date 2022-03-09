import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 18})

t = np.genfromtxt('src/husky_controllers/csv/time.csv', delimiter=',')

x_rho_0_3_beta_0 = np.genfromtxt('src/husky_controllers/csv/x_rho_0_3_beta_0.csv', delimiter=',')
y_rho_0_3_beta_0 = np.genfromtxt('src/husky_controllers/csv/y_rho_0_3_beta_0.csv', delimiter=',')

x_rho_0_3_beta_n0_2 = np.genfromtxt('src/husky_controllers/csv/x_rho_0_3_beta_n0_2.csv', delimiter=',')
y_rho_0_3_beta_n0_2 = np.genfromtxt('src/husky_controllers/csv/y_rho_0_3_beta_n0_2.csv', delimiter=',')

x_rho_1_5_beta_n0_3 = np.genfromtxt('src/husky_controllers/csv/x_rho_1_5_beta_n0_3.csv', delimiter=',')
y_rho_1_5_beta_n0_3 = np.genfromtxt('src/husky_controllers/csv/y_rho_1_5_beta_n0_3.csv', delimiter=',')

x_rho_1_5_beta_n0_15 = np.genfromtxt('src/husky_controllers/csv/x_rho_1_5_beta_n0_15.csv', delimiter=',')
y_rho_1_5_beta_n0_15 = np.genfromtxt('src/husky_controllers/csv/y_rho_1_5_beta_n0_15.csv', delimiter=',')

x_rho_1_5_beta_n0 = np.genfromtxt('src/husky_controllers/csv/x_rho_1_5_beta_n0.csv', delimiter=',')
y_rho_1_5_beta_n0 = np.genfromtxt('src/husky_controllers/csv/y_rho_1_5_beta_n0.csv', delimiter=',')

x_rho_1_beta_0 = np.genfromtxt('src/husky_controllers/csv/x_rho_1_beta_0.csv', delimiter=',')
y_rho_1_beta_0 = np.genfromtxt('src/husky_controllers/csv/y_rho_1_beta_0.csv', delimiter=',')

x_rho_1_beta_n0_2 = np.genfromtxt('src/husky_controllers/csv/x_rho_1_beta_n0_2.csv', delimiter=',')
y_rho_1_beta_n0_2 = np.genfromtxt('src/husky_controllers/csv/y_rho_1_beta_n0_2.csv', delimiter=',')

x_rho_1_beta_n0_5 = np.genfromtxt('src/husky_controllers/csv/x_rho_1_beta_n0_5.csv', delimiter=',')
y_rho_1_beta_n0_5 = np.genfromtxt('src/husky_controllers/csv/y_rho_1_beta_n0_5.csv', delimiter=',')

waypoints = [[1, 1],
             [0, 2]]

thr = 0.15

theta = np.linspace(0,2*np.pi,200)
plt.plot(np.ones(200)*waypoints[0][0] + thr*np.cos(theta), np.ones(200)*waypoints[0][1] + thr*np.sin(theta), ':', linewidth=2, color='black', label='Goal threshold')
plt.plot(np.ones(200)*waypoints[1][0] + thr*np.cos(theta), np.ones(200)*waypoints[1][1] + thr*np.sin(theta), ':', linewidth=2, color='black')

plt.plot(waypoints[0][0] ,waypoints[0][1] , 'o', mfc='none', color='black', label='Desired Waypoint 1')
plt.plot(waypoints[1][0] ,waypoints[1][1] , 'o', color='black', label='Desired Waypoint 2')

plt.plot(x_rho_0_3_beta_0, y_rho_0_3_beta_0, color='red', label='k_rho = 0.3, k_beta = 0')
plt.plot(x_rho_0_3_beta_n0_2, y_rho_0_3_beta_n0_2, color='blue', label='k_rho = 0.3, k_beta = -0.2')
plt.plot(x_rho_1_5_beta_n0_3, y_rho_1_5_beta_n0_3, color='purple', label='k_rho = 1.5, k_beta = -0.3')
plt.plot(x_rho_1_5_beta_n0_15, y_rho_1_5_beta_n0_15, color='yellow', label='k_rho = 1.5, k_beta = -0.15')
plt.plot(x_rho_1_5_beta_n0, y_rho_1_5_beta_n0, color='green', label='k_rho = 1.5, k_beta = 0')
plt.plot(x_rho_1_beta_0, y_rho_1_beta_0, color='cyan', label='k_rho = 1, k_beta = 0')
plt.plot(x_rho_1_beta_n0_2, y_rho_1_beta_n0_2, color='pink', label='k_rho = 1, k_beta = -0.2')
plt.plot(x_rho_1_beta_n0_5, y_rho_1_beta_n0_5, color='grey', label='k_rho = 1, k_beta = -0.5')

plt.title(f'k_rho and k_beta tuning for trajectory with pi/2 turn')
plt.gca().set_aspect('equal', 'box')
plt.xlim([-0.5,3])
plt.xlabel('x')
plt.ylabel('y')
legend = plt.legend(loc='lower right')
plt.grid()
plt.show()