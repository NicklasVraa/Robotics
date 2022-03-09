import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 18})

t = np.genfromtxt('src/husky_controllers/csv/time.csv', delimiter=',')

theta_0_5 = np.genfromtxt('src/husky_controllers/csv/k_alpha0_5.csv', delimiter=',')
theta_3 = np.genfromtxt('src/husky_controllers/csv/k_alpha3.csv', delimiter=',')
theta_6 = np.genfromtxt('src/husky_controllers/csv/k_alpha6.csv', delimiter=',')
theta_10 = np.genfromtxt('src/husky_controllers/csv/k_alpha10.csv', delimiter=',')
plt.plot(t, theta_0_5, color='red', label='k_alpha = 0.5 (Overdamped)')
plt.plot(t, theta_10, color='green', label='k_alpha = 10 (Underdamped)')
plt.plot(t, theta_3, color='blue', linewidth=2, label='k_alpha = 3 (Critically damped)')
plt.plot(t, np.ones_like(t)*1.57,':', color='black', linewidth=2, label='Setpoint: theta = 1.57')
plt.title(f'Angular step response tuning')
plt.gca().set_aspect(2,'box')
plt.xlabel('t')
plt.xlim([0,5])
plt.ylabel('theta')
legend = plt.legend(loc='lower right')
plt.grid()
plt.show()