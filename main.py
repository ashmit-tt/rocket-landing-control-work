import numpy as np
import matplotlib.pyplot as plt

from dynamics import rocket_dynamics
from simulate import rk4steps
from controller import PIDController, LQRController

dt = 0.01 
t_final = 15.0
z_target = 0
mass = 1
max_thrust = 2 * 9.81 * mass
state = np.array([100.0, 0.0])

#can be manually tuned to the situation that best fits the control of the rocket
#pid controller setup
pid = PIDController(Kp = 3, Ki = 0.05, Kd = 8.0)

#lqr set up
A = np.array([[0, 1], [0, 0]])
B = np.array([[0], [1/mass]])
Q = np.diag([100, 10]) #state cost
R = np.array([[0.01]]) #control cost
lqr = LQRController(A, B, Q, R)



t_list = []
z_list = []
v_list = []
thrust_list = []

set_LQR = False #wip, not fully implemented yet, so defaulting to PID for now


print(f"Running" + (" PID Controller" if not set_LQR else " LQR Controller"))

for t in np.arange(0, t_final, dt):
    z, v = state

    if z <= 0:
        print(f"Landed at {t: .2f}s")
        break
    
    if set_LQR:
        thrust = lqr.compute(state)
    else:
        thrust = pid.compute(z_target, z, v, dt)
        
    thrust = np.clip(thrust, 0, max_thrust)
    state = rk4steps(rocket_dynamics, state, thrust, dt)
    t_list.append(t)
    z_list.append(z)
    v_list.append(v)
    thrust_list.append(thrust)


plt.figure(figsize=(10, 10))

#subplot for altitude
plt.subplot(3, 1, 1)
plt.plot(t_list, z_list, label="Altitude (m)", color="blue", linewidth=2)
plt.axhline(y=z_target, color='r', linestyle='--', label="Target")
plt.ylabel("Altitude [m]")
plt.title("PID Controller Results w/ 1 kg rocket w/ 2 TWR")
plt.legend()
plt.grid(True)

#subplot for velocity
plt.subplot(3, 1, 2)
plt.plot(t_list, v_list, label="Velocity (m/s)", color="orange", linewidth=2)
plt.axhline(y=0, color='black', linestyle='-', alpha=0.3) # Zero velocity line
plt.ylabel("Velocity [m/s]")
plt.legend()
plt.grid(True)

# subplot for thrust
plt.subplot(3, 1, 3)
plt.step(t_list, thrust_list, label="Thrust (N)", color="green", where='post')
plt.axhline(y=9.81, color='gray', linestyle=':', label="Hover Thrust")
plt.xlabel("Time [s]")
plt.ylabel("Thrust [N]")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

    
