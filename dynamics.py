import numpy as np

ROCKET_MASS = 1 #in kg
g = 9.81

#aiming for 2.0 TWR for solid control

max_thrust = 2 * ROCKET_MASS * g # N

def rocket_dynamics(state, thrust):
    z, v = state

    thrust = np.clip(thrust, 0, max_thrust)

    dzdt = v  #change of altitude over time
    a = (thrust / ROCKET_MASS) - g #acceleration of rocket
    
    return np.concatenate([np.atleast_1d(dzdt), np.atleast_1d(a)])