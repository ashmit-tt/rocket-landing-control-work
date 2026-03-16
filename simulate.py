#4th order rk, error dt^5/
def rk4steps(f, state, u, dt):
    k1 = f(state, u)
    k2 = f(state + 0.5*dt*k1, u)
    k3 = f(state + 0.5*dt*k2, u)
    k4 = f(state + dt * k3, u)

    return state + (dt/6) * (k1  + 2*k2 + 2*k3 + k4)



