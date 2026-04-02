### Project Overview:

This project simulates a 1D vertical landing maneuver of a 1 kg rocket. The goal was for the rocket to descent from a varying altitude (currently 100 m) and
touchdown at a varying target height (currently 0 m) at near-zero velocity. I made this simulation in order to compare classical control theory 
applications (PID) to modern control theory applications (LQR) and to be able to work with these models myself.

### Results

| Metric | PID | LQR |
|--------|-----|-----|
| Landing time | 9.01 s | ~15 s |
| Touchdown velocity | ~−1 m/s | ~0 m/s |
| Peak velocity | −26 m/s | −23 m/s |
| Thrust behavior | Overshoot spike | Smooth decay |
| Gravity handling | Feed-forward term required | Encoded in state-space model |
| Tuning method | Manual (P, I, D) | Q/R matrix optimization |


### Technical Specificiations:

Vehicle Mass: 1 kg  
Max Thurst: 19.62 (TWR = 2.0)  
Simulation used: 4th-Order Runge-Kutta (RK4) integration  
Time step: 0.01 s  

### Implementation: 

The rocket was modeled by the following equation:

$$\Large \ddot{z} = \frac{T(t)}{m} - g$$

where T is the function of Thrust in respect to time, and g is 9.81 $$\frac{m}{s^2}$$.  

In order to simulate these dynamics, I implemented RK4 instead of Euler in order to ensure that the transition from gravity and thrust are handled 
with better precision. This is because RK4, unlike Euler, takes 4 samples of slope within a single time step, and takes a weighted average, which allows it
to capture the change of acceleration (jerk) more easily. This is actually best for my LQR controller (more about it below) as it makes sure that the math the controller sees matches the physics the rocket is experiencing.

### PID Controller:

My first idea was to implement the Proportional-Integral-Derviative (PID) Controller. This strategy works by adjusting the thurst exerted by the rocket to the current error.

The equation of error is below: 

$$\Large e(t) = z_{target} - z_{current} $$

where z is the altitude.  

The PID Controller works with three "knobs" that you can turn.

The P, the proportional term, essentially acts like a spring. The further you are from the target, the more the P term pushes you to get there. However, on its own, the P term is dumb. It would make the rocket accelerate to the ground and overshoot it. There's no way to slow down.  

The D, the derivative term, is our dampener. It looks at how fast our error is changing. If our rocket is falling too quickly, the term provides an upward thrust to counteract the velocity. 

The I, the integral term, sums up all of our past errors. If the rocket is hovering at 1 meter due to a slight imbalance between thrust and gravity, the I term would build up "pressure" to eliminate the final steady state offset and bring it down to 0m.  

Before, I didn't feed forward my mg term, which resulted in my rocket essentially crashing too fast. My PID controller had to learn gravity through the error building up, and by the time the I term built up enough "pressure" to stop the fall, the vehicle hit the target altitude with a high velocity, which is what I didn't want. 
Adding 9.81 N to the baseline allowed the controller to focus fully on managing the change of error, which led to a stable descent. Listed below is a graph of my PID controller of (P:3, I: 0.05, D: 8):

![b534ca4b-746d-4ef9-baa7-927e39c03d67](https://github.com/user-attachments/assets/5eaa7571-e6e9-410a-96d4-53c56de89e4a)


The rocket here landed in 9.01 seconds. We can see that the altitude graph has a S shape, our velocity has a V shape, and our thrust actually doesn't start until approximately 2.6 seconds in free fall.
However, we can also see when my altitude hits 0, we still had a velocity speed of approximately -1 m/s, which means I should have had a higher derivative term. 

### LQR Controller:

While the PID Controller was effective, it required a good amount of trial and error to estimate the right values of P,I,D in order to meet the mission requirements.  
The PID controller was also reactive; it only acted based on what happened in the past. To achieve my goal, I researched and implemented a Linear Quadratic Regulator (LQR) controller.  

Unlike the PID controller that required the knobs to account for the mass and gravity, the LQR controller uses a Space-State model.  

To use LQR, we represent the physics of the rocket into a first-order differential equation. 

$$\Large \dot{x} = Ax + Bu$$

The state vector, x, contains the altitude and the velocity. 

The derivative of the state vector (which is calculated), $$\dot{x}$$, contains the change of alitude and the change of velocity.

The A matrix tells us how the internal states of the rocket relate to each other. This is the system matrix.

The B matrix tells us how our control input (which is thrust) affects the change of our states. 

The u, is our control input, which is thrust. 

LQR, however, still requires some tuning, but not in the way as PID works. We use Q and R matricies.

Q is the state cost matrix than penalitalizes error. We can compare the penalized error of altitude to velocity. 

R is the control cost, which tells the controller how expensive or cheap fuel is. 

After determining what I need to accomplish my goal, I was able to use the control library in python to use the control.lqr command to simulate my rocket.

Listed below is the graph of my LQR controller:


![unnamed](https://github.com/user-attachments/assets/720a9051-8250-4f0e-b0f3-7310a034279e)



Although, it may look similar to my PID controller graph, there are some key differences to be made. First off, my rocket never necessarily touched the ground
within the 15 second time interval I gave it. It was adjusting the velocity as close as zero as it can be before touching the ground. We can see this with our velocity graph as it gets closer and closer to zero, instead of the PID controller that does crash. 
However, a similarity would be that both the PID controller and the LQR controller used gravity to aid with reaching the target altitude before using the engine for thrust. If I were to change that to make it a smoother gradual descent, I would increase the altitude cost and decrease the control cost.  







  





