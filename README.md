
## Udacity SDCND - Term 2: MPC Project ##

### I. The Model

The **classroom model** has been used.

i. State
 - x: position along x axis
 - y: position along y axis
 - psi: the steering angle
 - v: velocity of the car
 - cte: apporximation of the cross-track error along the y axis
 - epsi: error in the steering angle
 
ii. Actuators
 - delta: the applied steering angle
 - a: the applied throttle
 
iii. Update Equations
 - x<sub>t</sub> = x<sub>t-1</sub> \* v<sub>t-1</sub> \* cos(psi<sub>t-1</sub>) \* dt
 -  y<sub>t</sub> = y<sub>t-1</sub> \* v<sub>t-1</sub> \* sin(psi<sub>t-1</sub>) \* dt
 -  psi<sub>t</sub> = psi<sub>t-1</sub> + (v<sub>t-1</sub>/Lf) \* delta<sub>t-1</sub> \* dt
 -  v<sub>t</sub> = v<sub>t-1</sub> + a<sub>t-1</sub> + dt
 -  cte<sub>t</sub> = (f<sub>t-1</sub> - y<sub>t-1</sub>) + (v<sub>t-1</sub> \* sin(epsi<sub>t-1</sub>) \* dt)
 -  epsi<sub>t</sub> = ((psi<sub>t-1</sub> - psides<sub>t-1</sub>) - ((v<sub>t-1</sub>/Lf) \* delta<sub>t-1</sub> \* dt))

**f** is the value of the 3rd degree polynomial representing the reference line at the current value of x.  
**psides** is the desired psi, which is the tangential angle of the derivative of the polynomial at that point.

### II. Timestep Length and Elapsed Duration

The final values chosen are **N=15** and **dt=0.1**.

I experimented with the values suggested in the Q&A video: 10 and 0.1  
The car was able to stick to the path better after increasing t to 0.15 and further stablized at 0.2  
However, once the reference speed was set at values above 35, the time gap of 0.2 was not sufficient. It was reduced to 0.1 and to compensate, the value of N was increased to 15.

### III. Polynomial Fitting

The waypoint co-ordinates received from the simulator are first converted to the car's co-ordinate frame of reference in `Main.cpp::Lines 114 - 122`.  
The converted co-ordinates are fit to a polynomial at `Main.cpp::Lines 124` using the polyfit method.

### IV. Model Predictive Control with Latency

In order to account for the 100 ms latency, the initial state of the car supplied by the simulator is updated using the same model descibed above.   
Here, the **latency** period is used as the time gap **dt**.

Below is the code block from `Main.cpp::Lines 105 - 108`.

```cpp
px = px + v * cos(psi) * latency;
py = py + v * sin(psi) * latency;
psi = psi - (v/Lf) * steer_value * latency;
v = v + throttle_value * latency;
```
