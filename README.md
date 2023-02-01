# Notes on Control Theory

Went down an internet rabbit hole and cleared up my idea of Control Theory a lot. Wanted to write some of it down so here it is!

## The Problem
Most controllers (i.e.) PID, are linear controllers. Linear Controllers are simpler to understand, simpler to implement, and the control theory community has already developed tons of mathematical tools to solve and use them<sup>1</sup>. They theoretically only work well on linear systems, aka systems that follow homogeneity (scaling input scales output) and superposition (adding inputs adds outputs). However, no systems in real life are this way.

<sup>1. i.e. Transfer Functions to model the system (what output it produces for a particular input), Root Locus Plots to analyze the impact of one parameter on a system, Nyquist Plots to determine if a system is stable.</sup>

## Solutions

Much of the time, we will use a linear controller anyway. Sometimes, if needed, we can make some modifications to a linear controller to make it more fit for the job.

In the most simple case, most systems, even if they aren't linear, are close to linear with an "operating region". So, as long as the linear controller keeps the system in this region, the linear controller is relatively adequate.

In many cases, this, however, isn't possible. Take, for example, a robot arm that pivots like such (the red line is the robot arm): 
![image](https://user-images.githubusercontent.com/35478698/216101624-d525733e-6538-45ff-97d6-e04b4114ad5a.png)

Because of gravity, the robot arm will require a very different control mechanism when it is close to vertical vs close to horizontal. One way to get around this is **Gain Scheduling**. I found this to be a great video on the topic: https://www.youtube.com/watch?v=YiUjAV1bhKs (I love it when videos are able to explain it). Essentially, you sample the PID gains (in the case of a PID controller) in multiple operating regions that you know are significantly different; the operating regions can differ in, for example, the angle of the robot arm, or, if the robot arm has other segments or a load, the moment of inertia of the arm. Then, we construct a contrinuous map between the sample points in all the operating regions using interpolation. Finally, depending on position and moment of inertia of the robot arm, we can calculate the most optimal PID gains to control the robot arm with.

My autonomous mobile robot research project in high school, on which I wrote a paper (https://arxiv.org/abs/2110.09707) actually uses a very specific function and type of gain scheduling.

Another method of adapting a linear controller for a non-linear system is to use motion-profiling. One of the greatest limitations of a linear controller like PID is that it's reactive; it does not know anything about the future. This is bad; take self-driving cars as an example; if there is a turn up ahead, and the car needs to slow down, the PID controller would have no idea until it's too late (it's a purely **reactive** controller). Therefore, we can use a technique called Motion Profiling (also something I used in my [research paper](https://arxiv.org/abs/2110.09707)!). The motion profile, of course, must be fed information about the future, in the form of a path that the car will take. The motion profile takes this path, and outputs a function or time-table containing the target postion, velocity, acceleration, and jerk (and this can be extended to as many time-derivatives of position as your computer can handle) at all times. If we graph our motion profiles, they look something like this (these are actual motion profiles that the robot I constructed for my Autonomous Mobile Robot Research Project ran)!

![image](https://user-images.githubusercontent.com/35478698/216105190-fc8275f4-ae16-4073-b732-794f09f965aa.png)![image](https://user-images.githubusercontent.com/35478698/216105244-6c7269a7-96cb-4a1b-b510-255d8aa9ba92.png)![image](https://user-images.githubusercontent.com/35478698/216105265-956f573c-102e-4e42-89bd-60476241c5b8.png)

(these are slightly buggy, unfortunately)

So, back to our example with the autonomous car, if the car is about to round a corner, the motion profile will make the target velocity lower, and the target position of the car less far away. This will make the PID controller slow the car down. 

### A Non-linear Controller
Model Predictive Control (MPC) is a common non-linear controller used in robotics. To explain how it works, I'll once again take the example of a self-driving car.

Firstly, MPC defines a cost function. This takes into account everything that the controller needs to care about, each weighted appropriately. For example, a cost function for 
a self-driving car might look like this ([source](https://www.youtube.com/watch?v=XaD8Lngfkzk):

$$ c_t(x_t, u_t) = e^T_t Q_t e_t + u^T_t R_t u_t $$

where $c_t$ is the cost at time $t$, $e_t$ is the deviation from the reference path $(e_t = x_t - x^(ref)_t)$, and $u_t$ is the acceleration and steering rate (a vector containing  acceleration and angular velocity). $Q$ and $R$ are matrices with constants that determine the weights of each of these factors.
