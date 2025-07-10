---
layout: default
---

<script type="text/x-mathjax-config">
  MathJax.Hub.Config({
    extensions: ["tex2jax.js"],
    jax: ["input/TeX", "output/HTML-CSS"],
    tex2jax: {
      inlineMath: [ ['$','$$'], ["\\(","\\)"] ],
      displayMath: [ ['$$','$$'], ["\\[","\\]"] ],
      processEscapes: true
    },
    "HTML-CSS": { fonts: ["TeX"] }
  });
</script>
<script type="text/javascript" async
  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.7/MathJax.js?config=TeX-MML-AM_CHTML">
</script>

## Back-stepping Prescribed-Time Sliding Mode Control of a Quadrotor UAV

This repository contains Python code that implements a hybrid control strategy for a quadrotor UAV, combining Back-stepping Prescribed-Time Control (PTC) and Sliding Mode Control (SMC). The aim is to achieve precise trajectory tracking within a user-defined preset time, while also demonstrating robustness against variations in initial conditions. This implementation is based on the research presented in the paper "Back-stepping Prescribed-Time Sliding Mode Control of a Quadrotor UAV" by Roozbeh Badiei, Mohammad Bagher Sajjadi, Amir Hossein Davaie-Markazi, and Moosa Ayati. The paper was presented at the 2024 12th RSI International Conference on Robotics and Mechatronics (ICROM) with DOI: 10.1109/ICROM64545.2024.10903605.

### Controller Design

The control strategy presented in the paper and implemented in this code integrates two main control techniques:

1.  **Back-stepping Prescribed-Time Control (PTC)**: This method ensures that the trajectory tracking errors converge to zero within a user-defined finite time, regardless of the initial conditions. This is achieved by designing a control law that drives the system states towards the desired trajectory within a predetermined time period $t_p$.

    The core idea of prescribed-time stability is outlined in Definition 2 of the paper. It states that a system is prescribed-time stable if it is finite-time stable and there exists a predetermined time $t_p' = t_p + t_0$ (where $t_p$ is the user-defined target time and $t_0$ is the initial time) that does not depend on the system's initial conditions, within which the system's state converges.

    The Back-stepping approach formulates the control by defining error variables for each state. The error variables are defined as:
    $$e_{2i-1}=x_{2i-1}(t)-x_{d(2i-1)}(t)$$ $$e_{2i}=x_{2i}(t)-\dot{x}_{d(2i-1)}(t)-\alpha_{i}(t)$$
    where $i$ is an index from 1 to 6, and $\alpha_i(t)$ are virtual signals.
    The virtual control signals $\alpha_i(t)$ are defined as mentioned in equation (31) in the paper:
    $$\alpha_{i}=-\gamma_{2i-1}\frac{e_{2i-1}}{t_{p}+t_{0}-t}$$
    where $\gamma_{2i-1}$ are proportional gain coefficients, and $e_{2i-1}$ are the tracking errors.

    For $t < t_p$, the control laws for rotational and translational movements are given by equations (32) to (37) in the paper:
    $$U_{2}=\frac{1}{b_{1}}[-e_{1}(t)-\gamma_{2}(\frac{e_{2}(t)}{t_{p}+t_{0}-t})+\dot{\alpha}_{1}(t)+\dot{x}_{1d}-f_{1}(x,t)]$$ $$U_{3}=\frac{1}{b_{2}}[-e_{3}(t)-\gamma_{4}(\frac{e_{4}(t)}{t_{p}+t_{0}-t})+\dot{\alpha}_{2}(t)+\dot{x}_{3d}-f_{2}(x,t)]$$ $$U_{4}=\frac{1}{b_{3}}[-e_{5}(t)-\gamma_{6}(\frac{e_{6}(t)}{t_{p}+t_{0}-t})+\dot{\alpha}_{3}(t)+\dot{x}_{5d}-f_{3}(x,t)]$$ $$v_{x}=-e_{7}(t)-\gamma_{8}(\frac{e_{8}(t)}{t_{p}+t_{0}-t})+\dot{\alpha}_{4}(t)+\dot{x}_{7d}-f_{4}(x,t)$$ $$v_{y}=-e_{9}(t)-\gamma_{10}(\frac{e_{10}(t)}{t_{p}+t_{0}-t})+\dot{\alpha}_{5}(t)+\dot{x}_{9d}-f_{5}(x,t)$$ $$v_{z}=-e_{11}(t)-\gamma_{12}(\frac{e_{12}(t)}{t_{p}+t_{0}-t})+\dot{\alpha}_{6}(t)+\dot{x}_{11d}-f_{6}(x,t)$$
    These laws are derived using a Lyapunov function candidate $V(x) = \sum_{i=1}^{12}\frac{1}{2}e_{i}^{2}$. The derivative of the Lyapunov function is shown to satisfy:
    $$\dot{V}=\sum_{i=1}^{12}-\gamma_{i}\frac{e_{i}^{2}}{t_{p}+t_{0}-t}\le\gamma\frac{V}{t_{p}+t_{0}-t}$$
    where $\gamma=min_{i\in\{1,2,...,12\}}\gamma_{i}$. This guarantees prescribed-time stability as per Lemma 1 in the paper.

2.  **Sliding Mode Control (SMC)**: After the prescribed time $t_p$, the control switches to a Sliding Mode Control (SMC) to maintain the tracking errors at zero and ensure the stability and robustness of the closed-loop system. SMC is a robust nonlinear control technique that forces the system's trajectories onto a predefined stable "sliding surface" in the state space.

    The sliding surface $s_i(x)$ is defined as mentioned in equation (26) in the paper:
    $$s_{i}(x)=\dot{E}_{i}+\lambda_{i}E_{i}$$
    where $E_i = x_i - x_{d_i}$ is the tracking error, $\dot{E}_i$ is the derivative of the error, and $\lambda_i$ are positive constants. The control objective of SMC is to bring the error $E_i$ to zero. The switching control law is given by equation (28) in the paper:
    $$u_{sw}=K_{i}sign(s_{i})$$
    Where $K_i$ are positive switching constants. For $t \ge t_p$, the control laws for rotational and translational motion are defined by equations (41) to (46) in the paper:
    $$U_{2}=-\frac{1}{b_{1}}(\lambda_{1}(x_{2}-x_{2d})-\dot{x}_{2}+f_{1}(x,t)+K_{1}sign(s_{1}))$$ $$U_{3}=-\frac{1}{b_{2}}(\lambda_{3}(x_{4}-x_{4d})-\dot{x}_{4}+f_{2}(x,t)+K_{3}sign(s_{3}))$$ $$U_{4}=-\frac{1}{b_{3}}(\lambda_{5}(x_{6}-x_{6d})-\dot{x}_{6}+f_{1}(x,t)+K_{5}sign(s_{5}))$$ $$v_{x}=-(\lambda_{7}(x_{8}-x_{8d})-\dot{x}_{8}+f_{4}+K_{7}sign(s_{7}))$$ $$v_{y}=-(\lambda_{9}(x_{10}-x_{10d})-\dot{x}_{10}+f_{5}+K_{9}sign(s_{9}))$$ $$v_{z}=-(\lambda_{11}(x_{12}-x_{12d})-\dot{x}_{12}+f_{6}+K_{11}sign(s_{11}))$$
    The stability of the SMC part is proven using a Lyapunov function $V_{smc}=\sum_{i=1}^{6}\frac{1}{2}s_{2i-1}^{2}$. The derivative of this Lyapunov function, as shown in equation (48) in the paper:
    $$\dot{V}_{sme}(x)=\sum_{i=1}^{6}s_{2i-1}\dot{s}_{2i-1}= \sum_{i=1}^{6}-K_{2i-1}|s_{2i-1}|\le0$$
    guarantees the stability of the error dynamics.

The combined control scheme is illustrated in Figure 1 of the paper, showing how the desired angles and total thrust are obtained and applied to the quadrotor dynamics.

The quadrotor's dynamic model, as mentioned in equation (1) in the paper, is represented by a set of equations incorporating translational and rotational motion. The state vector is $\vec{x}=[\phi,\dot{\phi},\theta,\dot{\theta},\psi,\dot{\psi},x,\dot{x},y,\dot{y},z,\dot{z}]\in\mathbb{R}^{12}$.

### Code Structure

The Python code provided simulates the quadrotor's behavior under this hybrid control strategy.

#### Constants

The code begins by defining physical constants for the quadrotor, such as L (half-length), m (mass), g (gravity), and moments of inertia (Ix, Iy, Iz), as well as drag coefficients (b, d, Kf, Kt). These are crucial for accurate dynamic modeling of the UAV.

#### BackSteppingControl(t, tp, t0, wStar, x, XD, xDoubleDotD, FHAT_ST, gammaCtrl)

This function implements the Back-stepping Prescribed-Time Control logic.

* **Inputs**:
    * t: Current time.
    * tp: Prescribed time for convergence.
    * t0: Initial time.
    * wStar: Angular disturbance from propellers ($\Omega^*$).
    * x: Current state vector of the quadrotor.
    * XD: Desired state vector.
    * xDoubleDotD: Second derivative of the desired state vector.
    * FHAT_ST: Estimated fault/disturbance terms.
    * gammaCtrl: Array of proportional gain coefficients ($\gamma_i$) (not used).

* **Outputs**:
    * u: Calculated control inputs (thrust and torques).
    * e: Tracking errors.

#### PIDController(x, xOld, xd, xdOld, Ts, INTEGRALOLD, Kp, Kd, Ki)

This function, although present in the provided Python code, is explicitly stated in the paper as being used for *comparison* with the proposed control strategy, highlighting the superior performance of the PTC. It's a standard PID implementation for position and attitude control.

* **Inputs**: Current and old state vectors, desired current and old state vectors, sampling time, old integral terms, and PID gains (Kp, Kd, Ki).
* **Outputs**: Control inputs, total thrust, new integral terms, and new error terms.

#### SlidingModeControl(t, tp, t0, wStar, x, XD, xDoubleDotD, FHAT_ST, gammaCtrl)
This function implements the Sliding Mode Control (SMC) logic.
* **Inputs**:
    * t: Current time.
    * tp: Prescribed time for convergence.
    * t0: Initial time.
    * wStar: Angular disturbance from propellers ($\Omega^*$).
    * x: Current state vector of the quadrotor.
    * XD: Desired state vector.
    * xDoubleDotD: Second derivative of the desired state vector.
    * FHAT_ST: Estimated fault/disturbance terms.
    * gammaCtrl: Array of proportional gain coefficients ($\gamma_i$) (not used).

* **Outputs**:
    * u: Calculated control inputs (thrust and torques).
    * e: Tracking errors.

#### plotResults(t, x, Xd, u)

This function is a utility for plotting various simulation results to analyze the quadrotor's performance.

* **Plots**:
    * Quadrotor position states (x, y, z) against desired trajectories. This corresponds to the translational subsystem tracking results mentioned in the paper.
    * Quadrotor orientation states (roll $\phi$, pitch $\theta$, yaw $\psi$) against desired trajectories. This corresponds to the rotational subsystem tracking results.
    * 3D trajectory of the quadrotor compared to the desired 3D path.
    * Tracking errors for all states ($\phi, \dot{\phi}, \theta, \dot{\theta}, \psi, \dot{\psi}, x, \dot{x}, y, \dot{y}, z, \dot{z}$). The paper highlights the reduction in overshoots and tracking errors.
    * Control signals $U_3, U_4, U_5, U_6$ (roll, pitch, yaw torques, and vertical thrust equivalent).

#### Rotor2_Dynamic(t, x, u, FAULT_ANGLES)

This function defines the quadrotor's nonlinear dynamics.

* **Inputs**:
    * t: Current time.
    * x: Current state vector.
    * u: Control inputs (from BackSteppingControl or SlidingModeControl).
    * FAULT_ANGLES: Angles related to fault injection (used for disturbance modeling).
* **Calculations**:
    * Calculates the individual rotor squared angular velocities from the control inputs (uThrust, uPhi, uTheta, uPsi) using a transformation matrix.
    * Computes the angular disturbance wStar ($\Omega^*$).
    * Injects "faults" or disturbances (Disturbance) based on the FAULT_ANGLES.
    * Calculates the second derivatives of all states (linear accelerations and angular accelerations) based on the forces, moments of inertia, drag coefficients, and disturbances. The dynamic model of a UAV is given by equation (1) in the paper:
    $$\dot{x}=(\sin\phi\sin\psi+\cos\phi\sin\theta\cos\psi)\frac{U_{1}}{m}-\frac{k_{1}}{m}\dot{x},$$
    $$\dot{y}=(-\sin\phi\cos\psi+\cos\phi\sin\theta\sin\psi)\frac{U_{1}}{m}-\frac{k_{2}}{m}\dot{y}$$
    $$\dot{z}=-g+(\cos\phi\cos\theta)\frac{U_{1}}{m}-\frac{k_{3}}{m}\dot{z},$$
    $$\ddot{\phi}=(\frac{I_{yy}-I_{zz}}{I_{xx}})\dot{\theta}\dot{\psi}+\frac{J_{TP}}{I_{xx}}\dot{\theta}\Omega^{*}+\frac{U_{2}}{I_{xx}}-\frac{k_{4}l}{I_{xx}}\dot{\phi}.$$
    $$\ddot{\theta}=(\frac{I_{zz}-I_{xx}}{I_{yy}})\dot{\phi}\dot{\psi}-\frac{J_{TP}}{I_{yy}}\dot{\phi}\Omega^{*}+\frac{U_{3}}{I_{yy}}-\frac{k_{5}l}{I_{yy}}\dot{\theta},$$
    $$\ddot{\psi}=(\frac{I_{xx}-I_{yy}}{I_{zz}})\dot{\theta}\dot{\phi}+\frac{U_{4}}{I_{zz}}-\frac{k_{6}l}{I_{zz}}\dot{\psi}$$
* **Outputs**:
    * XDOT: Derivative of the state vector (rates of change for all 12 states).
    * Disturbance: Calculated disturbance forces/torques.
    * wStar: Angular disturbance.
    * W: Individual rotor angular velocities.
    * fPhi, fTheta, fPsi: Terms from angular dynamics.

#### setDesiredTrajectory(t_array, CASE, n=6)

This function generates different desired trajectories for the quadrotor to follow. The paper states that the desired reference signals and their derivatives are assumed to be known, bounded, and piecewise continuous. For Case 1, the desired trajectory for $x_d(t)$ and $y_d(t)$ is as mentioned in equation (49) in the paper:
$$x_{d}(t)=\begin{cases}2,&if~t<10,\\ 1,&if~10\le t<30,\\ 2,&if~t\ge30,\end{cases}$$
$$y_{d}(t)=\begin{cases}2,&if~t<20,\\ 1,&if~20\le t<40,\\ 2,&if~t\ge40,\end{cases}$$
$$z_{d}(t)=2,$$
$$\psi_{d}(t)=0.5$$

* **Inputs**:
    * t_array: Array of time points.
    * CASE: Integer specifying the trajectory type (1: Linear, 2: Elliptical, 3: BiLinear).
    * n: Number of degrees of freedom.
* **Outputs**:
    * XD: Desired state vector (position, velocity, angles, angular velocities).
    * XDoubleDotD: Second derivative of the desired state vector.

#### SlidingModeControl(x, XD, xDoubleDotd, K, a, fPhi, fTheta, fPsi)

This function implements the Sliding Mode Control (SMC) logic, which is activated after the prescribed time $t_p$.

* **Inputs**:
    * x: Current state vector.
    * XD: Desired state vector.
    * xDoubleDotd: Second derivative of the desired state vector.
    * K: Positive switching constants for SMC.
    * a: Coefficients for the sliding surface (lambda in the paper).
    * fPhi, fTheta, fPsi: Terms from angular dynamics.
* **Calculations**:
    * Computes the error e.
    * Calculates the sliding surface S (equation (26) in the paper).
    * Derives the control inputs (ux, uy, uz, uPhi, uTheta, uPsi) using the SMC law, incorporating the sliding surface and control gains. The tanh function is used instead of sign to mitigate the chattering phenomenon. These correspond to equations (41) to (46) in the paper.
* **Outputs**:
    * u: Calculated control inputs.
    * e: Tracking errors.

#### stateCalculation(K1RK, xOld, uold, Ts, tOld, FAULT_ANGLES)

This function performs a single step of the Runge-Kutta 4th order numerical integration method to update the quadrotor's state.

* **Inputs**:
    * K1RK: First Runge-Kutta coefficient (result of Rotor2_Dynamic).
    * xOld: Previous state vector.
    * uold: Control input at the previous time step.
    * Ts: Sampling time.
    * tOld: Previous time.
    * FAULT_ANGLES: Fault injection angles.
* **Calculations**: Computes K2, K3, K4 coefficients by evaluating Rotor2_Dynamic at intermediate points and then calculates the new state Q using the weighted average of these coefficients.
* **Outputs**:
    * Q: New state vector.

#### main()

This is the main simulation loop that orchestrates the entire process.

* **Initialization**: Sets up simulation parameters (time step Ts, max time tMax, prescribed time tp), initial conditions x0, and initializes arrays for states x, errors e, control signals u, and fault estimations.
* **Parameter Setup**: Defines gammaCtrl for Back-stepping parameters (e.g., $\gamma_1=\gamma_2=\gamma_3=\gamma_4=\gamma_5=\gamma_6=1.2$ as used in simulations) and K, a for SMC parameters ($\lambda_i=20$ and $K_i=50$ for $i=1,3,5,7,9,11$ as used in simulations).
* **Desired Trajectory**: Calls setDesiredTrajectory to generate the reference path for the quadrotor based on the chosen CASE.
* **Simulation Loop**: Iterates through time, performing the following steps at each time point:
    * Calculates the quadrotor dynamics and actual disturbances using Rotor2_Dynamic.
    * Updates the quadrotor's state using stateCalculation (Runge-Kutta 4th order).
    * Dynamically updates desired roll (phid_val) and pitch (thetad_val) angles based on the current control inputs (virtual control design). This is a crucial step in the decoupling of translational and rotational dynamics.
    * Applies either BackSteppingControl (for $t < t_p$) or SlidingModeControl (for $t \ge t_p$) to calculate the control signals u. This implements the hybrid control strategy.
* **Results**: After the simulation, it calls plotResults to visualize the quadrotor's performance.
