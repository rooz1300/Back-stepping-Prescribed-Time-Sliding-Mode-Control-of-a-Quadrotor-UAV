

# Advanced Quadrotor Control Simulation: Back-stepping & Sliding Mode Control

This repository contains Python code that implements a hybrid control strategy for a quadrotor UAV, combining Back-stepping Prescribed-Time Control (PTC) and Sliding Mode Control (SMC). The aim is to achieve precise trajectory tracking within a user-defined preset time, while also demonstrating robustness against variations in initial conditions. This implementation is based on the research presented in the paper "Back-stepping Prescribed-Time Sliding Mode Control of a Quadrotor UAV" by Roozbeh Badiei, Mohammad Bagher Sajjadi, Amir Hossein Davaie-Markazi, and Moosa Ayati. The paper was presented at the 2024 12th RSI International Conference on Robotics and Mechatronics (ICROM) with DOI: 10.1109/ICROM64545.2024.10903605 

The core objective is to force the UAV's tracking errors to converge to zero within a user-defined time, regardless of the initial conditions, and to maintain stability even when subjected to faults and disturbances.

-----

## 🚀 Key notes

  * **Hybrid Control Strategy**: Implements a two-phase control system that switches from PTC to SMC for optimal performance.
  * **Prescribed-Time Convergence**: Utilises a Back-stepping controller to guarantee that all tracking errors converge to zero within a predefined time (`tp`).
  * **Robust Sliding Mode Control**: Employs an SMC to ensure long-term stability and actively reject disturbances after the initial convergence period.
  * **Detailed Nonlinear Dynamics**: The simulation is built on a full nonlinear dynamic model of the quadrotor, including aerodynamic effects and rotor dynamics.
  * **Fault Injection**: Includes a mechanism to simulate actuator faults, allowing for rigorous testing of the controller's robustness.
  * **Multiple Trajectory Options**: Comes with several predefined, complex trajectories to test the controller's tracking capabilities.
  * **Comprehensive Visualization**: Automatically generates detailed plots for trajectory tracking, state errors, and control signals for easy analysis.

-----

## 🔧 Control Strategy Explained

The simulation employs a hybrid strategy that leverages the unique strengths of two different advanced controllers.

### Phase 1: Back-stepping Prescribed-Time Control (PTC)

  * **Active Period**: From the start of the simulation (`t=0`) until the prescribed time (`t < tp`).
  * **Purpose**: The primary goal of the PTC is **guaranteed, rapid error convergence**. It uses time-varying gains that increase as the deadline `tp` approaches, creating a strong control action that forces the quadrotor onto the desired trajectory within the specified time, irrespective of its starting position.

### Phase 2: Sliding Mode Control (SMC)

  * **Active Period**: For all time after the prescribed deadline (`t >= tp`).
  * **Purpose**: Once the errors have been eliminated, the SMC takes over to **robustly maintain stability**. It is exceptionally effective at rejecting external disturbances and compensating for uncertainties in the system model, ensuring the quadrotor remains locked onto the trajectory for the remainder of the simulation. A `tanh` function is used to smooth the control signal and mitigate chattering.

-----

## ▶️ How to Run

1.  **Clone the repository:**

    ```bash
    git clone https://github.com/rooz1300/Back-stepping-Prescribed-Time-Sliding-Mode-Control-of-a-Quadrotor-UAV
    run main.ipynb
    ```

2.  **Install dependencies:**
    Make sure you have Python 3 and pip installed. Then, install the required libraries.

    ```bash
    pip install numpy matplotlib scipy
    ```

3.  **Run the simulation:**
    Execute the main script from your terminal.

    ```bash
    python your_script_name.py
    ```

The script will run the simulation and automatically generate a series of plots showing the results.

-----

## ⚙️ Customization

You can easily modify the simulation parameters in the `main()` function of the script:

  * **`CASE`**: Change the desired trajectory.
      * `1`: Piecewise linear trajectory.
      * `2`: Smooth elliptical trajectory.
      * `3`: Complex bi-linear trajectory.
  * **`tp`**: Set the **prescribed time** (in seconds) for the Back-stepping controller's convergence deadline.
  * **`tMax`**: Adjust the total simulation time.
  * **`Ts`**: Modify the simulation time step. A smaller value (e.g., `0.001`) increases accuracy but slows down the simulation.
  * **`x0`**: Set the **initial conditions** (position, velocity, orientation) of the quadrotor.
  * **`FAULT_ANGLES`**: Introduce actuator faults by setting `ALPHA`, `BETA`, or `GAMMA` to non-zero values (in degrees, converted to radians).
  * **Controller Gains**: You can fine-tune the performance by adjusting the gain arrays:
      * `gammaCtrl`: Gains for the Back-stepping controller.
      * `K`, `a`: Gains for the Sliding Mode Controller.

-----

## 📈 Expected Output

After a successful run, several Matplotlib windows will appear, displaying:

1.  **Quadrotor Position States**: Plots of the actual vs. desired `x`, `y`, and `z` positions over time.
2.  **Quadrotor Orientation States**: Plots of the actual vs. desired roll (`φ`), pitch (`θ`), and yaw (`ψ`) angles.
3.  **3D Trajectory**: A 3D plot comparing the path taken by the quadrotor against the desired reference path.
4.  **Tracking Errors**: Plots showing the error for each state, illustrating how they converge to zero.
5.  **Control Signals**: The output torques and thrust generated by the controller over time.

-----

## 📦 Code Structure

  * `main()`: The main entry point. It initializes parameters, runs the simulation loop, and calls the plotting functions.
  * `BackSteppingControl()` / `SlidingModeControl()`: Contain the core logic for the two controller types.
  * `Rotor2_Dynamic()`: The physics engine of the simulation. It calculates the quadrotor's state derivatives based on its physical properties and the current control inputs.
  * `setDesiredTrajectory()`: Generates the reference trajectories (`CASE` 1, 2, or 3).
  * `stateCalculation()`: Performs the 4th-order Runge-Kutta (RK4) numerical integration to update the quadrotor's state at each time step.
  * `plotResults()`: Handles all data visualization.
