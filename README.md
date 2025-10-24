# Formula Student Lap Time Simulator & Setup Optimizer

This repository contains a comprehensive MATLAB project for vehicle dynamics simulation, specifically tailored for optimizing Formula Student / FSAE car setups. The simulator uses a quasi-steady-state, point-mass model to accurately predict lap times around any given track.

The core feature is its ability to test and compare multiple steering geometry configurations. By reading detailed data files that correlate turning radius to Ackermann percentage, the simulation dynamically adjusts the tire's coefficient of friction to model tire scrub. This allows for a quantitative comparison to find the setup that is theoretically fastest.

---

## 🚀 Key Features

* **Lap Time Prediction:** Accurately calculates the minimum possible lap time using a forward (acceleration) and backward (braking) pass algorithm.
* **Setup Comparison:** Automatically runs simulations for multiple setup files (defined in `.csv`) and generates a summary table comparing their lap times.
* **Dynamic Grip Modeling:** The `get_effective_mu` function adjusts the peak coefficient of friction based on the Ackermann percentage for each specific corner radius, modeling the real-world effects of tire scrub.
* **Data-Driven:** All vehicle, track, engine, and setup parameters are loaded from external `.csv` files, making it easy to test new configurations.
* **Visualization Suite:**
    * Generates static plots of the fastest lap's velocity profile.
    * Creates a 2D animated race visualization comparing the performance of different setups in real-time.

---

## ⚙️ How It Works

1.  **Track Processing:** The track is defined by X-Y coordinates, which are used to calculate segment lengths and the radius of curvature at each point.
2.  **Cornering Limits:** The maximum possible cornering speed is calculated for every point on the track, limited by tire grip and downforce.
3.  **Dynamic Ackermann Lookup:** For each corner, the simulator uses an interpolant to look up the specific Ackermann percentage from the setup file based on the corner's radius. This value is used to penalize grip if it deviates from the ideal target (-100%).
4.  **Longitudinal Simulation:** A forward pass calculates acceleration limited by engine power and drag. A backward pass then calculates the required braking points to ensure the car can slow down for the next corner.
5.  **Lap Time Calculation:** The final, optimized velocity profile is used to calculate the time taken to traverse each track segment, which are then summed for the total lap time.

---

## USAGE

1.  **Prepare Data:** Ensure all your data files (`track_data.csv`, `torque_curve.csv`, `gear_ratios.csv`, and `setup_X.csv`) are correctly formatted and placed in the project folder.
2.  **Configure Script:** Open the main script and update the `setup_files` cell array with the names of the setup files you wish to test.
3.  **Run:** Execute the script in MATLAB. The results and plots will be generated automatically.
