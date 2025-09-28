# MATLAB-Robot-Kinematics-and-Trajectory-Simulation

This MATLAB repository simulates robotic trajectories and performs kinematic analysis for a 6-DOF robot model. It includes LSPB, bang-bang, and cubic trajectory generation, using Modified Denavit-Hartenberg parameters. Ideal for educational and research purposes, it demonstrates forward/inverse kinematics, trajectory planning, and visualization.

## Table of Contents
- [Installation](#installation)
- [Files Description](#files-description)
- [Usage](#usage)
- [Example Outputs](#example-outputs)
- [License](#license)

## Installation
1. Clone the repository:
   ```sh
   git clone https://github.com/vi4697/MATLAB-Robot-Kinematics-and-Trajectory-Simulation.git
   cd MATLAB-Robot-Kinematics-and-Trajectory-Simulation
   ```
2. Add the required paths in MATLAB by running `startup_rvc.m`:
   ```matlab
   startup_rvc;
   ```

## Files Description
- **trajectory_LSPB.m**: Simulates a robotic trajectory using the Linear Segment with Parabolic Blend (LSPB) method and performs forward and inverse kinematics analysis.
- **trajectory_bangbang.m**: Generates a bang-bang trajectory for the robot and performs similar kinematic analyses as in `trajectory_LSPB.m`.
- **trajectory_cubic.m**: Generates a cubic polynomial trajectory for the robot and includes kinematic analyses.
- **trajectory_LSPB_task.m**: Specific implementation of the LSPB trajectory for a pick-and-place task, including detailed steps for different robot configurations.
- **compute_euler_angle.m**: Function to calculate the Euler angle rotation matrix for given Z-Y-Z angles.
- **LSPB_trajectory.m**: Functions to generate synchronized LSPB trajectories for multiple joints.
- **compute_matrixT.m**: Function to compute the transformation matrix based on D-H parameters.
- **define_modified_DH.m**: Defines the Modified Denavit-Hartenberg parameters for the robot.
- **custom_inverse_kinematics.m**: Custom implementation of the inverse kinematics based on the robot's D-H parameters.
- **startup_rvc.m**: Script to add required paths and check dependencies for the Robotics Toolbox by Peter Corke.

## Usage
### Running the Main Scripts
1. **LSPB Trajectory Simulation**:
   ```matlab
   trajectory_LSPB;
   ```

2. **Bang-Bang Trajectory Simulation**:
   ```matlab
   trajectory_bangbang;
   ```

3. **Cubic Trajectory Simulation**:
   ```matlab
   trajectory_cubic;
   ```

4. **LSPB Trajectory for Pick-and-Place Task**:
   ```matlab
   trajectory_LSPB_task;
   ```

### Using Auxiliary Functions
- Calculate Euler Angle Rotation Matrix:
  ```matlab
  R_zyz = compute_euler_angle(th_z1, th_y, th_z2);
  ```

- Generate LSPB Trajectory:
  ```matlab
  [q, t_max] = LSPB_trajectory(si, v0, sf, vf, ti, tf, timestep);
  ```

- Compute Transformation Matrix:
  ```matlab
  T = compute_matrixT(a, alph, d, q);
  ```

- Define Modified D-H Parameters:
  ```matlab
  MDH = define_modified_DH();
  ```

- Custom Inverse Kinematics:
  ```matlab
  q = custom_inverse_kinematics(Ti);
  ```

## Outputs
Running each main script will generate plots of the robot's trajectory and display various kinematic calculations. For example, `trajectory_LSPB.m` will produce a 3D plot of the LSPB trajectory and display forward and inverse kinematics results in the MATLAB command window.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### Notes
- Ensure you have the Robotics Toolbox for MATLAB installed.
