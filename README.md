# Drake

This repository is part of my class project (Bipedal Walking Robots - EEE 6734) at Intelligent Systems and Robotics department in UWF. 

Model-Based Design and Verification for Robotics.

Please see the [Drake Documentation](https://drake.mit.edu) for more
information.

## Installation
This installation instruction was tested on ubuntu 20.04.
```
mkdir ~/drake_from_source
cd ~/drake_from_source
python3 -m venv drake_from_source_venv
source drake_from_source_venv/bin/activate
git clone --filter=blob:none https://github.com/RobotLocomotion/drake.git
sudo ./setup/ubuntu/install_prereqs.sh
```
For python bindings, you have to install some dependencies.
```
ln -s /usr/bin/python3-config ~/drake_from_source/drake_from_source_venv/bin/python3-config
ln -s /usr/include/python3.8 ~/drake_from_source/drake_from_source_venv/include/python3.8
# you can also directly copy the python3.8 from the /usr/include directory to ~/drake_from_source/drake_from_source_venv/include directory.
pip3 install PyYAML
pip3 install numpy
pip3 install matplotlib
```
Now, build the repo.
```
mkdir drake-build
cd drake-build
cmake ../drake
make -j
```
For source installation, follow the instruction from this [page](https://drake.mit.edu/from_source.html).

For binary installation, follow the instruction from this [page](https://drake.mit.edu/pip.html#stable-releases).

For tutorial, follow the instruction from this [page](https://github.com/RobotLocomotion/drake/blob/master/tutorials/README.md).

## Bipedal Walking Robots
### 1st Assignment
Design a dynamic Simulation of the Rimless Wheel to develop phase portraits of the dynamics. Submit plots of the phase portraits as they vary from different slopes. Plot how the limit cycles of these phase portraits vary when contacts occur at different times. Explain how these elements relate to the physical parameters of the rimless wheel.

### Answer:
A rimless wheel acts as a passive dynamic walker.
#### Slope, 𝜸 = 0.0
![Alt text](assets/1.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/1_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 0.07
![Alt text](assets/2.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/2_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 0.08
![Alt text](assets/3.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/3_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 0.09
![Alt text](assets/4.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/4_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 0.1
![Alt text](assets/5.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/5_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 0.5
![Alt text](assets/6.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/6_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 1.0
![Alt text](assets/7.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/7_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 3.0
![Alt text](assets/8.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/8_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 6.0
![Alt text](assets/9.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/9_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 6.2
![Alt text](assets/10.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/10_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 6.3
![Alt text](assets/11.gif)

##### Phase diagram:

<p align="center">
  <img src="assets/11_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait for a rimless wheel on a level surface (slope of 0.0 radians) showcases a distinct dynamic pattern, illustrated by the blue lines which represent the trajectory of the wheel's angular velocity (`thetadot`) against its angular position (`theta`). The star marker indicates the initial condition from which the wheel's motion is simulated. Notably, the trajectories converge into a horizontal band, indicating that, irrespective of the starting angular velocity, the system stabilizes to a steady-state motion with minimal fluctuation in `thetadot`. The lack of a slope means there is no gravitational contribution to the wheel's potential energy, leading to a motion primarily governed by the initial kinetic energy and dissipation due to the ground contact. The uniformity of the gray energy contour lines suggests that energy dissipation is consistent across collisions, which is expected in the absence of a slope. This behavior is indicative of the natural tendency of the rimless wheel to reach a constant rolling speed on a flat surface, highlighting the inherent stability of the system under such conditions.
</p>

#### Slope, 𝜸 = 7.0

##### Phase diagram:

![Alt text](assets/12.gif)

<p align="center">
  <img src="assets/12_phase.png" alt="Description of Image" width="500">
</p>

<p style="font-size: small;">
    The phase portrait on a 7.0-radian slope reveals a highly dynamic system, where the blue trajectory depicting angular velocity (`thetadot`) against angular position (`theta`) shows multiple oscillations indicative of rapid accelerations and decelerations. Starting from the initial condition marked by a star, the wheel experiences a significant conversion of gravitational potential into kinetic energy as it descends the slope, followed by energy loss as it climbs, creating a wave-like pattern. The nearly vertical sections of the trajectory suggest the impact points of the wheel's legs with the ground, leading to abrupt changes in motion. The gray contour lines represent constant energy levels, enclosing the trajectory and indicating a potential perpetual motion if external forces were absent. This complex behavior underscores the influence of gravity on the wheel's stability and the challenges it faces in maintaining a predictable path on such steep inclines.
</p>


