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

## 1st Assignment of Bipedal Locomotion Course
Design a dynamic Simulation of the Rimless Wheel to develop phase portraits of the dynamics. Submit plots of the phase portraits as they vary from different slopes. Plot how the limit cycles of these phase portraits vary when contacts occur at different times. Explain how these elements relate to the physical parameters of the rimless wheel.
Gamma = 0.0
![Alt text](assets/1.gif)
Gamma = 0.07
![Alt text](assets/2.gif)
Gamma = 0.08
![Alt text](assets/3.gif)
Gamma = 0.09
![Alt text](assets/4.gif)
Gamma = 0.1
![Alt text](assets/5.gif)
Gamma = 0.5
![Alt text](assets/6.gif)
Gamma = 1.0
![Alt text](assets/7.gif)
Gamma = 3.0
![Alt text](assets/8.gif)
Gamma = 6.0
![Alt text](assets/9.gif)
Gamma = 6.2
![Alt text](assets/10.gif)
Gamma = 6.3
![Alt text](assets/11.gif)
Gamma = 7.0
![Alt text](assets/12.gif)

