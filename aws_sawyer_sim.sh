#!/bin/bash

sudo apt -y install ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser ros-kinetic-sns-ik-lib ros-kinetic-moveit python3-pip python3-apt python3-colcon-common-extensions

sudo -H pip3 install --upgrade pip
sudo -H pip3 install -U setuptools
sudo -H pip3 install -U colcon-ros-bundle

cd simulation_ws/src
wstool init .
wstool merge sawyer_simulator/sawyer_simulator.rosinstall
sawyer_moveit.rosinstall
wstool update

source /opt/ros/kinetic/setup.bash

cd ~/environment/simulation_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
colcon bundle

aws s3 mb s3://sawyer-bucket-robomaker

aws s3api put-object --bucket sawyer-bucket-robomaker --key output/

aws s3 cp ~/environment/simulation_ws/bundle/output.tar s3://sawyer-bucket-robomaker/sawyer_sim.tar

aws robomaker create-simulation-application --name sawyer_sim --sources s3Bucket=sawyer-bucket-robomaker,s3Key=sawyer_sim.tar,architecture=X86_64 --robot-software-suite name=ROS,version=Kinetic --simulation-software-suite name=Gazebo,version=7 --rendering-engine name=OGRE,version=1.x
