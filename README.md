# Robotic Deposition
 
### Introduction

Climate change is expected to continue disturbing the agricultural industry and food systems supply chain through the increase in growing-season temperatures and drought frequency. Recent advances in Ag-tech have made indoor farming a viable option for year-round crop growth. This project aims to combine robotics with additive manufacturing techniques to further improve the efficient use of resources and address challenges in food production.

Research has shown that biphasic materials such as hydrogels and food-inks can be utilized as media for the purpose of efficient nutrient delivery. Hydrogels were introduced to agriculture as a way to retain water close to the plant roots and by combining them with fertilizers, the hydrogel can slowly release nutrients improving plant growth. 
The 3D printing of biphasic materials can be difficult due to the non-homogeneity in batch production and complex layer interactions. Furthermore, autonomous deposition of hydrogels for fertilizer delivery or as a growth substrate requires that the robot plans its trajectory while avoiding collisions with obstacles. Visual perception, identification, and modeling the objects in the environment each pose additional challenges especially in occlusion-heavy scenarios such as leaves on a plant.


### Overview
This repository will allow you to simulate a UR5e robot arm and command it to extrude a desired shape given by g-code.  

UR5e Simulator from Universal Robotics
![](https://github.com/tonyhzheng/robotic_deposition/blob/main/assets/ur5e_sim.gif)

GUI for visualizing the data from the robot arm
![](https://github.com/tonyhzheng/robotic_deposition/blob/main/assets/dearpygui.gif)

Live plot of the deposition path
![](https://github.com/tonyhzheng/robotic_deposition/blob/main/assets/matplotlib.gif)
## Usage

### Setup
This repository was tested on  laptop running 'Ubuntu 16.04.6 LTS' with nvidia-410 and cuda-10. If you have different nvidia/cuda versions on your local machine, go to the [docker-compose.yml](https://github.com/tonyhzheng/robotic_deposition/blob/main/docker-compose.yml) file and modify the volumes to the corresponding versions (lines 15-17).  The docker environment running the simulation and code is self-contained with the necessary packages/dependencies installed. Make sure you follow the instructions of installing Docker and Docker Compose in https://docs.docker.com/compose/install/


#### Step 1.
Open a terminal and change the current directory to where you downloaded this folder 

$ cd ~/your/file/path/to/aifs_robotic_deposition

#### Step 2. 

First time ever only - Build the image from the dockerfile using: 

$ bash build_dockerfile.sh 

#### Step 3.
Start the container

$ bash start_docker_container.sh 

#### Step 4.
Join the container (you can repeat this command to join the container in different tabs)

$ bash join_docker_container.sh 

#### Step 5.
Now you should be inside the docker container. Enter the following commands to build the ROS package.

$ cd bind_mount/ && catkin_make3 

#### Step 6.
Exit the container with 'Ctrl+D' and rejoin it so the newly built package is sourced.

### Running
To replicate the experiment of bioprinting a structure in the shape of a cube, complete the following steps:

#### Step 1.
Connect to http://localhost:6080/vnc.html?host=localhost&port=6080 in a web browser (tested on Mozilla Firefox).

Press "Confirm Safety Configuration".

Press the red button on the bottom left next to "Power off".

Press "On" and "Start".

Press "Exit" on the bottom left.

Press "Move" on the top left. 

You should now see the UR5e robot arm.

#### Step 2.
You can start running the example using the launch file:

$ roslaunch robotic_deposition gcode_bioprint.launch 
 

### Useful Docker Commands

#### Show running docker containers

$ docker ps 

#### Stop all containers (installation changes persist when you stop and start container)

$ docker stop $(docker ps -a -q) 

#### Stop all containers by force (use if container not responding)

$ docker kill $(docker ps -q) 

#### Remove all containers (installation changes DO NOT persist when you rm and restart container, note that changes to the workspace folder persist as it is a volume that is binded from the host computer to the container)

$ docker rm $(docker ps -a -q) 

#### Remove all images

$ docker rmi -f $(docker images -a -q) 

## Authors

You can view the list of authors in the [AUTHORS](https://github.com/tonyhzheng/robotic_deposition/blob/main/AUTHORS) file.

## Contact

For any questions, please contact us at tony_zheng@berkeley.edu.
 
## License

This project is licensed under the APACHE License. Please see the [LICENSE](https://github.com/AI-Institute-Food-Systems/aifs-github-best-practices/blob/main/LICENSE) file for details.
 
## Funding

This work is supported by the USDA/NSF AI Institute for Next Generation Food Systems (AIFS) through the AFRI Competitive Grant no. 2020-67021-32855/project accession no. 1024262 from the USDA National Institute of Food and Agriculture.
