# Dynamic-PathFinding
This is a repo for a year 3 project for computer science for the University of Lincoln 2023-24. This project aims to achieve the following:
An autonomous robot that knows nothing about its environment apart from where its starts (x&y coordinates) and the goal (x&y coordinates). This "roomba" style robot features differential drive and will be able to path find and path plan through this environment
exploratorarly or through used previous explorations information to navigate to a target

## Documentation incoming
### Requirements for this project
- Ros 1 Noetic
- Ubuntu 18.04
- catkin build tools

### Usage & Information

To use this package this package must be installed inside a package housing inside the src directory after this the catkin_build command can be used to compile the package and then its ready to use.
After building and sourceing the package to activate the package use ## insert command here ## in a terminal and in a separate terminal run ##insert command here ## this will launch the gazebo simulation as well as the run the package code respectively
### Robot Behaviour
The robot's behaviour can be defined as the following:
If the robot has knowledge of the intended target from previous runs the robot will ask the user if they want to use them depending on the circumstance the robot will either navigate a fixed path to/towards the goal, otherwise it will exploratvely navigate the environment for the goal. This robot is more than capable of finding multiple routes to the same position and if it struggles for too long than a different metric for the AD* is activated.
#### Extra information
To modify where the robot will try to get to see the "Addtional reasources" folder and place the Nodes.txt and the Target.txt into the root of the package and this will allow the program to read them, next go in to the Target.txt and this is wehere you can add or remove protential targets to your liking



## Known bugs/problems
This project is not a polished product but the main programs are:
- If the target cannot be found there isnt a gracefull decay, just a robot that gets stuck with a message on the terminal along the lines of "conducting a regression".
- The robot is not guarenteed to take the most time or distance effiecent route to the target
- if this robot were to be used in a more complex environment with obstacles such as chairs its likely going to collide as the the obstacle avoidance is based on select lidar beams and not "collision regions"

## Demonstration
For a demonstration of this project see this youtube link: ## insert youtube link
