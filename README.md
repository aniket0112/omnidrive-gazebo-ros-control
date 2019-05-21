# omnidrive-gazebo-ros-control
Mobility control simulation on ROS Gazebo simulation for three wheel omnidrive robot
![Image Not Found](/images/omni_bot.png)
## Getting started

### Prerequisites
* [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) with Gazebo-9 (Desktop-Full Install)
* [Python 2.7](https://www.python.org/downloads/source/) with [numpy](https://askubuntu.com/questions/868599/how-to-install-scipy-and-numpy-on-ubuntu-16-04)

### Installing
* Clone the repository to the local machine
* Place the repository folder in ```/your/machine/catkin_ws/src```
* Navigate into ```catkin_ws``` folder and open terminal.
* Type ```catkin_make``` and press enter.
* Make sure your ```.bashrc``` file (located in Home Folder) has these lines added in them:
```source yourpath/ros/melodic/setup.bash```
```source yourpath/catkin_ws/devel/setup.bash```
* After catkin has completed making the workspace, ROS should be able to locate your new rospackage ```omni_bot```. To check it, type in terminal ```rospack find omni_bot```. This should print out the location of the ```omni_bot``` folder.
If the rospackage location is not displayed, please contact the collaborators.

### Simulating an Induced Mode Control
* Open a terminal and type ```roslaunch omni_bot gazebo.launch```. A Gazebo world should be loaded with a three wheel omnidrive robot surrounded by a blue colored disc (Laser Sensor visualiztion).
* Open another terminal inside the folder by entering command ```cd /your/machine/catkin_ws/src/omnidrive-gazebo-ros-control/omni_bot``` and type ```python main.py```.
* The robot will start to move in a sinusoidal curve as per default follow curve function. To change to go to goal behaviours, comment line 57 in ```main.py``` and uncomment line 56.
*```hybrid_automata.py``` has all the constants and configuration parameters defined for the Python controller design.
*The folders ```urdf``` and ```config``` collectively constitute the model specifications for the physics engine of simulator. A good way to understand about editting them is following ROS Gazebo tutorials [here](http://gazebosim.org/tutorials) or follow up [my blog](https://medium.com/@aniket0112/twolinkman-3b326c1320eb) for a crashcourse
*NOTE* : If anything is logged in red color while running any of the above commands, there has been some error. Contact collaborators in such case with the log messages.

### URDF for Omniwheel
* URDF for omniwheel is special because it is not as straightforward to design an omniwheel in Gazebo as it may seem at first thought. There are about 12 rotational joints on each wheel and there are three wheels.
* Of course, you can code each of those 36+3 joints one by one but that would easily drive you nuts. Macros in xacro saves the day. Read more about it [here](http://wiki.ros.org/xacro#Macros)
* A macro was made for a roller and then for the wheel rim. Each rim macro consists of 12 rollers with joint positions specified w.r.t. to the wheel origin. 
^ This way main body consists of only defining three wheels and their joint location and _voila_! The whole omnidrive robot is generated like a piece of cake! 

## Built with
* [SolidWorks](http://www.solidworks.in/Default.htm) - 3D CAD model
* [ROS Meloidc](http://wiki.ros.org/melodic) - ROS control node
* [Gazebo 9](http://gazebosim.org/) - Simulation environment
* [Python 2.7](https://www.python.org/) - Python interface for easy UI

## Authors
* [Aniket Sharma](https://github.com/aniket0112)

##Acknowledgements
[OpenBase](https://github.com/GuiRitter/OpenBase) is a great work on providing people with customizable three wheel omnidrive robot. I got the idea of using macros by looking at his codes. Thanks!
