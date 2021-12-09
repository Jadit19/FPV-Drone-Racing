# FPV-Drone-Racing

## Team Members:
- Adi Pratap Singh
- Adit Jain
- Anshul
- Karan

## Installation guide
This repo was tested out on ROS-Melodic and Ubuntu 18.04. Performance on other systems may vary.

In order to install the repo, first clone it in your system via:
```sh
git clone https://github.com/Jadit19/FPV-Drone-Racing.git
```

Now, initialize a catkin workspace inside the ``fpv_ws`` folder and build it via:
```sh
cd FPV-Drone-Racing/fpv_ws
catkin init
catkin build
```

Don't forget to source it before launching the project:
```sh
source devel/setup.bash
```

Great! Now launch the drone using:
```sh
roslaunch fpvbot_discription mav_hovering_example.launch
```