# robosub_2023
Team Inspiration's repository for the RoboSub 2023 competition. We are using ROS, python, and C++.

*Please use this repository only for source files*; do not upload binary files like images/videos/ROSBags.

**TO DO:**

write the .gitignore file

create the basic empty template programs 
- one for a hypothetical sensor stub, one mapper, a mission planner, and one for MAVROS (this will be made into a path follower)

verify integration using the template programs



**Branch Breakdown**

- Communication:Modem - This folder contains the software that Kyle Jacob worked on for the waterlinked modems 

- custom_msgs 

- map

- mission & map stuff 

- mission 

- motion

- movementfrom2019 - This folder contains the software what we used for the demo for the 2023 San Diego regional sea pearch competition. We used the movement logic from 2019 and in this folder we also copied over the files that contain the classes and functions that we called and used in the autonomous code.

- sensors - This folder contains the basic templates for sensor stubs as well as the sensor wrappers (initializes the functions and does logging)
