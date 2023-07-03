# robosub_2023

Team Inspiration's repository for the RoboSub 2023 competition. We are using ROS, python, and C++.

*Please use this repository only for source files*; do not upload binary files like images/videos/ROSBags.

## Setting up a virtual env

We are using python 3.8.10 on the sub, it is recommended to dev on the same version.
In order to have a clean install, we will be using a virtual environment.

```
pip install virtualenv
```

Then, create a virtual environment in the root of the repo:

```
virtualenv venv
```

To activate the virtual environment, run the following command:

```
activate venv
```

Then install the dependencies using pip:

```
pip install -r requirements.txt
```

If a package is missing, please add it to the requirements.txt file and run the command again.

**TO DO:**

write the .gitignore file

create the basic empty template programs

- one for a hypothetical sensor stub, one mapper, a mission planner, and one for MAVROS (this will be made into a path follower)

verify integration using the template programs
