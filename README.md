# robosub_2023

Team Inspiration's repository for the RoboSub 2023 competition. We are using ROS, python, and C++.

*Please use this repository only for source files*; do not upload binary files like images/videos/ROSBags.

## Getting Started

### Prerequisites

We have two different subs, Onyx and Graey, they both run on different software setups because of compat issues with Jetson Nano and Jetson NX.

#### Onyx

* Ubuntu 20.04
* ROS Noetic
* Python 3.8
* OpenCV 4.7.0

#### Graey

* Ubuntu 18.04
* Ros Melodic
* CV on python 2.7, everything else could run on python 3.6
* OpenCV 4.7.0

Because of this, when we design our code, we need to make sure that it is compatible with both versions of python and ROS.
Please don't use any python 3.8 features (f-strings, walrus operator, etc.) and make sure that everything is compatible for both subs.

### Installing

#### For Onyx

(for the following, you can also use the https link instead of the ssh link)

```bash
git clone git@github.com:InspirationRobotics/robosub_2023.git 
mv robosub_2023 auv && cd auv
python3 -m pip install -e .
```

#### For Graey

```bash
git clone git@github.com:InspirationRobotics/robosub_2023.git 
mv robosub_2023 auv && cd auv
python3 -m pip install -e .
python2 -m pip install -e .
```

#### On your own computer

```bash
git clone git@github.com:InspirationRobotics/robosub_2023.git 
cd robosub_2023
python3 -m pip install -e ."[cv,dev]"
# or 
python2 -m pip install -e ."[cv,dev]"
```

### Repo Structure

In order not to have a messy repo, we have a specific structure for the repo. Please follow this structure when adding new features.

```bash
robosub_2023
├── auv
│   ├── device
│   │   ├── # everything related to the sensors
│   │   ├── # each device should have its own file/folder
│   ├── localization 
│   │   ├── # TODO
│   ├── mission 
│   │   ├── # Mission classes
│   │   ├── # see auv/mission/template_mission.py for an example
│   ├── motion
│   │   ├── # Actuators code
│   │   ├── # each actuator should have its own file/folder
│   ├── cv
│   │   ├── # All CV classes and functions
│   │   ├── # see auv/cv/template_cv.py for an example
│   ├── utils
│   │   ├── # utility functions
├── scripts
│   ├── # bash scripts
├── mission
│   ├── # Mission files (calling mission classes)
├── tests
│   ├── # Unit tests (pytest)
├── config.json # where all the config values used in mission files are stored
├── setup.py # setup file for pip install
├── .gitignore
├── README.md
```

### Running

#### Tests

This will run all the tests in the tests folder using pytest.
Any file that has the name `test_*.py` will be run.

```bash
pytest
```

#### Mission

We have a mission class for each individual mission/task.
Those classes are then called in a mission file (see `mission/` folder).
The mission file will be the one giving the mission class the necessary parameters to run the mission (for examples variables from the config file).

You can run individually mission classes for debugging purpose (if they have a `if __name__ == "__main__"` block) by running:

```bash
python3 -m auv.mission.<mission_name>
# or
python2 -m auv.mission.<mission_name>
```

Else, you can import the mission class in a mission file and run it from there.

```py
from auv.mission.<mission_name> import <MissionClass>

# initialize mission
mission = <MissionClass>() 
# run mission
mission.run() 
# cleanup mission after it is done
mission.terminate()
```

#### CV

The idea behind the CV classes is that they are run in a separate thread and they publish their results to a ROS topic.
You can run individually CV classes for debugging purpose (if they have a `if __name__ == "__main__"` block) by running:

```bash
python3 -m auv.cv.<cv_name>
# or 
python2 -m auv.cv.<cv_name>
```

## Contributing

### Unit Tests

We use pytest for unit tests. Please write unit tests for your code and make sure that they pass before pushing your code.
This is to make sure that we don't break anything when we add new features.
Tests are mainly there to make sure that there is no syntax error and that the code runs without crashing, not necessarily to make sure that the code is doing what it is supposed to do (although it is a plus).

### Code Style

It is always pleasant to read code that is well formatted and well commented.
Ideally we would want to format our code using black, so try to run regularly on your code:

```bash
black .
```

If you have trouble running this command, you can use the following command instead:

```bash
python3 -m black .
```
