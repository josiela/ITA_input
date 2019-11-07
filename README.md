# ITA_input
Repository that features the code for the input tests from the interaction design course ITA (HAW Hamburg).
It features programming crazyflie2.1 drones.

### Dependencies
https://github.com/bitcraze/crazyflie-lib-python

### Setup
If you want to use the drone class in your code (and this should be your goal if you are here), clone this repository and set the pythonpath to the location of the repo.
How to add to the pythonpath: https://bic-berkeley.github.io/psych-214-fall-2016/using_pythonpath.html

### Structure
Drone.py - Class that handles the (dis)connection to the drone. Should be filled with "standard" functions like flyToPoint(x,y,z) or doABackflip() in the process. Feel free to contribute with ideas.

DroneTest.py - Short "test" (you can't really call it a test) that the drone class is doing its job

./speechrecognition/ - Files that combine the crazyflie-drone with speechrecognition

