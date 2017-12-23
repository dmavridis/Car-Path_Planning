# Introduction
In the Behavioral Planning project, a simulator is provided where the a self-driving car (ego car) is controlled and interacts with the environment, which consists of a road with 3 lanes in each direction as well as other cars. The ego car has to drive in a lane and avoid colisions or other dangerous interactions with the other cars. It needs to adapt to the speed of the car ahead and overtake when it is safe. Moreover, for the comfort of the passengers there are limitations in the speed, acceleration and jerk. 

# Approach 
The top level initial approach is to create two objects, the road and the vehicle. The vehicle one will include all the information related to position, velocity and behaviour of the vehicle. The two major behaviour functions are the lane and speed chance. All these methods are defined and impemented in this object. The road object is setting the landscape with the map coordinates and holds also information on the vehicles that are captured by the sensor fusion of the ego car. Moreover, it has the responsibility to find the best response of the ego vehicle to the changing environment. 

Unfortunately, there were implementation issues using the road object. The major one, was to pass the auto variable to the object. As a result, the road object only holds information on the map points. The major behaviour is therefore implemented in the main function.

# Description of the main function
The abstract description of the main function can be summarized in the following steps:
- Update the ego car position using the lastest sensor data
- Scan the sensor fusion data and find the closest s distances towards other vehicles for each lane
- Control the traffic by changing the behaviour of ego car, which includes either speed or lane changing
- Update the trajectory of the car

Steps 2 and 3 are explained in more detail, while more information for step 4 will be provided in the vehicle description section.

## Sensor fusion data processing
Checking the sensor fusion data which contain position and velocity information of the surrounding vehicles, the critical information is the distance of the front car in the same lane, as well as the distances of the closest front and rear vehicles of the left and right lanes of the ego vehicle, where applicable. This will give the ego car the window to perform a lane change. 

## Behaviour update
The purpose of the ego car is to drive at maximum permitted speed. When there is a car at the front moving slower the first priority for the ego car is to change lane and overtake the slower car. The sequence is to check first for the left and later for the right lane availability. If the car is in either the left or right most ones, only one direction lane change is available. If due to heavier traffic conditions any lane chance is not permitted, the car adjust its speed to the front one and is always checking for the conditions to allow the lane change. 

The implementation of the described behaviours is first checking whether ego car is getting closer than a safety distance to the front car. In the simple case that the speed needs adjusting, then the ego car is getting the command to reach the front car's speed. At the same time to avoid weird behaviour, that safety margin can close with decreasing speed and a constant distance between the involved cars is established. 

For the lane change, the safety window to the adjacent lanes is big enough then the ego car receives the command to change its lane number by 1. When this happens, a boolean variable `change_lane` is set, until the change is complete no other lane change command is allowed, to avoid again weird and unrealistic responses. The change is complete when the d Frenet coordinate of the car withing a 10% margin off the lane center. 

# Vehicle object description
The vehicle object holds the information on position, velocity and a few other control variables of the car. Two different types of vehicle types exist. The ego vehicle, which uses most of the available variables and the behavioral functions and the other vehicles, where only position, speed information and the UpdateOtherState function are used. 

## Trajectory generation
This is the most critical and hardest to implement function. The reason is that even when the correct points are calculated, there are discontinuities that lead to jeerky behaviour from the car. I spend lot of time on this problem and at the end used the approach from the walkthrough video, provided by the instructors. I had to tweak the prediction numbers though, as in some cases max jerk violation was observed.

# Performance 
The car is achieving more the 5 miles of error free behaviour. Running the model several times I noticed that there were couple of occasion were jerk limitation was violated by a little. 

# Reflections on the Project
This one was the most challenging projects of the class. It led me though to explore more the OOP approach and I was planning for a more structured approach. Due to implementation issues, I compromised with a simpler one. 

Additionally, the behaviour planning is rather simple. It is mainly using 3 states, keep lane with optional speed adjustment and change to left or right lane. There are neither more complicated states, as described in the course, nor any cost functions involved. It is an implementation that achieves the requirement and can be the basis for further progress. 

