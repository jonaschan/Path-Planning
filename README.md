# Path Planning
For the first project in this term of the Udacity Self Driving Car course, we look into creating a path planner which helps the vehicle safely drive around a highway at least 4.32 miles without any incident with the following criterias:
-  Driving according to the speed limit of 50 mph and does not slow down unless obstructed by traffic
-  The vehicle does not exceed the maximum acceleration and jerk of 10 ms^-2 and 10 ms^-3 respectively.
-  Car does not have collisions.
-  Car stays in its lane except when it decides to switch lanes.
-  The car should be able to switch lanes when it is behind a slower moving car and an adjacent lane is clear of other traffic

## Implementation

My implementation of the path planner is based mainly on the spline function shown excellently in detail in the project walkthrough. As explained in the walkthrough, the spline function fits a line to a given set of points using a smooth function. I implemented this technique by feeding in points along the line and then back to the simulator.

My version of a so-called cost function is basically a boolean list containing decision that the vehicle makes as it drives along the road. To begin with, my vehicle starts in lane 1 (middle lane). There are two other lanes; lanes 0(left most lane) and lane 2(right most lane) that my car is able to maneuvre throughout its 4.32 mile journey.

### Changing Lanes
So, now that we got that out, lets begin discussing how I designed my path planner for my vehicle to drive itself for 30 minutes ~ 19.06 miles. My design takes into account the following criterias before making a turn (primary reason why the vehicle decides to turn is due to coming up behind a vehicle which is slower than my current speed) (line 265)
1. Where am I? 
-  My vehicle first checks the lane it is currently in. Then proceeds to check the remaining lanes with the criterias below.
2. Is there a car on the other lane? If yes, how far is the car? 
   Using the sensor fusion data, the vehicle goes through the list of vehicles in the current lane that it is checking and check their      projected S value using the following formula:

                                    *check_car_s += (double)prev_size * 0.02 * check_speed;
   This basically estimates the position of the car in the next frame corresponding to their current speed.
   I then calculate the current distance between the vehicle and mine(car_s):
                              
                                    *dist_s = check_car_s - car_s
                                    
   So I then check for two conditions for the car distances; distance is within 30 miles and distance greater than 30 miles.
   If the distance between the two vehicles are within 30 miles, it then adds the "cost" or a false boolean to a list containing the        decisions that it has made. However, if the distance is greater than 30 miles, we then check if it is possible to turn. This is done    by first checking that the speed of the car is less than mine. If it is, then send true boolean to the decision list (line 357). This    is done so the car does not have to wait for an opening wide enough for it to turn but check if the distance is wide enough and its      speed is appropriate so it can turn safely. The check for the speed is very important since I have multiple instances in my previous    iterations where it failed to check the speed and was rear-ended since the car in the other lane was much faster than mine (since I      have reduced the speed of my car due to the slow car in front of me).
   
   Now that I have considered the turning conditions, it is now time to decide to turn. So the first step in deciding to make a turn,
   I look through the list of decisions mentioned earlier. The main "activation" function is to check if there is false at that            particular instance in time (line 363).
   
   If there is a false at that particular instance, then the car stays in its current lane and waits until it is safe to turn (no false    in the decision list). If however, there is no false, it simply means that it is currently safe to turn, and its lane changes either    to the left or right depending on the lane it is currently in.

### Vehicle Speed
There have been multiple occurences that the car either reduces its speed too quickly, car increases its speed to slowly or car increases its speed to quickly. This was due to the fact that i was just manipulating the speed by reducing and increasing its speed by a fixed amount (0.1 MPH). I "resolved" this issue by assigning a check on the speed of the vehicle in front of me at any given time, hence, if there is a car in front of me and if I am going faster than that car, I will reduce my speed until it matches the speed of the car in front of me and if I am going to fast, reduce the speed until it matches the speed of the car in front of me. Other than that, if there are no vehicles in front of me, maintain a speed of 49.50 MPH.

### Difficulties
I faced several difficulties since I am currently unable to install SocketIO on my Mac and resorted to using Parallel Desktop and running a Linux Bash on Windows on my machine primarily to compile the code and run the simulator. This means I only manage to obtain a good result only when there is nothing else running on my machine. 
   
# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

