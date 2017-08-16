# CarND-Path-Planning-Project by Abanoub David Awad
Self-Driving Car Engineer Nanodegree Program

## Project Goals
This car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change` lanes too. The car tries to avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  `[x,y,s,dx,dy]` values. `x` and `y` are the waypoint's map coordinate position, the `s` value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet `s` value, distance along the road, goes from `0` to `6945.554`.

## Challenges

This project comes with it a series of challenges, the goal is to be able to drive smoothly around the track such that humans would be sfe and comfortable were this a real self driving car. Using frenet coordinates, the issue of following the road is handled mostly for us, and the scope of this project is limited to generating trajectories of which lanes to switch to and when to make those switches.

-  cold start
   This was a relatively simple problem to sovle, we simply start at 0 velocity and slowly increase it at a fixed rate as we approach the speed limit.

- watching other cars and slowing down in response
  At any point we can use our sensors to detect whether there are nearby cars that could be dangerous.

- deciding when to switch lanes
  in this case, we make the decisions to switch lanes based on how close other cars are around us, and if there is room in one of the adjoining lanes that would make it worth switching lanes for.

  One improvement worth making is that if both lanes are logical possible switches tis controller currently makes no distinction as to which lane would be more advantageous to switch to in the "long run".

- developing points to follow
  The last part of this is separating our desired path through into points and generating smooth curves between them to follow in order to make sure that we don't have any jerking of the vehicle. For this problem we take to minimizing jerk using splines, a spline is guaranteed to go through all the points becuase it's a piecewise function of polynomials which is more powerful than one single polynomial of coefficients. Thanks

## Notes

In `CMakeLists.txt` the version of libuv was updated to 13.1, this may cause a compilation problem if the original version 11 from the project is being used. I changed this to remove some warnings and get the latest versions of libuv.

Including `util.cpp` directly was purely for convenience despite this being bad practice I think it's fine for the purposes of this project.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

`["x"]` The car's x position in map coordinates

`["y"]` The car's y position in map coordinates

`["s"]` The car's s position in frenet coordinates

`["d"]` The car's d position in frenet coordinates

`["yaw"]` The car's yaw angle in the map

`["speed"]` The car's speed in MPH

#### Previous path data given to the Planner

Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

`["previous_path_x"]` The previous list of x points previously given to the simulator

`["previous_path_y"]` The previous list of y points previously given to the simulator

#### Previous path's end s and d values

`["end_path_s"]` The previous list's last point's frenet s value

`["end_path_d"]` The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

`["sensor_fusion"]` A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Technical Details

1. The car uses a "perfect" controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

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


### Simulator.
Simulator download is [here](https://github.com/udacity/self-driving-car-sim/releases).

## Thanks

A really helpful resource for doing this project and creating smooth trajectories was using [this spline libary](http://kluge.in-chemnitz.de/opensource/spline/), the spline function is in a single hearder file is really easy to use.

Most of this project was written following the **VERY HELPFUL** guidance from the instructors! Thank you to the project coordinators and reviewers.

