# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Reflection

The following approach was used to arrive at the solution.

1. First, I attempted to make the car just [drive in a straight line](https://github.com/Deborah-Digges/SDC-ND-term-3/blob/9ccccf63b7ef29e579486e4cc0adf8b828e8c7ad/p1-path-planning/CarND-Path-Planning-Project/src/main.cpp#L244-L249). This was done quite easily using the code snippets provided in the classroom, by adding a small increment to the `x` and `y` coordinates.

```
  double dist_inc = 0.5;


  for(int i=0; i<50; ++i) {
    next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
    next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
  }
```

2. Next, I attempted to make the car [drive along it's lane](https://github.com/Deborah-Digges/SDC-ND-term-3/blob/ea307536e81210f49676466178749894a17c4006/p1-path-planning/CarND-Path-Planning-Project/src/main.cpp#L244-L261). This was done by incrementing the car's s coordinate by a small value for each point and by maintaining a constant d coordinate. In Frenet coordinates, this corresponds to a car moving along it's lane. There was some tuning required for the distance increment to prevent the car from exceeding the speed limit.

```
double dist_inc = 0.5;


for(int i=0; i<50; ++i) {
  // use i+1 instead of i, else the first point will be exactly where the car is at
  // and it won't be transitioning
  double next_s = car_s + dist_inc * (i + 1);


  // we are in the middle lane
  // the waypoints are measured from the double yellow lane in the middle of the road
  // so we're about 1.5 lanes from where the waypoints are
  // Each lane is 4m wide. To stay in the middle of the current lane, d should be 1.5 * 4 =  6
  double next_d = 6;


  vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);


  next_x_vals.push_back(xy.at(0));
  next_y_vals.push_back(xy.at(1));
}
```

3. The car was now following it's lane but exceeding the limits for both acceleration and jerk. In order to generate smoother, less jerky paths, a [spline was used to generate the trajectory](https://github.com/Deborah-Digges/SDC-ND-term-3/blob/ab2cab3656402e6fc48f01f3ca8eee5ee40cbf1c/p1-path-planning/CarND-Path-Planning-Project/src/main.cpp#L258-L355). The main steps involved in trajectory generation using a spline included:
- **Creation of a starting reference for the trajectory**: The car's coordinate or the last two points in the previous trajectory are used to create a starting reference for the trajectory
- **Creation of a widely spaced list of waypoints (x,y)**: The current waypoints were sparsely spaced which resulted in jerky behaviour around egdes when converting between Frenet and Cartesian Coordinates.
  - Three points are placed in 30m intervals ahead of the reference point by incrementing the s coordinate and keeping d constant, resulting in 4 points: `[ref_s, ref_s+30, ref_s + 60, ref_s + 90]`.
- **Coordinate tranformation of widely spaced waypoints**: A coordinate transformation of these 4 points is done to make the math simpler. The coordinates are transformed into the car's coordinate system.
- **Spline fitting**: A spline is fit to these transformed points
- **Generating points along the spline**: We [break up the spline points](https://github.com/Deborah-Digges/SDC-ND-term-3/blob/ab2cab3656402e6fc48f01f3ca8eee5ee40cbf1c/p1-path-planning/CarND-Path-Planning-Project/src/main.cpp#L330-L354) into small distance increments so that we travel at the desired velocity. 
- **Generate Trajectory**: This set of points is transformed back to the global coordinate system and constitutes our generated trajectory.

4. The car now moved quite smoothly but still collided with other vehicles along it's path. In order to detect and prevent collision, [the sensor fusion data about the surrounding vehicles was used](https://github.com/Deborah-Digges/SDC-ND-term-3/blob/ed31cfd09e7f0ff573d7a21f059ebc37af26e55b/p1-path-planning/CarND-Path-Planning-Project/src/main.cpp#L253-L278). The sensor fusion data was searched for any cars in the current lane of the car within some threshold distance. If there was such a car, the eho vehicle was made to slow down gradually to avoid collision. If there was no car detected in the car's lane, the car was made to accelerate gradually until the desired velocity was achieved.

5. The car now moved without colliding with other vehicles. However, it is possible that the car could be stuck behind a slow vehicle for a long time, even when there is a free lane to which it can switch.

## Video

The project can be seen in action in the video found [here](https://www.youtube.com/watch?v=53QXQfbnrG4&feature=youtu.be)

## Simulator. 

You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.


## The map of the highway

It is in data/highway_map.txt. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


Here is the data provided from the Simulator to the C++ Program

## Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH


## Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

## Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

## Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.


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