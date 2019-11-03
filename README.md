# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway
with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
You will be provided the car's localization and sensor fusion data,
there is also a sparse map list of waypoints around the highway. The car
should try to go as close as possible to the 50 MPH speed limit, which
means passing slower traffic when possible, note that other cars will
try to change lanes too. The car should avoid hitting other cars at all
cost as well as driving inside of the marked road lanes at all times,
unless going from one lane to another. The car should be able to make
one complete loop around the 6946m highway. Since the car is trying to
go 50 MPH, it should take a little over 5 minutes to complete 1 loop.
Also the car should not experience total acceleration over 10 m/s^2 and
jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are
the waypoint's map coordinate position, the s value is the distance
along the road to get to that waypoint in meters, the dx and dy values
define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance
along the road, goes from 0 to 6945.554.


#### Main car's localization Data (No Noise)
#### 
["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner
#### 
//Note: Return the previous list but with processed points removed, can
//be a nice tool to show how far along
the path has processed since last time.

`["previous_path_x"]` The previous list of x points previously given to
the simulator

`["previous_path_y"]` The previous list of y points previously given to
the simulator

#### Previous path's end s and d values 

`["end_path_s"]` The previous list's last point's frenet s value

`["end_path_d"]` The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

`["sensor_fusion"] A 2d vector of cars and then that car's [car's unique
ID, car's x position in map coordinates, car's y position in map
coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s
position in frenet coordinates, car's d position in frenet coordinates.

## Model Documentation
The goal of this project is to create a pipeline which safetly driving
on the highway, passing the other cars and switching the lanes if
required. I took the approach described in the lection and splitted my
pipeline into 3 parts:


- Prediction 
- Behavior 
- Trajectory


<img src="https://miro.medium.com/max/3298/0*jFtdKafJICJcwQKc" 
	 width="700" height="450" />

We are given a map of the highway, as well as sensor fusion and
localization data about our car and nearby cars. We are supposed to give
back a set of points `(x , y)` in a map that a perfect controller will
execute every 0.02 seconds.

Before starting with the implementation of behavior, prediction and trajectory parts of the pipeline, I've implemented several helper function which are used in the pipeline:
- Convert function from Degree in RAD 
- Distance calculation
- Calculation of the "Closest Waypoint"
- Calculation of the "Next Waypoint", required because the closest waypoint can also be behind the car
- Transformation in Frenet Coordinates and also back in XY Coordinate System

### Prediction
Prediction deals with the telemetry and sensor fusion data. In my code we check if another vehicle is upfront, left or right of my car.
And predict based on the velocity of other cars, where they would be in the next moment.
I've also added the "car_close" variable after running the simulator for the first time. Here I check if another vehicle has a very short distance to my car 
and if yes, I apply the emergency brake.

```cpp
if(prev_size > 0) {
                car_s = end_path_s;
            }
            
            bool car_left= false;
            bool car_right = false;
            bool car_ahead = false;
            bool car_close = false;
                        
            for(int i=0; i < sensor_fusion.size(); i++) {
                float d = sensor_fusion[i][6];
                int check_car_lane;
                
                if(d > 0 && d < 4) {
                    check_car_lane = 0;
                    
                }
                else if(d > 4 && d < 8) {
                    check_car_lane = 1;
                    
                }
                else if(d > 8 and d < 12) {
                    check_car_lane = 2;
                    
                }
                
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];
                
                //This will help to predict the where the vehicle will be in future
                check_car_s += (prev_size*0.02*check_speed);
                
                if(check_car_lane == lane) {
                    car_ahead |= check_car_s > car_s && (check_car_s - car_s) < 30;
                    
                }
                else if((check_car_lane - lane) == -1) {
                    car_left |= (car_s+25) > check_car_s  && (car_s-25) < check_car_s;
                    
                }
                else if((check_car_lane - lane) == 1) {
                    car_right |= (car_s+25) > check_car_s  && (car_s-25) < check_car_s;
                    
                }
                else if(check_car_lane == lane) {
                    car_close |= check_car_s > car_s && (check_car_s - car_s) < 10;
                }
            }
```

### Behavior

Behavior describes the the part of the algorithm which helps the vehicle to decided what to do during any point of time.
Since this project only considering the lane changes I've resigned to implement the cost function.
Besed on the prediction outcome my vehicle keeps the lane, reduce the speed or changes the lane to the left or to the right.
After running the simulator I've also implemented an emergency brake (5 * speed_diff) in case other vehicle are reducing 
the speed very fast or changing the lane with very close distance to my car.

```cpp
            if(car_ahead) {
                if(!car_left && lane > 0) {
                    lane--;
                    
                }
                else if(!car_right && lane !=2) {
                    lane++;
                    
                }
  
                else if (car_close){ //emergency break
                    ref_vel -= 10 * speed_diff;
                }
                else {
                    ref_vel -= speed_diff;
                }
                
            }
            else if(ref_vel < max_vel){
                ref_vel += speed_diff;
                
            }
```

### Trajectory planning

I've implemented the trajectory planning algorithm based on the walk through in the Q&A of the project.
For the trajectory generation we're using the polynomial spline function provided in the project description.
In order to have a smooth path, we're using the latest 2 points from the previous path and add to them 3 future waypoints.
So in total we have 5 points in x and 5 in y direction.
```cpp
vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);
ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
```


After initializing the spline,I've added the previous points to `next_x_vals` and `next_y_vals`
as it going to be the final control values pass it to the simulator. In the next step we need to find
 all spline points till the horizon (approx. 30m).
 
 ![](https://miro.medium.com/max/792/0*EKMMAlaeO7rh4zgo.jpg)
 
The spline points from start to horizon y points can be calculated by using the formula mentioned in the picture.
The fututre points are also added to the `next_x_vals` and `next_y_vals` arrays.
