# CarND-Path-Planning-Project Writeup
  
## Boundary Conditions 
Since the car should coomply with all traffic rules and moreover should drive comfortably, I have defined a set of static conditions that the EGO has to follow. 

### Safety Specific Boundaries

1. Never collide with any dynamic or static objects 
2. Always stay in a lane unless a lange change is performed
3. Perform only one lange change at a time
     
Rule no. 1 was implemented using a simple check for potential target objects that are in front of the EGO (target zone) and potential collision objects (collision zone) next to the EGO.
```
s_target_zone := EGO_s+10, ... , EGO_s+30
d_target_zone := EGO_lane

s_collision_zone  := EGO_s-10, ... , EGO_s+15
d_collision_zone := !EGO_lane 
``` 
Note that the target and the collision zone overlap.

Rule no. 2 was implemented using the lane counter representing the current lane (+the first point for the spline generation). For a lane change it is either incremendet or decremented by 1 (representing rule no. 3). 

 
### Comfort Specific 

1. Jerk-free in longitudinal and latitudinal direction
2. No direct subsequent lange changes 

Ensuring jerk-free motion was covered using the spline library and a small value for the acceleration / deceleration while the restriction in subsequent lane changes was implemented using a delay for each state transition (refer below for the states definition). 

## State Machine
My implementation includes only two states: FOLLOW_LANE and TRACK_GAP, while a lane change is covered in TRACK_GAP. 

## Cost Function
Represents the dynamic part. Since I have defined that that traffic rules are static boundary conditions, the cost function is limited to the velocity of the EGO vehicle and the distance to the potential objects on the desired lane. 

## Trajectory Generation 
I have used the insights from the project intro and implemented a spline based generator with 3 points with distance of ```s=30```  that define the spline and 50 reference points for the motion control. In order to guarantee smoothness in between two execution cycles, two consequent splines are stitched.  

## Improvements
Since the current solution represents a lean (but working) approach, the following improvements are necessary to handle more complex driving scenarions such as traffic jams. 

### Cost Function
1. Action Based Cost Function: instead of using the lanes, one improvement is using the actions (states) as the target for the cost estimation. 
2. Increase of complexity in cost estimation: the currently implemented cost function includes the desired velocity and the distance to potential target objects. By adding more inputs as the past velocities of the remaining traffic participants will increase the EGO's ability to maneuver around the track given more difficult conditions.

### State Machine Extension
Although the 2 implemented states represent a stable approach for the general task, more states would increase the safety of the EGO's actions. As the remaining traffic participants in the simulation move quite deterministic an abort in lane changes is not covered at the moment, for instance.
     
### Improved Trajectoriy Calculation
1. While the current implementation has only a static collision check, a better solution is to check if the generated spline is collision-free as well. This would decrease the probability of a collision during a lane change.
2. Considering that in real world scenarios waypoints are not as accurate as in the given simulation, a calculation of trajectory sets is necessary.

### Prediction Horizon
My implementation generates only one action at a time and extrapolates the remaining traffic participants linearly. In order to ensure more human like interactions and behaviors, the prediction horizon and the prediction complexity (implying a data driven approach) should be increased for long term planning. In combination with the improvements stated above, this would increase the robustness and the comfort aspects of the vehicle movement.

