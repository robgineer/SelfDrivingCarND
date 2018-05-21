# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
  
## Boundary Conditions 
Since the car should coomply with all traffic rules and moreover should drive comfortably, I have defined a set of rules. 

### Safety Specific 
The car should follow the set of 3 rules

1. Never collide with any dynamic or static objects 
2. Always stay in a lane unless a lange change is performed
3. Perform only one lange change at a time
     
Rule no. 1 was implemented using a simple check for potential "target" objects that are in front of the EGO (target zone) and potential collision objects (collision zone) next to the EGO.
```
Target Zone s := EGO_s+10, ... , EGO_s+30
Target Zone d := EGO_lane
`` 
`` 
Target Zone s := EGO_s-10, ... , EGO_s+15
Target Zone d := !EGO_lane 
`` 
Note that the target and the collision zone overlap.

Rule no. 2 was implemented using the lane counter representing the current lane and the first point for the spline generation. For a lane change it is either incremendet or decremented by 1 (representing rule no. 2). 

 
### Comfort Specific 
In order to ensure a smooth motion, the following rules were applied.

1. Jerk free for in longitudinal and latitudinal direction
2. No subsequent lange changes 

Ensuring jerk free motion was covered using the spline library and a small value for the acceleration / deceleration while the restriction in subsequent lane changes was implemented with a delay (state_counter). 

## State Machine
My implementation includes only two states: FOLLOW_LANE and TRACK_GAP, while a lane change is covered within  last. 

## Cost Function
Represents the dynamic part. As I felt that traffic rules are static boundary conditions, the cost function is  limited to the velocity of the EGO vehicle. 

## Trajectory Generation 
I have used the insights from the project intro and implemented a spline based generator with 3 points that define the spline and 50 reference points for the motion control. In order to guarantee smoothness in between two execution cycles, two consequent splines are stiched.  

## Further Improvements
### Cost Function
1. Action Based Cost Function: instead of using the lane as a target for the cost estimation, I would use the actions within this 
2. More complex 

### State Machine
Although the 2 implemented states represent a good approach for the general task, more states would increase the performance of the EGO.  

### Prediction Horizon
My implementation generates only one action at a time and extrapolates the remaining traffic participants linearly. In order to ensure more human like interactions and behaviors, the prediction horizon (and the prediction complexity) should be increased for long term planning.
     


