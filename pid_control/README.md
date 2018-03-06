# CarND_PID-Control

Writeup for the PID Control project.

## P I D parameter effects

### P-Value effects
The proportional part represents the reaction speed of the correction. Higher values imply quick changes. 
Lower values imply a less rapid adjustment of the steering angle.

```
If an error has been observed, how fast should the controller react to the error and start the correction? 
```

### I-Value effects
The integral part represents a long term error memory. Since this value includes all previoulsy recodred errors, higher appearances for the P values will have a long term impact on future corrections.

```
What is the systematic error?
```

### D-Value effects 
The differential part represents the magnitude of the correction to be applied that has been observed between two consecutive errors. Higher values of D lead to sharp and lower values lead to smooth corrections.

```
What is the magnitude of the correction?
```
 

## Parameter tuning
For this particular project I have tried to understand the impacts of every parameter has on the resulting steering of the vehicle. 
During my observations it became obvious that a lower value for P was necessary as one "overshoot" would leave to an endless attempts of the controller to normalize the steering. 
The major factors for this value are the curvature of the simulation track as well as the speed of the vehicle.
My observations revealed that a drift of the steering is not highly likely.
Since the  D value of the controller represents the magnitude of a change it has been expected to be higher than the P value. 

By try and error I have selected the following parameterset:
```
Kp = 0.1
Ki = 0
Kd = 0.9
```
This imolies the following logic: 

```
React smoothly on errors with a high magnitude and omit a systematic error
```

   
