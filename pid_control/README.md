# CarND_PID-Control

Writeup for the PID Control project.

## P I D parameter effects

### P-Value effects
The proportional part represents the reaction speed of the correction. Higher values imply quick changes. 
Lower values imply a less rapid adjustment of the steering angle.

```
If an error has been observed, how fast should the controller react for the error correction? 
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
My observations revealed that a drift of the steering (due to incorrect calibration for instance) is not highly likely.
Since the  D value of the controller represents the magnitude of a change it has been expected to be higher than the P value. In fact, I was expecting the P value to be only a fraction of the D value.  

Based on these initial assuptions I have started a few tests. It turns out that the following parameterset leads to smooth results for a throttle value of 0.6:
```
Kp = -0.1
Ki = 0
Kd = -1.0
```
This imolies the following logic: 

```
React smoothly on errors with a high magnitude and neglect a systematic error
```

## Speeding Things Up
In order to make things a bit more complicated, I have increased the throttle valve value to 0.9. This implied an increase of the error correction magnitude and a minor adjustment to the reaction. The resulting parameters for higher speeds that were working fine for most of the time are:
```
Kp = -0.08
Ki = 0
Kd = -3.0
```
   



   
