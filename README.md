# Laboratory of Applied Robotics Student Interface


The technical documentation of the whole code base can be found inside docs folder in html and latex format.

```
docs/html/index.html
``` 


Also we can see 
## Image Saver

To do the intrinsic calibration , the steps to be followed are by running the AR_simulator_calibration and AR_pipeline_img_saver. Through this the images are saved from the simulator which contains checkerboard . Result of this function will be intrinsic parameters. To save the image, the function in genericImageListener in the student_interface.cpp was changed acccordingly.

```
Implemented Function:   genericImageListener
Functionality:  To store the images in the config folder
Function Available in:  src/student_interface.cpp
Reference for implementation:  professor_interface.cpp
Result available in:  All stored images can be found in camera_image_captured_gazebo folder.
```

## Intrinsic calibration

The intrinsic calibration was carried out using the tool that was provided during the lecture. There was no change done in the code. Just used the tool and got the intrinsic parameters in the xml format. And then I copied the parameters to the "camera_params.config" available in the config folder of professor interface. 

Used Tool: Calibration tool
Result available in:  camera_params.config


## Image undistortion

Using the distortion coefficients obtained in the previous steps, I need to remove the distorted effect on the image. This is done using the opencv undistort function.

```
Implemented Function:   imageUndistort
Functionality:  To Undistort the image and obtain the distortion coefficients
Function Available in:  src/extrisnicCalib.cpp
Reference for implementation:  professor_interface.cpp and opencv library
```

![Image Undistortion](imgs/results/undistorted.png)



## Extrinsic Calibration

Now after the intrinsic calibration, I did the extrinsic calibration to determine the Rotational and translational matrix. Four points will be chosen in the image plane and then these 4 points will be solved using the solvePnP interface from opencv to solve the extrinsic problem. 
```
Implemented Function:   extrinsicCalib
Functionality:  To find the rotational and translational vector
Function Available in:  src/extrisnicCalib.cpp
Reference for implementation:  professor_interface.cpp and opencv library
Directly Copied functions: mouseCallback() and pickNPoints()
```
##Perspective Projection and Unwarping

Now to have a bird's eye view of the image, where we need to project the 3D objects in a image plane, I carried out Perspective Projection and Unwarping of image. This is again carried out the opencv interfaces. First, findPlaneTransform() has to caarried through which we get a perspective projection matrix, through which we can unwarp the image

```
Implemented Function:   findPlaneTransform() and unwarp()
Functionality:  To get a unwarped image
Function Available in:  src/extrisnicCalib.cpp
Reference for implementation:  professor_interface.cpp and opencv library
```
> Unwarp ground
![Image Unwarp gound](imgs/results/unwarp_ground.png)

> Unwarp Robot
![Image Unwarp robot](imgs/results/unwarp_robot.png)


## Process Map
After calibration, we need to process map is an important function for further navigation steps. It is decided that obstacles will be Red color with different shapes, gate as green rectangle and victims as green circles. 

### Obstacles detection- Red shapes

```
Implemented Function:  processObstacles()
Functionality: To get all obstacles information
Function Available in:  src/processMap.cpp
Reference for implementation:  professor_interface.cpp, demo code and opencv library
```

> Flow diagram of obstacle detection
![Obstacles Flow diagram](imgs/blocks/processObstacles.jpg)

> Obstacle detection Output
![Obstacles Result](imgs/results/processObstacles.png)



### Gate detection- Green Rectangle

```    
Implemented Function:  processGate()
Functionality:  To get all gate/Destination information
Function Available in:  src/processMap.cpp
Reference for implementation:  professor_interface.cpp, demo code and opencv library
```
> Flow diagram of Gate detection
![Gate Flow diagram](imgs/blocks/processGate.jpg)

> Gate detection Output
![gates Result](imgs/results/processGate.png)

### Victim Detection - Green Circles

```
Implemented Function:  processVictims()
Functionality: To get all victims location
Function Available in:  src/processMap.cpp
Reference for implementation:  professor_interface.cpp, demo code and opencv library
```

> Flow diagram of Victims detection
![Victims Flow diagram](imgs/blocks/processVictims.jpg)

> Victims detection Output
![Victims Result](imgs/results/processVictims.png)


### Victim ID detection:

    This is mainly done to detect the number of Victims, so that the priority to save the victims can be known to the planning algorithm. This involves template matching majorly and the templates were provided in lecture. I have used majorly the opencv template matching methods. But I also implemented the tesseract-ocr method, although the results were weird from them. None of the digits were recognized properly. 
    
    I found a logic in internet somewhere about the digit recognition including the orientation. To put it in simple words, each template will be rotated by 5 degrees and then they are set to calculate the score and the maximum of all of that will be be finalized as digits. With this logic, the digits are getting recognized.

```
Implemented Function:   get_victim_id()
Functionality:  To get the victim's priority
Function Available in:  src/processMap.cpp
Reference for implementation:  professor_interface.cpp, demo code and opencv library 
```
> Flow diagram of Vicitm ID recognition
![Victims recognition diagram](imgs/blocks/getVictimID.jpg)

> Vicitm ID recognition Sample Output(Digit 3 recognized)
![Victims recognition Result](imgs/results/getVictimID.png)


## Find Robot:
    I directly utilized the function provided by the teaching assistant as I found that implementation was already in the best shape.

```
Implemented Function:   findRobot()
Functionality:  To get the robot location
Function Available in:  src/findRobot.cpp
```

> Flow diagram of Robot Detection
![Robot detection diagram](imgs/blocks/findRobot.jpg)

> Robot detection  Output
![Robot detection Result](imgs/results/findRobot.png)


