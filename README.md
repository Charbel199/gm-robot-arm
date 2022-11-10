# GM Robot Arm

We are developing a robot arm that is capable of playing chess.
The main flow of this robot arm would be:
    - Take a snapshot of the current chess-board layout using a top-down camera
    - Deduce the previous made by the human by comparing the previous and current chessboard layout using computer vision
    - Choose the next best move using a chess engine
    - Perform this move through autonomous control of the robot arm

## General architecture

![](docs/gm-robot-arm.first_diagram.svg)


## Software Components

- Stockfish
- ROS
- OpenCV

### Stockfish

We are using the Stockfish chess engine since it's open source and has a variety of functionalities.

## Hardware Components

- 6 DOF Robot arm (LeArm)
- Azure Kinect camera
- Arduino Mega 2560

### 6 DOF Robot arm (LeArm)

This robotic arm requires a power supply of 7.5V and 3A for proper operation. It ships with a controller which can be connected to a custom mobile
application to test the servos.

### Azure Kinect Camera
**Steps to use the azure kinect camera through docker**(_Note: Using the depth camera in docker requires OpenGL which we did not implement yet_)
1) Navigate to the AzureKinect directory
2) Run `. DockerBuild.bash`
3) Wait for the docker image to build 
4) Navigate back to the root of the repo
5) Run `. CameraRun.bash`
6) Using ROS, subscribe to the topic /rgb/image_raw for a BGRA format image 


## Requirements


## How to run


## Developers

- [Charbel Bou Maroun](https://github.com/Charbel199)
- [Georges Daou](https://github.com/George1044)
- [Ralph Sakhat](https://github.com/Ralph-S)


