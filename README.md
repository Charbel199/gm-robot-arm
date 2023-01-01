
<p align="center">
  <img src="docs/gm-robot-arm.png" height="400">
</p>

--------------------------------------------------------------------

# GM Robot Arm

We are developing a robot arm system that is capable of playing chess. This is just a fun project meant to help us learn new thing.
Here is the simple action flow of the robot arm:
- Take a snapshot of the current chess-board layout using a top-down camera
- Deduce the previous made by the human by comparing the previous and current chessboard layouts using computer vision
- Choose the next best move using a chess engine
- Perform this move through autonomous control of the robot arm


Here is a [video demo](https://www.youtube.com/watch?v=iJfm-6d8eYs) showcasing our robot arm playing a full chess game (And winning).


## Installation

```bash
git clone https://github.com/Charbel199/gm-robot-arm
cd gm-robot-arm
pip install -r ./docker/requirements.txt # Will be later replaced with the actual docker container
```

### Azure Kinect Camera Setup
**Steps to use the azure kinect camera through docker**(_Note: Using the depth camera in docker requires OpenGL which we did not implement yet_)
1) Navigate to the AzureKinect directory
2) Run `. DockerBuild.bash`
3) Wait for the docker image to build 
4) Navigate back to the root of the repo
5) Run `. CameraRun.bash`
6) Using ROS, subscribe to the topic /rgb/image_raw for a BGRA format image 


## How to run it

Run each of the following commands in a separate terminal:
```bash
. CameraRun.bash
python3 ./cores/control_core.py
python3 main.py
```

There are some environment variables which can be set before running `main.py`:

- **USE_CAMERA**: this variable specifies if we want to use the camera or pre-existing images on the system (useful when the camera is not connected)
- **IS_SIMULATION**: this variable allows us to run the robot in simulation mode, where it simply follows a set of predetermined moves instead of evaluating the state of the board and playing the move deduced by the engine.
- **USE_ROBOT**: this variable allows us to run the system without actually using the robotic arm, which is valuable when testing other aspects of the system as the arm is time consuming.
- **WITH_SOUND**: allows the user to mute the clock sound if desired.
- **USE_PREVIOUS_CALIBRATED_BOARD**: this variable allows us to surpass the calibration phase in the case where we are playing in an environment where calibration has already taken place.
- **ENGINE_SIDE**: allows us to choose if the robot is playing as white or black.
- **ELO**: this variable determines what elo the engine will be using (valuable if the user is a beginner and would like to play against someone of the same skill)

To properly use the engine, the following sequence takes place in the gm core:

- Press ‘e’ to calibrate the board (this can be bypassed if the USE_PREVIOUS_CALIBRATED_BOARD environment variable is true). The gm core will then instruct the vision core to calibrate the board based on the current image. This must be done on an empty chess board.
- Press ‘i’ to take a snapshot of the initial board layout. This is important for the vision core to have a base image to start comparing to and figure out what moves were done. Make sure you have placed all the pieces before you press ‘i’.
- Start playing, and press ‘space’ when you complete your move.
  - The robot will make sure that it is the user's turn and that the previous steps (calibration and initialization) are complete.
  - It will launch the chess clock.
  - The gm core instructs the vision core to capture the current image.
  - It then instructs it to compare the previous board image to the current board image and determine which squares changed.
  - It then asks the chess core to determine which move was executed based on which squares changed.
  - It will update the virtual board state.
  - It then switches turn to the robots turn.
- Once it is the robots turn:
  - The gm core retrieves the best move from the chess core based on the current board state.
  - It then asks the chess core to dissect the best move into a sequence of moves that the control core can execute (PICKs, PLACEs, and YEETs)
  - The gm core then sets the control move counter based on how many control moves need to be completed for the entire move to be completed.
  - It then publishes all the moves on the topic that the control core is subscribed to.
  - The gm core waits until the move is fully executed (move counter is 0)
  - The gm core instructs the vision core to capture the current image.
  - It then instructs the chess core to update the virtual board.
  - Finally, it switches back to the user’s turn.


--------------------------------------------------------------------

## General architecture

![](docs/gm-robot-arm.diagram.svg)

Here is a communication diagram showing how the various components communicate with each other:

![](docs/gm-robot-arm.communication.png)

## Software Components

- Stockfish
- ROS
- OpenCV

## Hardware Components

- [6 DOF Robot arm (LeArm)](https://www.hiwonder.com/store/learn/2.html)
- Azure Kinect camera
- Arduino Mega 2560

_Note: The robotic arm used is intended for educational purposes, therefore, its servos are not the most accurate.
It would obviously have been better to use a more precise and expensive arm for better results._

_Note 2: The Azure kinect camera is way overkill for this project, but it was the camera we had at hand._

## Chess pieces

There chess pieces were designed on SOLIDWORK, here is a picture of the bishop 3d model:

![](docs/bishop.png)

## Developers

- [Charbel Bou Maroun](https://github.com/Charbel199)
- [Georges Daou](https://github.com/George1044)
- [Ralph Sakhat](https://github.com/Ralph-S)

## Acknowledgements

We would like to thank [3D Maker shop](https://www.3dmakershop.com/) for sponsoring our project and helping us print all the chess pieces that we used.

