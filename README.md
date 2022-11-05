# GM Robot Arm

We are developing a robot arm that is capable of playing chess.
The main flow of this robot arm would be:
    - Take a snapshot of the current chess-board layout using a top-down camera
    - Deduce the previous made by the human by comparing the previous and current layout using computer vision
    - Choose the next best move using a chess engine
    - Perform this move through autonomous control of the robot arm

## Software Componens

- Stockfish
- ROS
- OpenCV

### Stockfish

We are using the Stockfish chess engine since it's open source and has a variety of functionalities.

## Hardware Components

- 6 DOF Robot arm (LeArm)
- Azure Kinect camera
- Arduino Mega 2560

## Requirements


## How to run


## Developers

- [Charbel Bou Maroun](https://github.com/Charbel199)
- [Georges Daou](https://github.com/George1044)
- [Ralph Sakhat](https://github.com/Ralph-S)


