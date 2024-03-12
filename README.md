# Subterrain Challenge - Autonomous Systems

This repository contains the documentation and the source code for the "Subterrain Challenge," part of the Autonomous Systems course at the Technical University of Munich (TUM), taught by Professor Markus Ryll.

## Team 11
Team 11 is composed of the following members:

1. Natale Consalvi
2. Milica Kalanj
3. Claudia Passerini
4. Marcel Ramón Rodríguez
5. Giuseppe Soldati

## Challenge Description
The "Subterrain Challenge" is a project within the Autonomous Systems course.

## Repository Contents
- `State machine` that manages the entire process: hover, testing of the detection algorithm, arrival at the cave entrance, and the beginning of the autonomous exploration part.
- The `light detection` algorithm is implemented within the `perception` package. Colored point clouds are constructed from the depth camera image and mapped using a 2D and subsequently 3D procedure to register the positions of the lamps.
- Within the perception pipeline, lightweight point clouds are also generated at a higher frequency for creating a `voxel grid` with `octomap`.
- For cave exploration, 2 packages were used and adapted to our case:
  - The first is `uav_frontier_detection`, used to read the voxel grid and analyze `frontier points`, generate a cluster, and select the `best frontier`.
  - The second package implements `local and global path planning`. It is capable of `generating a trajectory` that avoids collision of the drone with the cave walls.

## Dependencies and Installation

To install the dependencies, use the `requirements.txt` file located in the `code` folder. Navigate to the `code` folder and run the following command:

`pip install -r requirements.txt`

Ensure that you have the necessary dependencies installed before building the project with catkin build. The source code required for building the project is located in the src folder.

## Running Instructions
This project has been implemented for UBUNTU 20.04.6 and ROS Noetic.


## Contacts
For any questions or issues, feel free to contact Team 11 members.

