This repository is a demonstration of ROS 1 (Melodic and Noetic) to ROS 2 (Foxy -> Humble) multi-machine communication using Docker. The repository contains a demo for running [F1tenth](https://github.com/privvyledge/autodriver.f1tenth.git) teleoperation with sensor data remote visualization. The repository is a part of the [AutoDriver](https://github.com/privvyledge/autodriver.git) project. The F1tenth docker image starts the ROS2 Humble nodes, while the autonomous commands are sent from a ROS 1 node.

## Requirements
- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)
- ROS 1 (Melodic or Noetic). Tested with Melodic and Noetic but other versions may work.
- ROS 2 (Foxy or Humble). Tested with Foxy and Humble but other versions may work.

## Quickstart
1. Clone this repository: `git clone https://github.com/privvyledge/ros1_to_ros2_communication.git`
2. Navigate to the repository: `cd ros1_to_ros2_communication/docker`
3. Run the docker image: `docker-compose up`

