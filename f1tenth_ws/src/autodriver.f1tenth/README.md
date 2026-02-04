## Quickstart
1. Purchase materials following the Bill of Materials (BOM) in the `docs` directory.
2. Assemble the car following the assembly instructions in the `docs` directory.
3. Create a ROS2 workspace and navigate to the `src` directory: `mkdir -p ~/f1tenth_ws/src && cd ~/f1tenth_ws/src`
4. Clone this repository: `git clone https://github.com/privvyledge/autodriver.f1tenth.git`
5. Install packages: `cd autodriver.f1tenth && chmod +x scripts/* && ./scripts/install_software_jetson.sh && sudo systemctl restart docker && sudo usermod -aG docker $USER && newgrp docker`
6. Pull the docker image
7. Run teleoperation
   1. Connect the joystick to the Jetson
   2. Connect the VESC to the battery and Jetson
   3. Connect the YDLIDAR to the Jetson
   4. Connect the RealSense to the Jetson