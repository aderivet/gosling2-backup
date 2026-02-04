This document details the process of setting one (or more) of the F1/10 autonomous vehicle as well as summarizes the components involved. By the end, any user, even without prior experience should have a mid to high level understanding of the platform and how to use them.

# System Overview

<figure>
<img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image1.jpeg" style="width:2.58333in;height:1.34466in" alt="A car with a blue and black object on top Description automatically generated" />
<figcaption><p>: F1/10 Platform</p></figcaption>
</figure>

The F1/10 platform is a scaled modular Ackermann vehicular robot with the sensors and actuators needed for autonomous vehicle research and experimentation in areas of perception, localization, and controls developed by the Autonomous Systems Group within RASLAB at the FAMU-FSU College of Engineering.

<figure>
<img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image2.png" style="width:6.33333in;height:4.01111in" alt="A diagram of a car Description automatically generated" />
<figcaption><p>: System Architecture</p></figcaption>
</figure>

# Hardware Components (v1)

<table>
<colgroup>
<col style="width: 50%" />
<col style="width: 50%" />
</colgroup>
<thead>
<tr>
<th><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image3.png" style="width:2.68372in;height:1.50481in" alt="A close-up of a camera Description automatically generated" /></p>
<p>Figure : Intel Realsense D435i (Stereo Camera)</p></th>
<th><ul>
<li><p>1x RGB Visual Camera @ 30FPS</p></li>
<li><p>2x Infrared Cameras @ 30FPS</p></li>
<li><p>1x IMU @ 200Hz</p></li>
<li><p>Depth Image from Stereo (infrared cameras) @ 30FPS</p></li>
<li><p>Pointcloud from depth with optional color up to 4m @ 30FPS</p></li>
</ul></th>
</tr>
</thead>
<tbody>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image4.png" style="width:2.31736in;height:1.73333in" alt="A small blue and black device Description automatically generated" /></p>
<p>: YDLIDAR X4 (LaserScanner)</p></td>
<td><ul>
<li><p>2D Pointcloud up to 12m @ 8Hz</p></li>
</ul></td>
</tr>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image5.png" style="width:1.90751in;height:1.475in" alt="A black electronic device with a fan Description automatically generated" /></p>
<p>Figure 5: Nvidia Jetson Orin Nano (Embedded Computer)</p></td>
<td><ul>
<li><p>CPU: 6-core Arm® Cortex®-A78AE</p></li>
<li><p>GPU: NVIDIA Ampere architecture with 1024 CUDA cores and 32 tensor cores</p></li>
<li><p>RAM: 8GB 128-bit LPDDR5</p></li>
<li><p>Storage: 1TB NVMe SSD</p></li>
<li><p>Power consumption: 15W</p></li>
</ul></td>
</tr>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image6.jpeg" style="width:2.68333in;height:1.72489in" alt="A white and black video game controller Description automatically generated" /></p>
<p>Figure : Playstation 5 DualSense Controller</p></td>
<td><ul>
<li><p>Bluetooth controller</p></li>
</ul></td>
</tr>
</tbody>
</table>

<table>
<colgroup>
<col style="width: 54%" />
<col style="width: 45%" />
</colgroup>
<thead>
<tr>
<th><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image7.png" style="width:3.36089in;height:1.58065in" alt="A red electronic device with black wires Description automatically generated" /></p>
<p>: VESC 6 Mk VI (Actuator)</p></th>
<th><ul>
<li><p>Brushless motor ESC</p></li>
<li><p>Voltage: 11.1v (3S LiPo battery)</p></li>
<li><p>Current: 80A continuous (120 A max)</p></li>
<li><p>1x IMU</p></li>
<li><p>Actuator feedback</p></li>
</ul></th>
</tr>
</thead>
<tbody>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image8.jpg" style="width:2.40833in;height:1.75878in" alt="A close-up of a usb flash drive Description automatically generated" /></p>
<p>: USB Isolator</p></td>
<td><ul>
<li><p>Creates a separate, isolated ground</p></li>
<li><p>Breaks the ground loop caused by connecting two batteries with different grounds (LiPo and Li-ion) when connecting the VESC to the PC</p></li>
</ul></td>
</tr>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image9.png" style="width:1.65in;height:1.61765in" alt="A black rectangular object with a black background Description automatically generated" /></p>
<p>: NP-F750 Battery (Jetson + Sensors)</p></td>
<td><ul>
<li><p>Battery Type: Li-ion</p></li>
<li><p>Voltage: 7.2v (min), 7.7v (nominal), 8.4 (max)</p></li>
<li><p>Capacity: 5600mAh</p></li>
<li><p>Current (charge): 1A</p></li>
</ul></td>
</tr>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image10.png" style="width:2.31282in;height:1.57314in" alt="A black rectangular object with a hole in the center Description automatically generated" /></p>
<p>: NP-F Adapter</p></td>
<td><ul>
<li><p>Voltage: 7.4 v or 12.0v</p></li>
<li><p>Current (discharge): 2A</p></li>
</ul></td>
</tr>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image11.jpeg" style="width:1.775in;height:1.84376in" alt="A black battery charger with green text Description automatically generated" /></p>
<p>: NP-F Battery Charger</p></td>
<td><ul>
<li><p>Input Voltage: 5v</p></li>
<li><p>Output Voltage: 8.4v</p></li>
<li><p>Current (charge): 2A (1A per battery)</p></li>
</ul></td>
</tr>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image12.jpeg" style="width:2.53901in;height:1.47431in" alt="A battery with a wire attached to it Description automatically generated" /></p>
<p>: Venom LiPo Battery (VESC)</p></td>
<td><ul>
<li><p>Battery Type: LiPo</p></li>
<li><p>Voltage (3S LiPo): 9.2v (min), 11.1v (nominal), 12.6v</p></li>
<li><p>Current (discharge): 175A continuous (350A max)</p></li>
<li><p>Current (charge): 5A</p></li>
<li><p>Capacity: 5000mAh</p></li>
</ul></td>
</tr>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image13.jpg" style="width:2.83333in;height:2.21036in" alt="A blue electronic device with buttons and labels Description automatically generated" /></p>
<p>: Traxxas EZ Peak Plus Dual Charger</p></td>
<td><ul>
<li><p>Voltage (output): 12v (for 3S LiPo)</p></li>
</ul></td>
</tr>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image14.png" style="width:2.875in;height:2.09851in" alt="A close-up of a car Description automatically generated" /></p>
<p>Figure 14: Traxxas 4 Tec 2.0 VXL Chassis</p></td>
<td><ul>
<li><p>Max speed: 70 MPH</p></li>
<li><p>Wheelbase: 0.256m</p></li>
<li><p>Wheel radius: 0.033m</p></li>
<li><p>Overall Drive Ratio: 6.87</p></li>
</ul></td>
</tr>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image15.png" style="width:2.95833in;height:0.976in" alt="A side view of a car Description automatically generated" /></p>
<p>: 2019 Cadillac CTS-V</p></td>
<td><ul>
<li><p>Plastic body shell</p></li>
</ul></td>
</tr>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image16.png" style="width:2.69978in;height:1.775in" alt="A metal frame with a black background Description automatically generated" /></p>
<p>: Rollcage</p></td>
<td><ul>
<li><p>Made from Nylon-12</p></li>
</ul></td>
</tr>
</tbody>
</table>

# Software Requirements:

- Operating system: Ubuntu 22.04 (Codename: Jammy Jellyfish)

- Python version: 3.10

- CUDA version: 12.2

- CuDNN version: 8.9.4.25

- TensorRT version: 8.6.2.3

- ROS Version: 2

- ROS Distro: Humble

- Packages:

  - C++ Drivers:

| **Package** | **Version/Branch** | **Link to Repository** |
|----|----|----|
| Librealsense SDK | v2.55.1 | [GitHub](https://github.com/IntelRealSense/librealsense) |
| YDLIDAR SDK | Master | [GitHub](https://github.com/YDLIDAR/LIDAR_SDK) |
| VESC Driver | 1952e79 | [GitHub](https://github.com/privvyledge/f1tenth_system) |

- Python packages:

| **Package** | **Version/Branch** | **Purpose** |
|----|----|----|
| Numpy | 1.21.4 | Array manipulation |
| Acados | 0.1 | Real-time Optimization |
| Casadi | 3.6.5 | Real-time Optimization |
| Do-MPC | 4.6.5 | Optimization Prototyping |
| OpenCV | 4.5.0 | Image Processing |
| Open3D | 0.18.0 | Pointcloud and RGB-D processing and perception |
| Torch | 2.3.0 | CPU/GPU Tensor manipulation |
| Ultralytics | 8.3.39 | Image-based object detection |

- ROS Packages

| **Package** | **Purpose** |
|----|----|
| Realsense ROS Wrapper | Publish realsense data as ROS messages. |
| YDLIDAR ROS Driver | Publish LaserScans as ROS messages |
| Image transport | Image compression/decompression |
| PointCloud Transport | PointCloud compression/decompression |
| Rviz2 | Visualization |
| Image pipeline | Image processing |
| IMU filters | IMU orientation estimation |
| Robot localization | Extended Kalman Filter based sensor fusion |
| F1tenth_launch | Launches all nodes with defaults for autonomous driving. |

# Hardware Connection Setup (Sensor Assembly)

| Step 1: Get the base plate. | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image17.jpg" style="width:2.65in;height:0.53858in" alt="A grey rectangular object with a white background Description automatically generated" /> |
|----|----|
| Step 2: Mount the computer on the baseplate via the four screws on the outer edge with the DC jack and USB facing the rear of the car. | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image18.jpg" style="width:2.63681in;height:0.79199in" alt="A computer chip with a white and black rectangular object Description automatically generated with medium confidence" /> |
| Step 3: Mount the VESC upside down under the baseplate. | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image19.jpg" style="width:3.09167in;height:1.03616in" alt="A computer chip with different colors Description automatically generated with medium confidence" /> |
| Step 4: Attach the rollcage to the baseplate. | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image20.jpg" style="width:3.09167in;height:1.26477in" alt="A computer generated machine with a green and red base Description automatically generated with medium confidence" /> |
| Step 5: Mount the LIDAR to the top-center of the rollcage. | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image21.jpg" style="width:2.925in;height:1.84265in" alt="A computer chip with a blue cap Description automatically generated" /> |
| Step 6: Mount the camera to the top-front of the rollcage. | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image22.jpg" style="width:2.84167in;height:1.75937in" alt="A computer chip with a blue cap Description automatically generated with medium confidence" /> |
| Step 7: Screw in the NP-F battery adapter to the inclined rear part of the rollcage. Then slide in the battery to the adapter. | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image23.png" style="width:2.56667in;height:1.2167in" alt="A close-up of a machine Description automatically generated" /> |

The resulting sensor assembly should be like the photos below:

<table>
<colgroup>
<col style="width: 47%" />
<col style="width: 52%" />
</colgroup>
<thead>
<tr>
<th><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image24.jpeg" style="width:2.42056in;height:2.24583in" alt="A close-up of a machine Description automatically generated" /></p>
<p>: VESC Assembly</p></th>
<th><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image25.jpeg" style="width:2.77917in;height:2.16468in" alt="A close-up of a device Description automatically generated" /></p>
<p>: Sensor and PC Assembly</p></th>
</tr>
</thead>
<tbody>
</tbody>
</table>

# Hardware Connection Setup (Body Assembly)

| Step 1: Attach the base plate to the chassis on the standoffs. | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image26.jpg" style="width:3.00833in;height:1.5171in" alt="A close-up of a machine Description automatically generated" /> |
|----|----|
| Step 2: Attach the shell to the chassis | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image27.png" style="width:3.21667in;height:3.41565in" alt="A close up of a machine Description automatically generated" /> |

# Wiring (Power) Connection Sequence

# 

| Step 1: Remove the shell from the chassis. | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image28.png" style="width:2.65833in;height:5.64787in" alt="A blue round object with wires Description automatically generated" /> |
|----|----|
| Step 2: Connect the powerbank and its dc barrel connector to the jetson | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image29.png" style="width:3.51667in;height:1.58363in" alt="A close up of a machine Description automatically generated" /> |
| Step 3: Place the battery on the chassis | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image30.png" style="width:1.43808in;height:2.75111in" alt="A battery in a car Description automatically generated with medium confidence" /> |
| Step 4: Connect the VESC to the battery using the XT-90 connectors | <img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image31.png" style="width:3.38333in;height:1.66745in" alt="A close up of a device Description automatically generated" /> |
| Step 5 (optional): Attach the shell to the chassis |  |

<figure>
<img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image32.png" style="width:6.5in;height:2.95115in" />
<figcaption><p>: Wiring</p></figcaption>
</figure>

# Network Configuration

| **Computer Name** | **Network Hostname** | **IP Address** | **Network Interface** | **RustDesk ID(Password)** |
|----|----|----|----|----|
| *Remote PC*: Linux Workstation | \<remote PC hostname\> | 192.168.2.xxx | WiFi: \<WiFi name\> | \<rustdesk ID\> : \<rustdesk password\> |
| *F1/10***:** car1 | \<car1 hostname\> | 192.168.2.xxx | WiFi: \<WiFi name\> | \<rustdesk ID\> : \<rustdesk password\> |

## Network Connection Steps:

1.  (optional: for visualization purposes or multi-machine ROS2 communication) Remote PC:

    1.  Connect the Remote PC to the WiFi and enter the password (UVS_wifi)

    2.  Install RustDesk

    3.  Setup a permanent password in the RustDesk settings

    4.  Open RustDesk

    5.  Enter the RustDesk credentials for the F1/10 in the table above

2.  On any F1/10

    1.  Connect the F1/10 to the WiFi and enter the password.

    2.  Install RustDesk

    3.  Setup a permanent password in the RustDesk settings

# DualSense\* Controller Setup

<table>
<colgroup>
<col style="width: 53%" />
<col style="width: 46%" />
</colgroup>
<thead>
<tr>
<th><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image33.png" style="width:2.88333in;height:2.32751in" alt="Controller front view. Clockwise from the top left A to N." /></p>
<p>Figure 20: DualSense Controller (Front)</p>
<p><em>Source : <a href="https://controller.dl.playstation.net/controller/lang/en/2100002.html">https://controller.dl.playstation.net/controller/lang/en/2100002.html</a></em></p></th>
<th><ol type="A">
<li><p>Directional Buttons</p></li>
<li><p>Create/Share button</p></li>
<li><p>Light bar</p></li>
<li><p>Touch pad/button</p></li>
<li><p>Player Indicator</p></li>
<li><p>Options button</p></li>
<li><p>Action buttons</p></li>
<li><p>Right stick/R3</p></li>
<li><p>Speaker</p></li>
<li><p>PS Button</p></li>
<li><p>Headset Jack</p></li>
<li><p>Microphone</p></li>
<li><p>Mute button</p></li>
<li><p>Left stick/L3</p></li>
</ol></th>
</tr>
</thead>
<tbody>
<tr>
<td><p><img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image34.png" style="width:3.3in;height:2.18603in" alt="Controller top view. From the left A to E." /></p>
<p>: Dualsense Controller (Top)</p>
<p><em>Source: <a href="https://controller.dl.playstation.net/controller/lang/en/2100002.html">https://controller.dl.playstation.net/controller/lang/en/2100002.html</a></em></p></td>
<td><ol type="A">
<li><p>R1 button</p></li>
<li><p>R2 button</p></li>
<li><p>USB port</p></li>
<li><p>L1 Button</p></li>
<li><p>L2 Button</p></li>
</ol></td>
</tr>
</tbody>
</table>

\* *Other controllers tested with the exact configuration and performance: DualShock 4, Logitech F710*

## Pairing Instructions:

1.  Press and hold the PS button (J) and the Create/Share (B) button at the same time until the Light Bar (C) blinks continuously.

2.  Connect to the Jetson via RustDesk as detailed above

3.  Open Bluetooth settings and connect to the DualSense Wireless Controller

4.  The light bar (C) and Player Indicator (E) should remain solid when paired.

<figure>
<img src="./attachments/F1tenthDocumentation_v1_1_Release/media/image35.png" style="width:3.65833in;height:1.91515in" alt="A white video game controller with blue arrows Description automatically generated" />
<figcaption><p>: Control Scheme</p></figcaption>
</figure>

# Software Setup

1.  Install ROS 2 Humble: Follow the official installation instructions [found here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

2.  Configure the environment:

<table>
<colgroup>
<col style="width: 29%" />
<col style="width: 70%" />
</colgroup>
<thead>
<tr>
<th><ul>
<li><p>Automatically load the path to ROS2 packages when a new terminal is opened.</p></li>
</ul></th>
<th><ul>
<li><p>echo "source /opt/ros/humble/setup.bash" &gt;&gt; ~/.bashrc</p></li>
</ul></th>
</tr>
</thead>
<tbody>
<tr>
<td><ul>
<li><p>Set CYCLONE_DDS as the default RMW implementation</p></li>
</ul></td>
<td><ol type="1">
<li><p>sudo apt update &amp;&amp; sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp</p></li>
<li><p>echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" &gt;&gt; ~/.bashrc</p></li>
</ol></td>
</tr>
<tr>
<td><ul>
<li><p>Create the ros2_ws</p></li>
</ul></td>
<td><ul>
<li><p>mkdir -p ~/ros2_ws/src &amp;&amp; colcon build –symlink-install</p></li>
</ul></td>
</tr>
</tbody>
</table>

3.  Clone the necessary packages (listed in Software Requirements section above) to the ros2_ws and recompile.

## Launch Teleoperation Nodes (For a detailed and comprehensive explanation of every launch argument, check the comments in the launch file [**here**](https://github.com/privvyledge/f1tenth_launch/blob/humble-dev/launch/teleop.launch.py).)

1.  Launch the teleop nodes with the following commands: ros2 launch f1tenth_launch teleop.launch.py launch_vehicle:=True launch_sensors:=False launch_localization:=False vesc_poll_rate:=50.0.

2.  Nodes launched:

<table>
<colgroup>
<col style="width: 50%" />
<col style="width: 49%" />
</colgroup>
<thead>
<tr>
<th><ul>
<li><p>VESC ROS Driver</p></li>
</ul></th>
<th><ul>
<li><p>Steering</p></li>
<li><p>Motor</p></li>
<li><p>IMU</p></li>
</ul></th>
</tr>
</thead>
<tbody>
<tr>
<td><ul>
<li><p>Joystick Driver (DualSense)</p></li>
</ul></td>
<td><ul>
<li><p>Publishes joystick inputs as ROS messages</p></li>
</ul></td>
</tr>
<tr>
<td><ul>
<li><p>Joy to Ackermann Node</p></li>
</ul></td>
<td><ul>
<li><p>Converts joystick messages to car speed and steering commands.</p></li>
</ul></td>
</tr>
<tr>
<td><ul>
<li><p>Ackermann to VESC node</p></li>
</ul></td>
<td><ul>
<li><p>Converts longitudinal speed and steering commands to motor RPM and steering PWM commands.</p></li>
</ul></td>
</tr>
<tr>
<td><ul>
<li><p>VESC to Ackermann node</p></li>
</ul></td>
<td><ul>
<li><p>Publishes kinematic wheel odometry node</p></li>
</ul></td>
</tr>
</tbody>
</table>

3.  Parameters:

<table>
<colgroup>
<col style="width: 50%" />
<col style="width: 49%" />
</colgroup>
<thead>
<tr>
<th><ul>
<li><p>launch_vehicle</p></li>
</ul></th>
<th><ul>
<li><p>Whether to turn on the VESC. Useful for testing only sensors or replaying recorded data.</p></li>
</ul></th>
</tr>
</thead>
<tbody>
<tr>
<td><ul>
<li><p>vesc_poll_rate</p></li>
</ul></td>
<td><ul>
<li><p>Rate at which to communicate with the VESC to send commands and receive feedback. Higher frequencies lead to more sensor noise.</p></li>
</ul></td>
</tr>
<tr>
<td><ul>
<li><p>launch_sensors</p></li>
</ul></td>
<td><ul>
<li><p>Whether to turn on all sensors.</p></li>
</ul></td>
</tr>
<tr>
<td><ul>
<li><p>launch_localization</p></li>
</ul></td>
<td><ul>
<li><p>Whether to launch local and global localization nodes.</p></li>
</ul></td>
</tr>
</tbody>
</table>

## Driving instructions:

1.  Hold down the L1 button to arm the motors (Note: the car will not move if L1 is released)

2.  Move the left analog stick up and down to drive/reverse. (Note: move slowly otherwise the car will accelerate rapidly).

3.  Move the right analog stick left and right to steer the car

*NOTES: The provided scripts launch visualization and sensor compression/decompression nodes as well as enabling sensors.*

# Battery Management

#### LiPo Batteries (e.g Venom 3S LiPo)

1.  Keep the batteries at storage voltage of 11.3v when not in use.

2.  Charge the battery using the Traxxas EZ Peak Plus Dual Charger in advance mode. See official instructions [here](https://traxxas.com/sites/default/files/HKC18030-R05_2972X-TraxxasID-Dual%20Charger-INST-EN.pdf) (page 3).

3.  The VESC prevents the battery from being drained below safe voltages as well as configurable max charge/discharge current limits.

#### DualSense Battery: 

1.  The device includes an in-built Battery Management System which includes under-voltage and over-voltage protection.

2.  The device automatically shuts off when the battery is low.

3.  Recharge via USB.

#### NP-F Battery:

1.  The NP-F battery adapter prevents the battery from being drained below safe voltages.

2.  Charge via the NP-F chargers shown above.

# Operating Procedures

## Pre-operation checklist

1.  Inspect the batteries

2.  Charge the batteries (ESC, PC and controller) to full voltage.

3.  Ensure all cables are connected and there are no shorts.

## Startup sequence: 

### Power-up sequence

1.  Connect the DC jack to the Jetson to turn it on.

2.  Connect the sensors to the jetson.

3.  Connect the LiPo battery to the VESC.

4.  Connect the micro-USB from the VESC to the USB isolator

5.  Connect the USB isolator to the Jetson

6.  Connect the sensors to the Jetson via USB

### Connection establishment

1.  The jetsons power status LED should be solid green.

2.  The VESCs status LED should be solid red.

3.  The USB isolators LED should be solid red.

### Normal Operation Procedures:

- Actuators (steering and throttle) should respond when armed.

- The LIDAR should start spinning (if sensors are activated).

### Shutdown sequence

1.  Press CTRL-C on the terminal used to launch the nodes to cancel all running nodes (this could take about 4 seconds to safely shutdown the actuators and sensors).

2.  Shut down the PC: sudo shutdown now.

3.  Disconnect all batteries, especially the VESC.

### Emergency procedures

1.  Elevate the car off the ground.

2.  Disconnect the VESC from the battery.

3.  Unplug the DC barrel jack from the Jetson.

# Troubleshooting Guide

**Network Connectivity Issues**

<table>
<colgroup>
<col style="width: 50%" />
<col style="width: 50%" />
</colgroup>
<thead>
<tr>
<th><strong>Issue</strong></th>
<th><strong>Solution</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td><ul>
<li><p>ROS topics not visible across machines</p></li>
</ul></td>
<td><ul>
<li><p>Make sure both machines are connected to the same network (not necessary for RustDesk)</p></li>
<li><p>Ping the IP address of machine B from machine A, i.e “ping 192.168.2.194” from Remote PC</p></li>
<li><p>Ensure the terminal environment variable $ROS_LOCALHOST_ONLY=0</p></li>
<li><p>Ensure the terminal environment variable $ROS_DOMAIN_ID is either unset on both machines or equal</p></li>
</ul></td>
</tr>
<tr>
<td><ul>
<li><p>ROS topic rates too low on remote PC</p></li>
</ul></td>
<td><ul>
<li><p>For high bandwidth messages like PointClouds or images, use compressed messages as detailed below.</p></li>
<li><p>Reduce the number of devices connected to the WiFi router</p></li>
<li><p>Make sure the devices are within the range of the WiFi router</p></li>
</ul></td>
</tr>
<tr>
<td><ul>
<li><p>RustDesk (or SSH) disconnects</p></li>
</ul></td>
<td><ul>
<li><p>Occurs when CPU utilization is at 100% which usually occurs if Rviz is running on the Jetson while running teleoperation with sensing and other nodes. Run Rviz on the remote PC instead.</p></li>
</ul></td>
</tr>
</tbody>
</table>

**Controller Connectivity Issues**

<table>
<colgroup>
<col style="width: 37%" />
<col style="width: 36%" />
<col style="width: 26%" />
</colgroup>
<thead>
<tr>
<th><strong>Issue</strong></th>
<th><strong>Explanation</strong></th>
<th><strong>Solution</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td><ul>
<li><p>Controller keeps disconnecting (or does not connect)</p></li>
</ul></td>
<td><ul>
<li><p>Due to the cheap Network Interface Card included in the Jetson, WiFi and Bluetooth connections can be unstable.</p></li>
</ul></td>
<td><ul>
<li><p>For now, the only solution is to retry until successful.</p></li>
<li><p>(Future) replace with better NIC or adapters.</p></li>
</ul></td>
</tr>
</tbody>
</table>

**Power Issues**

<table>
<colgroup>
<col style="width: 35%" />
<col style="width: 39%" />
<col style="width: 25%" />
</colgroup>
<thead>
<tr>
<th><strong>Issue</strong></th>
<th><strong>Explanation</strong></th>
<th><strong>Solution</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td><ul>
<li><p>LiPo Battery connected to VESC drained below safe voltage.</p></li>
</ul></td>
<td><ul>
<li><p>Due to hardware/software bugs in the VESC 6 VI, the auto-disconnect function does not work and the VESC keeps draining the battery (although at low rates).</p></li>
</ul></td>
<td><ul>
<li><p>Disconnect the battery from the VESC after each use.</p></li>
<li><p>Use a Power Distribution Unit.</p></li>
<li><p>Upgrade to VESC 6 HP.</p></li>
</ul></td>
</tr>
<tr>
<td><ul>
<li><p>Jetson node rates lower than expected.</p></li>
</ul></td>
<td><ul>
<li><p>Due to the limited power output of the NP-F battery and adapter which is incapable of supplying more than half the power required by the Jetson, the Jetsons performance is throttled.</p></li>
</ul></td>
<td><ul>
<li><p>(Future) replace the NP-F components with a Power Distribution Unit capable of supplying the required power.</p></li>
</ul></td>
</tr>
<tr>
<td><ul>
<li><p>RealSense or LIDAR fails to connect.</p></li>
</ul></td>
<td><ul>
<li><p>Due to the limited output of the Jetsons single USB controller, starting up all sensors at once leads to current limits which temporarily disables some of the USB outputs.</p></li>
</ul></td>
<td><ul>
<li><p>Software delay is used to startup the sensors sequentially.</p></li>
<li><p>Add a powered USB hub.</p></li>
</ul></td>
</tr>
</tbody>
</table>
