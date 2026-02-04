# f1tenth_launch
<!-- Required -->
<!-- Package description -->
F1TENTH launch files.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->
Check external links for dependencies.

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to f1tenth_autoware_launch_py
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->
1. Run F1TENTH AWSIM simulator.
2. Run launch file:
```bash
ros2 launch f1tenth_autoware_launch_py f1tenth.launch.py map_path:=autoware_map/imola
```

## References / External links
<!-- Optional -->
* [External packages](https://github.com/PPI-PUT/autoware/blob/ppi/f1tenth.repos)
* [Map](https://chmura.put.poznan.pl/s/6yFJgRDeLtiIml7)
* [F1TENTH AWSIM](https://chmura.put.poznan.pl/s/ztLrYaJwZupU667)