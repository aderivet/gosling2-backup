#!/bin/bash

ros2 topic pub /planning/scenario_planning/scenario tier4_planning_msgs/msg/Scenario "{current_scenario: Parking, activating_scenarios: [Parking]}"

exit 0
