#!/bin/bash

ros2 topic pub /api/localization/initialization_state autoware_adapi_v1_msgs/msg/LocalizationInitializationState "{stamp: now, state: 3}"

exit 0
