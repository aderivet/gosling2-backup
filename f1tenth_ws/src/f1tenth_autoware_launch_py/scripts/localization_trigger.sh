#!/bin/bash

ros2 service call /localization/trigger_node std_srvs/srv/SetBool "{data: true}"

exit 0
