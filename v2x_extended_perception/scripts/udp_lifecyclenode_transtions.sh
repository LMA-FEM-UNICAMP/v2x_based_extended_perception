#!/bin/bash

ros2 lifecycle set /v2x/driver/udp_receiver 1
ros2 lifecycle set /v2x/driver/udp_sender 1
ros2 lifecycle set /v2x/driver/udp_receiver 3
ros2 lifecycle set /v2x/driver/udp_sender 3