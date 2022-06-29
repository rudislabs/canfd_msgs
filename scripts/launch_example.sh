#!/bin/bash
pkill python3
sudo ip link set can0 down
echo "Waiting 1 second to restart bus"
sleep 1
sudo ip link set can0 up type can bitrate 1000000 dbitrate 4000000 loopback off fd on fd-non-iso off restart-ms 100
echo "Waiting 5 seconds for UCAN to come online"
candump can0 &
sleep 5
python3 ROS2CyphalMessageSubscriptionToSocketCANExample.py &
sleep 3
python3 ROS2CyphalMessagePCAExample.py &
sleep 6
pkill candump
