#!/bin/bash
# can_reset.sh for ModelSolution
echo "Reset CAN BUS Setting"
sudo modprobe peak_usb
sleep 0.5s
sudo ip link set can0 down
sleep 0.5s
sudo ip link set can0 up type can bitrate 500000  # Initialize NIC
sleep 1s
cansend can0 000#8100   # Reset CANOpen Node
echo "Waiting 5s..."
sleep 5s
cansend can0 000#0100   # Start CANOpen Command
echo "Operational State"
