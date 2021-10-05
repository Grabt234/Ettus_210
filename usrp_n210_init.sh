#!/bin/bash
#
echo "-- Initialising USRP N210 devices.."

# Search for devices
uhd_find_devices

# NOTE:
# note "dev <name> mtu" name corresponds to network card name

# to find run "$ ip link list" in terminal and try use the altName

# if multiple networ cards check the MAC address 
#                       corresponding to configured card


# Set MTU size
# https://www.cyberciti.biz/faq/how-can-i-setup-the-mtu-for-my-network-interface/
sudo ip link set dev enp0s25 mtu 4092

# Set buffer size
# https://ixnfo.com/en/changing-tx-and-rx-network-interface-buffers-in-linux.html
sudo ethtool -G enp0s25 rx 4092 tx 2048

# Netwrok buffer size
sudo sysctl -w net.core.rmem_max=50000000
sudo sysctl -w net.core.wmem_max=50000000

echo "-- Done!"
