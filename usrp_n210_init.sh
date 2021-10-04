#!/bin/bash
#
echo "-- Initialising USRP X300 devices.."

# Search for devices
uhd_find_devices

# Set MTU size
# https://www.cyberciti.biz/faq/how-can-i-setup-the-mtu-for-my-network-interface/
# note "dev <name> mtu" name corresponds to network card name
sudo ip link set dev UHD _link mtu 9202

# Set buffer size
# https://ixnfo.com/en/changing-tx-and-rx-network-interface-buffers-in-linux.html
sudo ethtool -G UHD _link rx 4092 tx 4092

# Netwrok buffer size
sudo sysctl -w net.core.rmem_max=50000000
sudo sysctl -w net.core.wmem_max=50000000

echo "-- Done!"
