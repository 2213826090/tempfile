#!/bin/bash
# Scripts that disconnects the DUT and waits for the device reconnection
adb disconnect
adb wait-for-device &
wait_pid=$!
sleep 3
adb connect 192.168.42.1

wait "$wait_pid"
wait_status=$?

if [ wait_status ]; then
    echo "PASS"
else
    echo "FAIL"
fi