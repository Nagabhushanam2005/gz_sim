#!/bin/bash

rm -f simulation_log.txt

gz sim -v4 -s -r "waves copy 2.sdf" > simulation_log.txt &
SIM_PID=$!
gz sim -v4 -g &
SIM_PID_2=$!

# Run the logger
./build/listener &
LOGGER_PID=$!

if [[ "$1" == "-pid" ]]; then
    ./build/thrust &
    THRUST_PID=$!
fi

# Wait for 50 seconds
sleep 50

kill $SIM_PID $LOGGER_PID $SIM_PID_2
if [[ "$1" == "-pid" ]]; then
    kill $THRUST_PID
fi

python3 plot_data.py