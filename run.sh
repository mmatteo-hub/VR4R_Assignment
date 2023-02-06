#!/bin/sh

# Exit when any command fails
set -e

# The user need to insert the name (and therefore the number) of
# drones that need to be used in the coverage
echo "Insert the name of the drones (spaced by commas)."
read DRONES_NAMES_STRING
# Separating the names by the comma
DRONE_NAMES=$(echo $DRONES_NAMES_STRING | tr "," "\n")

for NAME in $DRONE_NAMES 
do
    # Opening a new terminal with the drone controller
    gnome-terminal -- roslaunch drone_coverage pid_drone_controller.launch drone_name:="$NAME"
done

# Launching the coverage algorithm
gnome-terminal -- roslaunch drone_coverage drone_coverage.launch drones_names:="[$DRONES_NAMES_STRING]"
# Launching the graph loader
gnome-terminal -- rosrun graph_loader graph_loader.py