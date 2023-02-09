import os


def roslaunch(package, file, args=""):
    os.system("gnome-terminal -- roslaunch "+package+" "+file+" "+args)

def rosrun(package, file, args=""):
    os.system("gnome-terminal -- rosrun "+package+" "+file+" "+args)

# Getting the names (and therefore the number) of drones available in the AirSim simulation
print("Insert the names of drones (as defined in the AirSim settings) separated by spaces.")

drones_names = list(map(lambda name : name.strip(), filter(None, input().split(" "))))
# Spawining the pid controllers
for drone_name in drones_names :
    roslaunch("drone_coverage", "pid_drone_controller.launch", "drone_name:="+drone_name)

relay_names = drones_names.copy()
relay_names.insert(0, "base_station")
relay_names.append("")
# Spawning the relay nodes
for i in range(1, len(drones_names)+1):
    args = "prev_name:="+relay_names[i-1]+" self_name:="+relay_names[i]+" next_name:="+relay_names[i+1]
    roslaunch("drone_coverage", "relay_node.launch", args)

# Spawning the drone coverage algorithm
roslaunch("drone_coverage", "drone_coverage.launch", "drones_names:=["+",".join(drones_names)+"]")

# Spawing the graph laoder
rosrun("graph_loader", "graph_loader.py")