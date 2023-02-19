import os
import rospy

class GnomeTerminalHelper:

    def __init__(self):
        self._tabs = []

    def add_tab(self, name, command):
        self._tabs.append(GnomeTabHelper(name, command))

    def create(self):
        # Creating the command
        command = "gnome-terminal"
        command += " "+self._tabs[0].create(True)
        for i in range(1, len(self._tabs)) :
            command += " "+self._tabs[i].create(False)
        # Executing the command
        os.system(command)


class GnomeTabHelper:
    def __init__(self, name, command):
        self.name = name
        self.command = command
        pass

    def create(self, is_first):
        tab_type = "--window" if is_first else "--tab"
        return tab_type+" -t \""+self.name+"\" -e \""+self.command+"\""


# Getting the names (and therefore the number) of drones available in the AirSim simulation
print("Insert the names of drones (as defined in the AirSim settings) separated by spaces.")

# All and only the drones names
drones_names = list(map(lambda name : name.strip(), filter(None, input().split(" "))))
# Setting the drones names as parameter on the ROS ParameterServer
rospy.set_param("drones_names", "["+",".join(drones_names)+"]")
rospy.set_param("drone_max_vel_horz", 3.0)
rospy.set_param("drone_max_vel_vert", 3.0)
rospy.set_param("drone_max_vel_rotz", 10.0)

# The drones names and base_statetion and last empty node
relay_names = drones_names.copy()
relay_names.insert(0, "base_station")
relay_names.append("")

for relay_name_index in range(1, len(drones_names)+1) :
    relay_name = relay_names[relay_name_index]
    # Spawining a console for each drone
    terminal = GnomeTerminalHelper()
    # Spawning the pid controller
    args = "drone_name:="+relay_name
    terminal.add_tab("","roslaunch drone_coverage drone_pid_controller.launch "+args)
    # Spawining the collision avoidance
    args = "drone_name:="+relay_name
    terminal.add_tab("", "roslaunch drone_coverage drone_collision_avoidance.launch "+args)
    # Spawining the relay node
    args =  "prev_name:="+relay_names[relay_name_index-1]+" " 
    args += "self_name:="+relay_name+" " 
    args += "next_name:="+relay_names[relay_name_index+1]+" "
    args += "base_name:="+relay_names[0]
    terminal.add_tab("","roslaunch drone_coverage relay_node.launch "+args)
    terminal.create()


terminal = GnomeTerminalHelper()
# Spawning the the graph loader node
terminal.add_tab("","roslaunch graph_loader graph_knowledge.launch")
# Spawning the drone coverage node
terminal.add_tab("","roslaunch drone_coverage drones_coverage.launch")
terminal.create()