import os

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

drones_names = list(map(lambda name : name.strip(), filter(None, input().split(" "))))
# Spawining the pid controllers
terminal = GnomeTerminalHelper()
for drone_name in drones_names :
    terminal.add_tab("","roslaunch drone_coverage drone_pid_controller.launch drone_name:="+drone_name)
terminal.create()

relay_names = drones_names.copy()
relay_names.insert(0, "base_station")
relay_names.append("")
# Spawning the relay nodes
terminal = GnomeTerminalHelper()
for i in range(1, len(drones_names)+1):
    args = "prev_name:="+relay_names[i-1]+" self_name:="+relay_names[i]+" next_name:="+relay_names[i+1]
    terminal.add_tab("","roslaunch drone_coverage relay_node.launch "+args)
terminal.create()

# The drones name to pass as parameter
drones_names_arr =",".join(drones_names)

terminal = GnomeTerminalHelper()
# Spawning the the graph loader node
terminal.add_tab("","rosrun graph_loader graph_loader.py")
# Spawning the drone coverage node
terminal.add_tab("","roslaunch drone_coverage drones_coverage.launch drones_names:=["+drones_names_arr+"]")
# Spawning the collision avoidance node
terminal.add_tab("", "roslaunch drone_coverage drones_collision_avoidance.launch drones_names:=["+drones_names_arr+"]")
terminal.create()