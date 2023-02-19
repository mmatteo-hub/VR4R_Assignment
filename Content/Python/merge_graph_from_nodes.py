# Importing the Unreal Engine library for python
import unreal
import json
import os

# Getting all the nodes
nodes = unreal.GameplayStatics.get_all_actors_with_tag( unreal.EditorLevelLibrary.get_editor_world(), "node_graph")

# The directory where the script is contained
directory = os.path.dirname(os.path.realpath(__file__))
# Getting the json data
with open(directory+"\graph.json", 'r') as file:
    data = json.load(file)

# Getting the base station which is the first PlayerStart Actor found in the world
base_station = unreal.GameplayStatics.get_all_actors_of_class(unreal.EditorLevelLibrary.get_editor_world(), unreal.PlayerStart)[0]
base_pos = base_station.get_actor_transform().translation

for node in nodes:
    # The position of the Actor, it is in cm
    actual_pos = node.get_actor_transform().translation
    graph_pos = unreal.Vector((actual_pos.x-base_pos.x)/100, (actual_pos.y-base_pos.y)/100, ((actual_pos.z-base_pos.z)/100)+2)
    # Storing the new position of the node
    data["nodes"][node.get_actor_label()]["position"] = [graph_pos.x, graph_pos.y, graph_pos.z]

# Opening a new file for storing the changed data
with open(directory+"\graph_new.json", 'w') as file:
    json.dump(data, file, indent=4)