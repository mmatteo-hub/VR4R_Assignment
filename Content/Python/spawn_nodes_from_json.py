# Importing the Unreal Engine library for python
import unreal
import json
import os

# Deleating all the previous instantiated spheres Actors
old_nodes = unreal.GameplayStatics.get_all_actors_with_tag(unreal.EditorLevelLibrary.get_editor_world(), "node_graph")
for old_node in old_nodes:
    old_node.destroy_actor()

# Getting the base station which is the first PlayerStart Actor found in the world
base_station = unreal.GameplayStatics.get_all_actors_of_class(unreal.EditorLevelLibrary.get_editor_world(), unreal.PlayerStart)[0]
base_pos = base_station.get_actor_transform().translation

# The directory where the script is contained
directory = os.path.dirname(os.path.realpath(__file__))
# Getting the json data
with open(directory+"\graph.json", 'r') as file:
    data = json.load(file)

# Getting the sphere mesh from the default UnrealEngine library
sphere_mesh = unreal.EditorAssetLibrary.load_asset('/Game/Blueprints/NodeGraph')
for node_name in data["nodes"]:
    # Computing the position of the sphere which is the base_pos + pos in graph
    # They are a bit under the pos in the graph so that the drones hover over them
    graph_pos = data["nodes"][node_name]["position"]
    actual_pos = unreal.Vector(base_pos.x+graph_pos[0]*100, base_pos.y+graph_pos[1]*100, base_pos.z+(graph_pos[2]-2)*100)
    # Spawning the Sphere Actor into the world
    # Notice that the position of the actor must be scaled to cm
    sphere =  unreal.EditorLevelLibrary.spawn_actor_from_object(sphere_mesh, actual_pos)
    # Setting the label of the Actor as the name of the node
    sphere.set_actor_label(node_name)
    sphere.tags.append("node_graph")
    # Setting the Actor in the correct path
    sphere.set_folder_path("Spheres")
    # Setting the correct scale
    sphere.set_actor_scale3d(unreal.Vector(2.0, 2.0, 2.0))