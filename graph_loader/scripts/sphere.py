import json
import unreal

# Connect to the Unreal Editor
editor = unreal.EditorLevelLibrary()

# Open the JSON file and load the data
with open('../graphs/path.json', 'r') as file:
    data = json.load(file)

# Get a reference to the current level
level = unreal.EditorLevelLibrary.get_editor_world().get_level(0)

# Define the properties of the sphere
sphere_radius = 50

# Loop through the points and spawn a sphere at each one
for node in data['nodes']:
    # Create a new sphere actor
    sphere_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.StaticMeshActor.static_class(),
        unreal.Vector(data['nodes'][node][0], data['nodes'][node][1], data['nodes'][node][2]),
        level.get_actor_location(),
    )

    # Set the static mesh of the sphere to a built-in sphere mesh
    sphere_mesh = unreal.EditorAssetLibrary.load_asset('/Engine/BasicShapes/Sphere')
    sphere_actor.set_editor_property('static_mesh_component', sphere_mesh)

    # Set the radius of the sphere
    sphere_actor.set_editor_property('collision_radius', sphere_radius)

# Save the level
unreal.EditorLevelLibrary.save_current_level()