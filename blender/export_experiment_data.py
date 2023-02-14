import bpy
import json

# Selecting Actors
actor_names = []
robot_names = []

bpy.ops.object.select_all(action="DESELECT")

# Writing Position data
scene = bpy.context.scene
# Getting scene scale from boundary
scene_size = bpy.data.objects["boundary"].scale

# Root level structure
json_root = {
    "scene": {"scale": {"x": scene_size[0], "y": scene_size[1], "z": scene_size[2]}}
}

fname = bpy.path.basename(bpy.context.blend_data.filepath)
path = bpy.data.filepath[: -len(fname)]
json_root.update({"filename": fname})

for ob in bpy.data.objects:
    if ob.name.startswith("actor"):
        ob.select_set(True)
        actor_names.append(ob.name)

# Setting up robots. For now ignore initial states
robot_data = {'robot_fovs': []}
num_robots = 0
for camera in bpy.data.cameras:
    robot_data['robot_fovs'].append(camera.angle)
    num_robots += 1
json_root.update(robot_data)
# Number of actors and drones
num_actors = len(actor_names)
json_root.update({'num_targets': num_actors, 'num_robots': num_robots})


position_data = {"num_frames": (scene.frame_end + 1 - 1), "actor_positions": {}}

# Add actor names
for name in actor_names:
    position_data["actor_positions"] = []

# Add position data over time
for t in range(1, scene.frame_end + 1):
    scene.frame_set(t)
    for ob in bpy.context.selected_objects:
        true_loc = ob.matrix_world.to_translation()
        true_rot = ob.matrix_world.to_quaternion().to_euler()
        position_data["actor_positions"].append(
            {"location": [x for x in true_loc], "rotation": [x for x in true_rot]}
        )


# Dump into file
json_root.update(position_data)

# Write to file
fname_trim = fname[: -len(".blend")]
savename = path + f"{fname_trim}_data.json"

with open(savename, "w") as f:
    f.write(json.dumps(json_root))

print(json.dumps(json_root))
print(f"Writing: {savename}")
