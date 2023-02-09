import bpy
import json

actor_names = []
# Selecting Actors

bpy.ops.object.select_all(action='DESELECT')

for ob in bpy.data.objects:
    if ob.name.startswith('actor'):
        ob.select_set(True)
        actor_names.append(ob.name)

# Number of actors
num_actors = len(actor_names)

# Getting scene scale from boundary
scene_size = bpy.data.objects['boundary'].scale

# Root level structure
scene_data = {'scene': {'scale': {'x': scene_size[0], 'y': scene_size[1], 'z': scene_size[2]}}}
fname = bpy.path.basename(bpy.context.blend_data.filepath)
path = bpy.data.filepath[:-len(fname)]
scene_data.update({'filename': fname})

# Writing Position data
scene = bpy.context.scene
position_data = {'num_frames': (scene.frame_end+1 - 1), 'actor_positions': {}}

# Add actor names
for name in actor_names:
    position_data['actor_positions'].update({name: []})
    

# Add position data over time
for t in range(1, scene.frame_end+1):
    scene.frame_set(t)
    for ob in bpy.context.selected_objects:
        true_loc = ob.matrix_world.to_translation()
        true_rot= ob.matrix_world.to_quaternion().to_euler()
        position_data['actor_positions'][ob.name].append({'location': [x for x in true_loc], 'rotation': [x for x in true_rot]})


# Dump into file
scene_data.update(position_data)

# Write to file
fname_trim = fname[:-len(".blend")]
savename = path + f"{fname_trim}_data.json"

with open(savename, 'w') as f:
    f.write(json.dumps(scene_data))

print(json.dumps(scene_data))
print(f'Writing: {savename}')    