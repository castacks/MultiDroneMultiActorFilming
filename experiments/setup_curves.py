import bpy

bpy.ops.object.select_all(action="DESELECT")
bpy.data.worlds["World"].node_tree.nodes["Background"].inputs[0].default_value = (1, 1, 1, 1)
marker = bpy.data.objects["start_mark"]

for ob in bpy.data.objects:
    if "Bezier" in ob.name:
        #@ob.select_set(True)
        ob.data.bevel_depth = 0.05
        bpy.ops.object.select_all(action="DESELECT")
        marker.select_set(True)
        bpy.ops.object.duplicate_move()
        marker.select_set(False)
        print(bpy.context.selected_objects)
        new_marker = bpy.context.selected_objects[0]
        new_marker.location = ob.location
        bpy.ops.object.select_all(action="DESELECT")
        ob.select_set(True)
        new_marker.select_set(True)
        bpy.ops.object.make_links_data(type="MATERIAL")