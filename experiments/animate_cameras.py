import bpy
import json
import sys
import os

def main():
    PI = 3.14159

    planner_kinds = ['greedy', 'formation', 'assignment']

    # Get correct location    
    fname = bpy.path.basename(bpy.context.blend_data.filepath)
    path = bpy.data.filepath[: -len(fname)]
    os.chdir(path)

    # Animate and render for each planner    
    for planner in planner_kinds:
        planner_path = f'blender_output/{planner}'
        os.makedirs(planner_path, exist_ok=True)

        solution_dict = {}
        with open(f"{planner}/solution.json", "r") as f:
            solution_dict = json.load(f)

        bpy.ops.object.select_all(action="DESELECT")
        # Find cameras
        for ob in bpy.context.visible_objects:
            if "uav" in ob.name:
                ob.select_set(True)

        heading_angles = {
            "E": 0.0,
            "NE": PI / 4.0,
            "N": PI / 2.0,
            "NW": 3 * PI / 4.0,
            "W": PI,
            "SW": 5 * PI / 4.0,
            "S": 3 * PI / 2.0,
            "SE": 7 * PI / 4.0,
        }
        # Get names of cameras
        camera_names = []
        for idx, camera in enumerate(bpy.context.selected_objects):
            camera_names.append(camera.name)

        # Animate cameras
        for t in range(0, solution_dict["elements"][0][1][0]["horizon"]):
            # scene.frame_set(t)
            for idx, camera in enumerate(bpy.context.selected_objects):
                x = solution_dict["elements"][idx][1][t]["state"]["x"]
                y = solution_dict["elements"][idx][1][t]["state"]["y"]
                h = solution_dict["elements"][idx][1][t]["state"]["heading"]
                angle_z = heading_angles[h] - PI/2.
                camera.location.x = x
                camera.location.y = y
                camera.location.z = 5.0
                camera.rotation_euler.z = angle_z
                camera.rotation_euler.x = PI/2. - 0.3490655
                camera.rotation_euler.y = 0.0
                camera.keyframe_insert(data_path="location", frame=t)
                camera.keyframe_insert(data_path="rotation_euler", frame=t)

        # Render out each camera
        for camera_name in camera_names:
            render_path = f'{planner_path}/{camera_name}/'
            os.makedirs(render_path, exist_ok=True)
            bpy.context.scene.camera = bpy.data.objects[camera_name]
            bpy.context.scene.render.filepath = render_path
            bpy.ops.render.render(animation=True)
        
if __name__ == "__main__":
    main()
