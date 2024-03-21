#!/bin/python3

import os
import shutil

rootdir = "."

export_folder = "export_for_drive"
if not os.path.exists(export_folder):
    os.makedirs(export_folder)

for subdir, dirs, files in os.walk(rootdir):
    for file in files:
        if export_folder not in subdir:
            if "evaluation" in file and "tikz" not in file:
                path = os.path.join(subdir, file)
                splits = path.split("/")[1:]
                fname = splits[0] + "_" + splits[1]
                dest = os.path.join(export_folder, fname)
                print(dest)
                shutil.copy(path, dest)