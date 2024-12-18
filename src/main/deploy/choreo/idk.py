import os
import sys
from pathlib import Path
import json

PATH = Path(__file__).resolve().parent

DEFAULT_TRAJECTORY = {
    "waypoints":[],
    "samples":[],
    "splits":[]
}

DEFAULT_SNAPSHOT = {
    "waypoints":[],
    "constraints":[],
    "targetDt":0.05
}

targetDt = {
"exp":"0.05 s",
"val":0.05
}

for file in os.listdir(PATH):
    if file.endswith(".traj"):
        traj: dict[str, dict]
        with open(PATH/file, 'r') as f:
            traj = json.load(f)
            traj["trajectory"] = DEFAULT_TRAJECTORY
            traj["snapshot"] = DEFAULT_SNAPSHOT
            traj["params"]["targetDt"] = targetDt
            if ("pplib_commands" in traj):
                del traj["pplib_commands"]

        with open(PATH/file, 'w') as f:
            json.dump(traj, f, indent=2)