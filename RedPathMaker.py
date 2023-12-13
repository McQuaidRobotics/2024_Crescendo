
from dataclasses import dataclass
from typing import List, Tuple, Optional

FIELD_LENGTH = 16.53


@dataclass
class Translation:
    x: float
    y: float

    def translate_across_field(self) -> "Translation":
        return Translation(FIELD_LENGTH - self.x, self.y)

    @staticmethod
    def from_json(json: Optional[dict[str, any]]) -> Optional["Translation"]:
        if json is None:
            return None
        return Translation(json["x"], json["y"])

    def to_json(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y}

@dataclass(init=False)
class Rotation:
    deg: float

    def __init__(self, deg: float):
        self.deg = deg % 360

    def translate_across_field(self) -> "Rotation":
        return Rotation(180 - self.deg)

    @staticmethod
    def from_json(json: Optional[float]) -> Optional["Rotation"]:
        if json is None:
            return None
        return Rotation(json)

    def to_json(self) -> float:
        return self.deg

@dataclass
class Waypoint:
    anchor_point: Translation
    prev_control: Optional[Translation]
    next_control: Optional[Translation]
    vel_override: Optional[float]
    holonomic_rotation: Optional[Rotation]
    is_reversal: bool
    is_stop_point: Optional[bool]

    def translate_across_field(self) -> "Waypoint":
        return Waypoint(
            self.anchor_point.translate_across_field(),
            self.prev_control.translate_across_field() if self.prev_control else None,
            self.next_control.translate_across_field() if self.next_control else None,
            self.vel_override,
            self.holonomic_rotation.translate_across_field() if self.holonomic_rotation else None,
            self.is_reversal,
            self.is_stop_point
        )

    @staticmethod
    def from_json(json: dict[str, any]):
        return Waypoint(
            Translation.from_json(json["anchorPoint"]),
            Translation.from_json(json.get("prevControl", None)),
            Translation.from_json(json.get("nextControl", None)),
            json.get("velOverride", None),
            Rotation.from_json(json.get("holonomicAngle", None)),
            json["isReversal"],
            json.get("isStopPoint", False)
        )

    @staticmethod
    def to_json(waypoint: "Waypoint") -> dict[str, any]:
        return {
            "anchorPoint": waypoint.anchor_point.to_json(),
            "prevControl": waypoint.prev_control.to_json() if waypoint.prev_control else None,
            "nextControl": waypoint.next_control.to_json() if waypoint.next_control else None,
            "holonomicAngle": waypoint.holonomic_rotation.to_json() if waypoint.holonomic_rotation else None,
            "isReversal": waypoint.is_reversal,
            "velOverride": waypoint.vel_override,
            "isLocked": True,
            "isStopPoint": waypoint.is_stop_point,
            "stopEvent": {
                "names": [],
                "executionBehavior": "parallel",
                "waitBehavior": "none",
                "waitTime": 0
            }
        }


def waypoints_from_json(json: dict[str, any]) -> List[Waypoint]:
    return [Waypoint.from_json(wp) for wp in json["waypoints"]]

def json_from_waypoints(waypoints: List[Waypoint]) -> dict[str, any]:
    return {
        "waypoints": [Waypoint.to_json(wp) for wp in waypoints],
        "markers": []
    }


DIR = "./src/main/deploy/pathplanner"

if __name__ == "__main__":
    import json, os

    # walk the directory,
    # delete any file that ends in "_R.path",
    # read all .path files,
    # translate them across the field and write them to a new file with "_R.path" appended
    for root, _, files in os.walk(DIR):
        for file in files:
            if file.endswith("_R.path"):
                os.remove(os.path.join(root, file))
        for file in files:
            if file.endswith(".path") and not file.endswith("_BI.path") and not file.endswith("_R.path"):
                json_data = json.load(open(os.path.join(root, file)))
                waypoints = waypoints_from_json(json_data)
                waypoints = [wp.translate_across_field() for wp in waypoints]
                json_data = json_from_waypoints(waypoints)
                json.dump(json_data, open(os.path.join(root, file[:-5] + "_R.path"), "w"), indent=4)


