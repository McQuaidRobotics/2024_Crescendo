import json
from PIL import Image, ImageDraw
import math
import os

percent_size = 0.25
robot_box = True

# Parameters
width, height = 3840, 1556
pixels_per_meter = 170
frame_width = 0.6604
# (3326, 78)
origin = (513, 1477)


base_image_path = './autogifgenerator/2160xDarkCroppedFixed.png'
robot_image_path = './autogifgenerator/Peter_Griffin.png'

# Load the base image
base_image = Image.open(base_image_path)
robot_image = Image.open(robot_image_path)
robot_image = robot_image.resize((int(frame_width * pixels_per_meter), int(frame_width * pixels_per_meter)))

base_image = base_image.resize((int(base_image.width * percent_size), int(base_image.height * percent_size)))
robot_image = robot_image.resize((int(robot_image.width * percent_size), int(robot_image.height * percent_size)))

if robot_box:
    draw = ImageDraw.Draw(robot_image)
    box_thickness = 1
    draw.rectangle(
        [box_thickness, box_thickness, robot_image.width - box_thickness, robot_image.height - box_thickness],
        outline="red",
        width=box_thickness
    )


# Load the auto file

auto_directory = './autogifgenerator/autos'

auto_files = [os.path.join(auto_directory, f) for f in os.listdir(auto_directory) if f.endswith('.meow')]

for auto_file_path in auto_files:
    note_locations = {
        # Location : Display
        (16.581120000000002 * 0.175, 7.0) : True,
        (16.581120000000002 * 0.175, 5.54) : True,
        (16.581120000000002 * 0.175, 4.1) : True,
        (16.581120000000002 * 0.825, 7.0) : True,
        (16.581120000000002 * 0.825, 5.54) : True,
        (16.581120000000002 * 0.825, 4.1) : True,
        (16.581120000000002 / 2.0, 7.44) : True,
        (16.581120000000002 / 2.0, 5.77) : True,
        (16.581120000000002 / 2.0, 4.11) : True,
        (16.581120000000002 / 2.0, 2.42) : True,
        (16.581120000000002 / 2.0, 0.76) : True,
    }

    # Load the auto file
    auto_data = json.load(open(auto_file_path, 'r'))
    auto_file_name = './autogifgenerator/gifs/' + os.path.basename(auto_file_path) + '.gif'
    robot_poses = auto_data['poses']
    note_data = auto_data['notes']

    completed_poses = []
    frames = []
    robot_image_cache = {}

    for i in range(len(robot_poses)):
        frame = base_image.copy()  # Copy the base image for each frame
        draw = ImageDraw.Draw(frame)

        raw_x = robot_poses[i]['translation']['x']
        raw_y = robot_poses[i]['translation']['y']
        
        x = (origin[0] + raw_x * pixels_per_meter) * percent_size
        y = (origin[1] - raw_y * pixels_per_meter) * percent_size


        rotation = robot_poses[i]['rotation']['radians']
        print(f"Frame {i}: x={x}, y={y}, rotation={rotation}")  # Debugging print to check coordinates
        rotation_degrees = rotation * (180 / math.pi) - 90
        
        if rotation_degrees not in robot_image_cache:
            robot_image_cache[rotation_degrees] = robot_image.rotate(rotation_degrees, expand=True)
        
        rotated_robot_image = robot_image_cache[rotation_degrees]
        
        top_left_x = int(x - rotated_robot_image.width / 2)
        top_left_y = int(y - rotated_robot_image.height / 2)
        
        frame.paste(rotated_robot_image, (top_left_x, top_left_y), rotated_robot_image)



        for note in note_locations:
            for data in note_data:
                if i == data['pose_index'] and (data['note_translation']['x'], data['note_translation']['y']) == note:
                    note_locations[note] = False

        for note in note_locations:
            if note_locations[note]:
                note_x = (origin[0] + note[0] * pixels_per_meter) * percent_size
                note_y = (origin[1] - note[1] * pixels_per_meter) * percent_size
                draw.ellipse([note_x - 5, note_y - 5, note_x + 5, note_y + 5], fill="orange")

        frames.append(frame)

    # double FIELD_LENGTH = 16.581120000000002
    # double FIELD_WIDTH = 8.19912

    # (514, 1477)

    # Save as GIF
    print(f"Saving {auto_file_name}")
    frames[0].save(auto_file_name, save_all=True, append_images=frames[1:], optimize=True, duration=20, loop=0)