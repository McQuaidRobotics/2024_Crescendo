
DIR = "./src/main/deploy/pathplanner"

if __name__ == "__main__":
    import os
    for root, _, files in os.walk(DIR):
        for file in files:
            if file.endswith("_R.path"):
                os.remove(os.path.join(root, file))


