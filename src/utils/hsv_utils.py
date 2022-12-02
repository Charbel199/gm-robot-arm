import re
import json

import numpy as np


def parse_hsv_json(file):
    # Create general dictionary
    all_hsv_values = {}

    with open(file, "r") as f:
        hsv_values = json.load(f)
    for k in hsv_values:
        # Process JSON hsv values
        hsv_values[k] = (re.findall("\d+", hsv_values[k]))
        hsv_values[k] = [int(x) for x in hsv_values[k]]

        all_hsv_values[f"{k}_min"] = np.array(hsv_values[k][:3])
        all_hsv_values[f"{k}_max"] = np.array(hsv_values[k][3:])

    return hsv_values


if __name__ == "__main__":
    parse_hsv_json("assets/hsv/gm-colors.json")
