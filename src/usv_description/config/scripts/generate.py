#!/usr/bin/env python3
"""
Generate a Gazebo model.sdf from a YAML buoy layout file.
Color values also accept integers: 0=green, 1=red, 2=yellow, 3=black
"""

import yaml
import argparse
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, indent, ElementTree

MARKER_RG_MAP = {
    "red": "port",
    "green": "starboard",
}

URI_MAP = {
    "round": "round_buoy_",
    "marker": "marker_buoy_",
}

def build_sdf(buoys: dict, model_name: str = "follow_the_path_challenge") -> ElementTree:
    # Count per-color for sequential naming (G1, G2 … / 1, 2 …)
    color_counters = 0

    sdf = Element("sdf", version="1.6")
    model = SubElement(sdf, "model", name=model_name)

    for buoy_id, props in buoys.items():
        x = float(props.get("x", 0.0))
        y = float(props.get("y", 0.0))
        z = float(props.get("z", 0.0))
        roll  = float(props.get("roll",  0.0))
        pitch = float(props.get("pitch", 0.0))
        yaw   = float(props.get("yaw",   0.0))

        obs_type = str(props.get("type", "buoy"))
        obs_color = str(props.get("color", "red"))
        if obs_type == "marker":
            obs_color = MARKER_RG_MAP[obs_color]

        color_counters += 1

        # naming convention matches the original SDF
        name = f"obs_{color_counters}"

        include = SubElement(model, "include")
        SubElement(include, "static").text = "true"
        SubElement(include, "name").text = name
        SubElement(include, "pose").text = f" {x} {y} {z} {roll} {pitch} {yaw} "
        SubElement(include, "uri").text = URI_MAP[obs_type]+obs_color

    indent(sdf, space="  ")
    return ElementTree(sdf)


def main():
    parser = argparse.ArgumentParser(description="Generate model.sdf from a YAML buoy layout.")
    parser.add_argument("input", help="Path to input YAML file")
    parser.add_argument("-o", "--output", default="model.sdf", help="Output SDF file (default: model.sdf)")
    parser.add_argument("--model-name", default="follow_the_path_challenge",
                        help="SDF model name (default: follow_the_path_challenge)")
    args = parser.parse_args()

    yaml_path = Path(args.input)
    if not yaml_path.exists():
        raise FileNotFoundError(f"YAML file not found: {yaml_path}")

    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    tree = build_sdf(data, model_name=args.model_name)

    script_dir = Path(__file__).parent
    out_path = script_dir / "../../models" / Path(args.input).stem / "model.sdf"
    with open(out_path, "w") as f:
        f.write('<?xml version="1.0"?>\n')
        tree.write(f, encoding="unicode", xml_declaration=False)

    print(f"Written: {out_path}  ({len(data)} buoys)")


if __name__ == "__main__":
    main()