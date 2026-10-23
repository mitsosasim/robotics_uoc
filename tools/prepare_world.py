#!/usr/bin/env python3
"""Resolve the supplied world's external mesh paths for a local ROS workspace."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def prepare(sjtu_path, output):
    sjtu_path, output = Path(sjtu_path).resolve(), Path(output).resolve()
    source = ROOT/'assignments/03_particle_filter/assign3/worlds/new_world.world'
    tree = ET.parse(source)
    for uri in tree.findall('.//uri'):
        if '/sjtu-drone/meshes/' in uri.text:
            asset = sjtu_path/'meshes'/uri.text.rsplit('/', 1)[1]
        elif uri.text.startswith('model://fly_indoor/'):
            asset = sjtu_path/'models'/uri.text[len('model://'):]
        else:
            continue
        if not asset.is_file():
            raise FileNotFoundError(f'Required Gazebo mesh not found: {asset}')
        uri.text = asset.as_uri()
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding='unicode', xml_declaration=True)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sjtu-path', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'build/new_world.world')
    args = parser.parse_args()
    print(prepare(args.sjtu_path, args.output))
