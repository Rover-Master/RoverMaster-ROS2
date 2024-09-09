#!/usr/bin/env python3
# ============================================================
# This script extracts package names from given xml file paths
# and prints them to stdout.
# ============================================================
# Author: Yuxuan Zhang
# Email : robotics@z-yx.cc
# License: MIT
# ============================================================
import xml.etree.ElementTree as ET
import argparse, sys

parser = argparse.ArgumentParser(description="Extract package name from given xml file")
parser.add_argument("paths", nargs='+', default=[], help="Path to the package.xml file")
args = parser.parse_args()

def extract_package_name(xml_file):
    root = ET.parse(xml_file).getroot()
    name_node = root.find("name")
    if name_node is not None:
        return name_node.text
    else:
        return None

for path in args.paths:
    package_name = extract_package_name(path)
    if package_name is not None:
        print(package_name)
    else:
        print(f"Package name missing in {path}", file=sys.stderr)
