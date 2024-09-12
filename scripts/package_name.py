#!/usr/bin/env python3
# ============================================================
# This script extracts package names from given xml file paths
# and prints them to stdout.
# ============================================================
# Author: Yuxuan Zhang
# Email : robotics@z-yx.cc
# License: MIT
# ============================================================
import sys
from glob import glob
from pathlib import Path
import xml.etree.ElementTree as ET

SRC = Path("src")


ignore_list = [_.parent for _ in SRC.rglob("COLCON_IGNORE") if _.is_file()]


def ignored(c: Path):
    for p in ignore_list:
        if c.is_relative_to(p):
            return True
    return False


package_list = [_ for _ in SRC.rglob("package.xml") if not ignored(_)]


def extract_package_name(xml_file):
    try:
        root = ET.parse(xml_file).getroot()
        name_node = root.find("name")
        if name_node is not None:
            return name_node.text
    except Exception as e:
        print(f"Error parsing {xml_file}: {e}", file=sys.stderr)
    return None


for path in package_list:
    package_name = extract_package_name(path)
    if package_name is not None:
        print(package_name)
    else:
        print(f"Package name missing in {path}", file=sys.stderr)
