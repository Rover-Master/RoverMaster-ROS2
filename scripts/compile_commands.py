#!/usr/bin/python3
# ============================================================
# This python3 script aggregates all compile_commands.json
# files from build directories into a single file at the
# workspace root folder.
# It helps vscode-clangd extension to find all the headers
# and provide better code hints and completion.
# ============================================================
# This script is supposed to be run from the workspace root.
# Makefile will automatically run this after build completes.
# ============================================================
# Author: Yuxuan Zhang
# Email : robotics@z-yx.cc
# License: MIT
# ============================================================

import json
from glob import glob

compile_commands = []

for file in glob('build/**/compile_commands.json'):
    with open(file, 'r') as f:
        data = json.load(f)
        compile_commands.extend(data)

with open('compile_commands.json', 'w') as f:
    json.dump(compile_commands, f, indent=2)
