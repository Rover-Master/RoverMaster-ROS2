#!/usr/bin/python3
import json
from glob import glob

compile_commands = []

for file in glob('build/**/compile_commands.json'):
    with open(file, 'r') as f:
        data = json.load(f)
        compile_commands.extend(data)
        print(file)

with open('compile_commands.json', 'w') as f:
    json.dump(compile_commands, f, indent=2)
