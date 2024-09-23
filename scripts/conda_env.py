#!/usr/bin/env python3
import sys, yaml

try:
    env = yaml.safe_load(sys.stdin)
except yaml.YAMLError as e:
    print(e)
    sys.exit(-1)

if "name" in env:
    print(env["name"])
    sys.exit(0)
else:
    print("environment file does not have a name")
    sys.exit(-1)
