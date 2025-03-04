import os
from pathlib import Path

print("I am located at", Path(__file__).parent)
print("You are at", Path(os.getcwd()))