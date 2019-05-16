from os import sys, path
for p in sys.path:
    if "PyUiApiTests" in p:
        sys.path.append(path.dirname(path.dirname(p)))
