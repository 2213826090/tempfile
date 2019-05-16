import importlib
import os
import pickle
import pprint
import pydoc
import sys
ROOT = "/home/oane/Work/automation/"
PATH = "testlib/scripts"
PYOBJECT_PATH = "testlib/external/pyobjects.py"

step_files = []
for dp, dn, filenames in os.walk(ROOT + PATH):
    for f in filenames:
        if "steps" in f and ".pyc" not in f:
            step_file = {}
            step_file["module"] = dp
            step_file["step_file"] = f
            step_files.append(step_file)

from testlib.scripts.connections.local import local_steps
steps = []
for step_file in step_files:
    print "*****" + step_file["module"] + "." + step_file["step_file"] + "*****"
    module = step_file["module"].split(ROOT)[1].replace("/", ".") + "." + step_file["step_file"].strip(".py")
    module_path = step_file["module"] + "/" + step_file["step_file"]
    module_python_path = module_path.split(ROOT)[1].replace("/", ".").strip(".py")
    module = importlib.import_module(module_python_path)

    command = "python " + ROOT + PYOBJECT_PATH + " " + module_path
    stdout, stderr = local_steps.command(command = command)()
    new_step = {}
    for line in stdout.split("\n"):
        if "_step)" in line:
            if new_step.has_key("class"):
                new_step["python_path"] = module_python_path
                new_class = getattr(module, new_step["class"])
                new_step["doc"] = new_class.__doc__
                steps.append(new_step)
                new_step = {}
            line_elems = line.split(".")
            new_step["category"] = line_elems[0]
            new_step["class"] = line_elems[1].split("(")[0]
            new_step["step"] = line_elems[1].split("(")[1].strip(")")
        if "__init__" in line:
            new_step["params"] = line.split("(")[1].strip(")")
    new_step["python_path"] = module_python_path
    new_class = getattr(module, new_step["class"])
    new_step["doc"] = new_class.__doc__
    steps.append(new_step)
    print "******************************************************************"
out = open("testlib_steps.dict", "w")
import json
for step in steps:
    #pprint.pprint(json.dumps(step, indent=4), out)
    pprint.pprint(step, out)
print len(steps)
    #pickle.dump(step, out)
out.close()

