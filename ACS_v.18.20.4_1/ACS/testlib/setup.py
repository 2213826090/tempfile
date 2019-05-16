#!/usr/bin/env python

########################################################################
#
# @filename:    setup.py
# @description: add testlib to your PYTHONPATH
# @author:      alexandrux.n.branciog@intel.com
#
########################################################################

import os
import subprocess

module_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

# add testlib to python path
update_path = "echo export PYTHONPATH={0}:'$PYTHONPATH' >> ~/.bashrc".\
                format(module_dir)
subprocess.Popen(update_path, shell = True)

# create logs dyrectory
create_logs_folder = "mkdir -p {0}".format(os.path.join(module_dir, "testlib", "logs"))
subprocess.Popen(create_logs_folder, shell = True)

