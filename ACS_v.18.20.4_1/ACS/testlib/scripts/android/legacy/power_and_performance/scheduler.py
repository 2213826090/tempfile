#!/usr/bin/env python

##############################################################################
#
# @filename:    runner_pnp.py
#
# @description: Initiates runner instances on all available machines
#
# @author:      emilianx.c.ioana@intel.com
#
##############################################################################

from testlib.scripts.power_and_performance import pnp_utils

import os
machines = pnp_utils.run_query("select * from `machines` where status=1 OR (status=2 and last_update<date_sub(now(),interval '20:0' minute_second))")

for machine in machines:
    command = "nohup python /sp/pnp_automation/testlib/runners/pnp_runner.py {0} {1} {2} &".format(machine[0],machine[3],machine[4])
    print command
    os.system(command)
