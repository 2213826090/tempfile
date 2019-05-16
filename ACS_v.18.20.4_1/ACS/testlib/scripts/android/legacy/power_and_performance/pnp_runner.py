#!/usr/bin/env python

#######################################################################
#
# @filename:    instrumentation_runner.py
# @description: Instrumentation runner
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android import android_utils
from testlib.scripts.power_and_performance import pnp_utils
from testlib.scripts.power_and_performance import pnp_steps

import os
import time
import sys


def pnp_runner(unique_id, machine_ip, adb_port):
        
    ############################################################################
    # Set env. variables for adb -P commands
    ############################################################################
    os.environ["ADB_PORT"]=sys.argv[3]
    os.environ['ANDROID_ADB_SERVER_PORT']=str(sys.argv[3])
    
    ############################################################################
    # Set runner parameters
    ############################################################################
    runnerini="./{0}runner.ini".format(unique_id)
    
    adb_handler = adb_steps.connect_device(serial="{0}:5555".format(machine_ip), port=adb_port)
    adb_connection = adb_handler.adb_connection
    adb_handler()
    
    query = 'update `machines` set status=2,last_update=now() where ip="{0}"'.format(machine_ip)
    pnp_utils.run_query(query)
    
    next_test = pnp_utils.decide_next_test(adb_connection)
    pnp_utils.set_next_test(runnerini, machine_ip, next_test)
    runner_props = pnp_utils.set_runner_props(runnerini)

    if runner_props["benchmark"] == "Antutu":
        result = pnp_steps.antutu(local_port=(int(adb_port)+1000), 
                                    port = adb_port, 
                                    serial = runner_props["serial"], 
                                    benchmark= runner_props["benchmark"], 
                                    test_case = runner_props["test"]
                                    )()
    elif runner_props["benchmark"] == "Quadrant":
        result = pnp_steps.quadrant(local_port=(int(adb_port)+1000), 
                                    port = adb_port, 
                                    serial = runner_props["serial"], 
                                    benchmark= runner_props["benchmark"], 
                                    test_case = runner_props["test"]
                                    )()
    elif runner_props["benchmark"] == "3D Mark":
        result = pnp_steps.threedmark(local_port=(int(adb_port)+1000), 
                                    port = adb_port, 
                                    serial = runner_props["serial"], 
                                    benchmark= runner_props["benchmark"], 
                                    test_case = runner_props["test"]
                                    )()
    elif runner_props["benchmark"] == "GL Benchmark":
        result = pnp_steps.glbenchmark(local_port=(int(adb_port)+1000), 
                                    port = adb_port, 
                                    serial = runner_props["serial"], 
                                    benchmark= runner_props["benchmark"], 
                                    test_case = runner_props["test"]
                                    )()
    elif runner_props["benchmark"] == "Flash":
        result = pnp_steps.pnp_flasher(local_port=(int(adb_port)+1000), 
                                    port = adb_port, 
                                    serial = runner_props["serial"], 
                                    benchmark= runner_props["benchmark"], 
                                    test_case = runner_props["test"]
                                    )()
    query = 'update `machines` set status=1,retries=0 where ip="{0}"'.format(machine_ip)
    pnp_utils.run_query(query)
    
    for k,r in result.iteritems():
        if runner_props["benchmark"] == "3D Mark":
            runner_props["benchmark"] = runner_props["test"]
        build_specific = pnp_utils.get_report_details(adb_connection, runner_props["benchmark"], k, r, "t100", runner_props["serial"])
        query = pnp_utils.create_push_result_query("measurements_test", build_specific)
        pnp_utils.run_query(query)
    
    
pnp_runner(sys.argv[1], sys.argv[2], sys.argv[3])
