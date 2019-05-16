
#!/usr/bin/env python

#######################################################################
#
# @filename:    AOSP_Settings_Developer_options_Enable_disable.py
# @description: Turn ON and OFF devloper options
# @author:      dpanday@intel.com
#######################################################################

# Build in libraries
import sys

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps

# ############# Get parameters ############
globals().update(vars(get_args(sys.argv)))
args = {}
if script_args[0].upper() != 'NONE':
    for entry in script_args:
        key, val = entry.split("=")
        args[key] = val


# Setup
ui_steps.press_home(serial=serial)()


if "scenario" in args.keys():
    scenario=args["scenario"]
else:
    scenario=""

if "iteratios" in args.keys():
    iterations=args["iterations"]
else:
    iterations=1
iterations=int(iterations)


# Setup
ui_steps.press_home(serial=serial)()

# Run
if scenario == "Show_surface_updates":
    ui_steps.enable_options_from_developer_options(serial=serial,
                 developer_options=["Show_surface_updates"]
                                               )()
    ui_steps.disable_options_from_developer_options(serial=serial,
                 developer_options=["Show surface updates"],
                                                enabled=True)()
elif scenario == "Force_GPU_rendering" :
    ui_steps.enable_options_from_developer_options(serial=serial,
                                                   developer_options=["Force GPU rendering"],
                                                   )()
    ui_steps.disable_options_from_developer_options(serial=serial,

                                                  developer_options=["Force GPU rendering"],
                                                    enabled=True)()
elif scenario == "Strict_mode_enabled" :
    ui_steps.enable_options_from_developer_options(serial=serial,
                                                   developer_options=["Strict mode enabled"],
                                                   )()
    ui_steps.disable_options_from_developer_options(serial=serial,
                                                    developer_options=["Strict mode enabled"],
                                                    enabled=True)()
elif scenario == "Show_hardware_layers_updates" :
    ui_steps.enable_options_from_developer_options(serial=serial,
                                                   developer_options=[
                                                       "Show hardware layers updates"],
                                                   )()
    ui_steps.disable_options_from_developer_options(serial=serial,
                                                    developer_options=[
                                                        "Show hardware layers updates"],
                                                    enabled=True)()

elif scenario =="Pointer_location":
    ui_steps.enable_options_from_developer_options(serial=serial,
                                                   developer_options=["Pointer location"],
                                                   )()
    ui_steps.disable_options_from_developer_options(serial=serial,
                                                    developer_options=["Pointer location"],
                                                    enabled=True)()


elif scenario =="Show_all_ANRs":
    ui_steps.enable_options_from_developer_options(serial=serial,
                                                   developer_options=["Show all ANRs"],
                                                   )()
    ui_steps.disable_options_from_developer_options(serial=serial,
                                                    developer_options=["Show all ANRs"],
                                                    enabled=True)()


#teardown
####each step has its own teardown





