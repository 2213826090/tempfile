#!/usr/bin/env python

##### imports #####
import sys
from testlib.base.base_utils import get_args
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.android.fastboot import fastboot_utils

##### initialization #####
globals().update(vars(get_args(sys.argv)))

##### test start #####
fastboot_utils.push_uiautomator_jar(serial=serial)
fastboot_steps.time_set(serial=serial)()
##### test end #####