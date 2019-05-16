#!/usr/bin/env python

########################################################################
#
# @filename:    fastboot_step.py
# @description: fastboot test step
# @author:      costin.carabas@intel.com
#
########################################################################

from testlib.scripts.android.android_step import step as android_step

class step(android_step):
    '''helper class for all adb test steps'''

    def __init__(self, **kwargs):
        android_step.__init__(self, **kwargs)
