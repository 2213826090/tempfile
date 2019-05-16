#!/usr/bin/env python

########################################################################
#
# @filename:    ssh_step.py
# @description: SSH test step
# @author:      ion-horia.petrisor@intel.com
#
########################################################################

from testlib.base.base_step import step as base_step
from testlib.utils.connections.ssh import SSH as connection_ssh


class step(base_step):
    '''helper class for all ssh test steps'''
    ssh_connection = None

    def __init__(self, sleep_time = 1, **kwargs):
        self.ssh_connection = connection_ssh(**kwargs)
        base_step.__init__(self, **kwargs)