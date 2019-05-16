#!/usr/bin/env python

########################################################################
#
# @filename:    local_step.py
# @description: Local test step
# @author:      ion-horia.petrisor@intel.com
#
########################################################################

from testlib.base.base_step import step as base_step
from testlib.utils.connections.local import Local as connection_local


class step(base_step):
    '''helper class for all local (host machine) test steps'''
    local_connection = None

    def __init__(self, **kwargs):
        self.local_connection = connection_local(**kwargs)
        base_step.__init__(self, **kwargs)
