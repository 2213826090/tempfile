#!/usr/bin/env python

##############################################################################
#
# @filename:    abstract_step.py
# @description: Defines the mock class to test the astract step
# @author:      aurel.constantin@intel.com
#
##############################################################################

from testlib.base.base_step import step as base_step


class mock_step(base_step):
    """ Mock step used for mock testing. It prints out the step name and the parameters given to it.
        This step always passes.
    """
    def __init__(self, **kwargs):
        base_step.__init__(self, **kwargs)
        print "Calling mock step {0} with the following parameters: ".format(self.__class__.__name__ )
        for key in kwargs:
            print  "{0} = {1}".format(key, kwargs[key])
        if not kwargs:
            print "No parameters ware given"

    def do(self):
        pass

    def check_condition(self):
        return True
