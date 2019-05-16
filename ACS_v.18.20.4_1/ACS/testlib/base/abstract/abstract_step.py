#!/usr/bin/env python

##############################################################################
#
# @filename:    abstract_step.py
# @description: Defines the decorator class to implement the astract step
# @author:      aurel.constantin@intel.com
#
##############################################################################

from testlib.base.abstract import abstract_utils
from testlib.base import base_utils

class abstract_step(object):
    __metaclass__ = base_utils.SingletonType

    def __init__(self, use_module):
        self.target_module = abstract_utils.import_module(use_module)

    def __call__(self, original_class):
        return abstract_utils.get_obj(self.target_module, original_class.__name__)


