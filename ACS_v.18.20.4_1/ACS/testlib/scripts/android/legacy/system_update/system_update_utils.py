#!/usr/bin/env python

#######################################################################
#
# @filename:    system_update_utils.py
# @description: Utils for system update tests
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

internal, external, version =\
    open("/home/sys_spjenkins/work/testlib/scripts/system_update/resources.in",
         "rt").readlines()


def get_internal_url():
    return internal.strip()

def get_external_url():
    return external.strip()

def get_version_path():
    return version.strip()
