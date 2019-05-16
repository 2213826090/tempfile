#!/usr/bin/env python

########################################################################
#
# @filename:    android_utils.py
# @description: android specific utils
# @author:      ion-horia.petrisor@intel.com
#
########################################################################


from testlib.utils.connections.local import Local


def get_package_name_from_apk(apk_path):
    local_connection = Local()
    command = "aapt dump badging " + apk_path + " | grep package"
    stdout, stderr = local_connection.run_cmd(command = command)
    return stdout.split("'")[1]


def get_app_label_from_apk(apk_path):
    local_connection = Local()
    command = "aapt dump badging " + apk_path + " | grep application-label:"
    stdout, stderr = local_connection.run_cmd(command = command)
    return stdout.split("'")[1]

