#!/usr/bin/env python

#######################################################################
#
# @filename:    audio_utils.py
# @description: Audio test utilities
# @author:      saddam.hussain.abbas@intel.com
#
#######################################################################

from testlib.utils.connections.adb import Adb
import re


def check_eavb_mode(serial):
    adb_connection = Adb(serial=serial)
    return adb_connection.get_prop("persist.eavb.mode")

def set_eavb_mode(serial, mode):
    try:
        adb_connection = Adb(serial=serial)
        adb_connection.set_prop(prop="persist.eavb.mode", value=mode.lower())
        return True
    except Exception as e:
        print "[{0}] Failed to set eavb mode '{1}'".format(serial, mode)
        print e.message
        return False

def check_gptp_automotive_profile(serial):
    adb_connection = Adb(serial=serial)
    return adb_connection.get_prop("persist.gptp.automotive_profile")

def set_gptp_automotive_profile(serial, state):
    try:
        state = "y" if state.lower() == "on" else "n"
        adb_connection = Adb(serial=serial)
        adb_connection.set_prop(prop="persist.gptp.automotive_profile",
                                   value=state)
        return True
    except Exception as e:
        print "[{0}] Failed to set gptp automotive profile to {1}".format(
            serial, state)
        print e.message
        return False

def check_gptp_service(serial):
    adb_connection = Adb(serial=serial)
    if check_eavb_mode(serial=serial) == "m":
        return adb_connection.get_prop("init.svc.gptp_a")
    else:
        return adb_connection.get_prop("init.svc.gptp_as")

def check_avb_service(serial):
    adb_connection = Adb(serial=serial)
    return adb_connection.get_prop("init.svc.avbstreamhandler")

def set_eavb_audio_on_speaker(serial, state):
    try:
        state = "true" if state.lower() == "on" else "false"
        adb_connection = Adb(serial=serial)
        adb_connection.run_cmd(command="audio_system_setparameters "
                                       "is_eavb_stream={0}".format(state))
        return True
    except Exception as e:
        print "[{0}] Failed to set is_eavb_stream={1}".format(serial, state)
        print e.message
        return False

# TODO: Check if alsa playback is happening. If its using alsa, stop the playback reboot device
# and then start stop stream handler
def avb_stream_handler(serial):
    try:
        adb_connection = Adb(serial=serial)
        adb_connection.run_cmd(command="stop avbstreamhandler")
        if (check_avb_service(serial=serial) == "stopped"):
            adb_connection.run_cmd(command="start avbstreamhandler")
            if (check_avb_service(serial=serial) == "running"):
                return True
    except Exception as e:
        print "[{0}] Failed to start AVB stream handler services".format(serial)
        print e.message
        return False

# TODO: Check if alsa playback is happening. If its using alsa, stop the playback reboot device
# and then start stop gptp
def gptp(serial):
    try:
        adb_connection = Adb(serial=serial)
        adb_connection.run_cmd(command="stop gptp")
        if (check_gptp_service(serial=serial) == "stopped"):
            adb_connection.run_cmd(command="start gptp")
            if (check_gptp_service(serial=serial) == "running"):
                return True
    except Exception as e:
        print "[{0}] Failed to start gptp services".format(serial)
        print e.message
        return False

def sound_card(serial):
    try:
        adb_connection =Adb(serial= serial)
        api_level = adb_connection.get_prop(prop = "ro.build.version.sdk")
        if (api_level < 26):
            card = adb_connection.parse_cmd_output("alsa_aplay_32 -l ")
        else:
            card = adb_connection.parse_cmd_output("alsa_aplay -l ")

        for line in card.splitlines():
            #print line
            search_obj = re.search("(.+): Speaker", line)
            if search_obj:
                rest, device = search_obj.group(1).split(",")
                device_name = re.search("\[(.+)\]", rest).group(1)
                print "Sound card present at" ,device, device_name
                return True
        else:
            raise Exception
    except Exception as e:
        print "[{0}] Failed to list sound card".format(serial)
        print e.message
        return False
