"""
:copyright: (c)Copyright, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL AVE SV
:summary: Implements some common methods for OSbV tests
:since: 17/12/2013
:author: jabowen, cgsapp, sasmith2, others

Functions:
    cellular_signal_properties(device="PHONE1")
    wifi_state(device="PHONE1")
    set_focus_lock(appIdentity='Test_App', focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock'), timeout_sec=120)
    release_focus_lock(appIdentity='Test_App', focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock'))
    cleanup_focus_lock(appIdentity='Test_App', focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock'))
    get_lock_signature(focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock'))
    set_lock_signature(lockSignature, focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock'))
    test_step_temp_dir(test_step_object)

"""
import time
import os
import tempfile
import errno
import sys
import subprocess
from Core.PathManager import Folders

from Device.DeviceManager import DeviceManager
from ErrorHandling.DeviceException import DeviceException
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT as LOGGER

def open_file(filename):
    if sys.platform == "win32":
        process = subprocess.Popen([filename], stdout = subprocess.PIPE, stderr=subprocess.PIPE, shell= True)
    else:
        opener ="open" if sys.platform == "darwin" else "xdg-open"
        process = subprocess.call([opener, filename])
    return process

def stop_file(process):
    if sys.platform == "win32":
        subprocess.Popen("TASKKILL /F /PID {pid} /T".format(pid=process.pid))
    else:
        opener ="open" if sys.platform == "darwin" else "xdg-open"
        subprocess.Popen("kill -9 {pid}".format(pid=process.pid),shell=True)

def wifi_state(device):
    '''
       Returns string stating the current state of the wifi connection with the name of each connected network.

       PARAM device: device object used for Networking UECmds.
    '''
    network_api = device.get_uecmd("Networking")
    if network_api.get_wifi_power_status():
        state = "On"
        network_list = network_api.list_connected_wifi()
        if network_list[0] != "None":
            state = state + " - connected to " + network_list[0]
            for network in network_list:
                if network not in state:
                    state = state + ", " + network
    else:
        state = "Off"

    return state

def set_focus_lock(appIdentity='Test_App', focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock'), timeout_sec=120):
    '''
        Check to see if the GUI focus-lock file already exists. If it doesn't,
        "set" it by creating the lock file. If the focus-lock file does already
        exist, then wait for it to be released.
    '''
    debug_print = LOGGER.debug

    startTime = time.time()
    timeoutMessage = "%s timed out waiting for the GUI focus lock to be released"%appIdentity

    # Create a signature for this instance to protect against race-conditions
    if appIdentity != 'Test_App':
        appSignature = appIdentity
    else:
        import random
        random.seed(startTime)
        appSignature = str(random.randint(1989,80486))

    # If the lock is set and we can identify that this is our own old lock that
    # we didn't release previously, just release it now and move on
    if os.path.exists(focusLockFile):
        if appSignature == get_lock_signature():
            os.remove(focusLockFile)

    while time.time()-startTime < timeout_sec:
        # Wait for the focus-lock to be released
        debug_print("%s is waiting for the GUI focus lock"%appIdentity)
        while os.path.exists(focusLockFile):
            if time.time()-startTime >= timeout_sec:
                debug_print(timeoutMessage)
                return False

        # Assuming GUI focus is not currently being held by another app, so set the
        # focus lock
        if not set_lock_signature(appSignature):
            continue

        if appSignature != get_lock_signature():
            # A race condition occurred and this app lost. Try again.
            debug_print("It seems that a race condition prevented %s from setting the the focus lock. Trying again..."%appIdentity)
            continue

        # Lock should be set now
        debug_print("%s has obtained the GUI focus lock"%appIdentity)
        return True

    # Timeout occurred before the focus-lock could be set
    debug_print(timeoutMessage)
    return False

def release_focus_lock(appIdentity='Test_App', focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock')):
    '''
        Release the GUI focus-lock by deleting the lock file to signal that
        other apps can take focus safely.
    '''
    debug_print = LOGGER.debug

    # Check to make sure that the focus-lock hasn't been released prematurely
    if not os.path.exists(focusLockFile):
        debug_print("The focus-lock file does not exist, but %s did not release it"%appIdentity)
        # This is a benign error and not worth failing the whole test over, so
        # log the above message and return True
        return True

    try:
        os.remove(focusLockFile)
    except Exception, e:
        debug_print("An error occurred while trying to release the focus-lock: %s"%e)
        return False

    # Lock has been released successfully
    debug_print("%s is releasing the GUI focus lock"%appIdentity)
    return True

def cleanup_focus_lock(appIdentity='Test_App', focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock')):
    '''
        Delete any focus-lock files that may be lingering on the system during
        setup or teardown
    '''
    debug_print = LOGGER.debug

    if os.path.exists(focusLockFile):
        try:
            os.remove(focusLockFile)
        except Exception, e:
            debug_print("%s found a lingering focus-lock from a previous run, but failed to release it! --> %s"%(appIdentity,e))
            return False
    # The lock file either didn't exist or it was removed successfully
    return True

def get_lock_signature(focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock')):
    '''
        Read the contents of the existing lock file, specified by the
        focusLockFile parameter
    '''
    debug_print = LOGGER.debug

    try:
        lockFile = open(focusLockFile,'r')
        lockFileSignature = lockFile.read()
        lockFile.close()
    except Exception, e:
        debug_print("There was a problem reading a signature from the lock file: %s"%e)
        return ''

    return lockFileSignature

def set_lock_signature(lockSignature, focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock')):
    '''
        Create the focus lock file and write a signature to it so that the app
        that set it can be identified later
    '''
    debug_print = LOGGER.debug

    try:
        lockFile = open(focusLockFile,'w')
        lockFile.write(lockSignature)
        lockFile.close()
    except OSError, e:
        # A race condition must have occurred. Try again.
        debug_print("It seems that a race condition prevented %s from setting the the focus lock. Trying again..."%appIdentity)
        return False
    except IOError, e:
        # Somehow, the lock file was not fully released, even though
        # os.path.exists reported that it did not exist
        debug_print("The focus lock was not fully released. Trying again...")
        return False
    except Exception, e:
        debug_print("There was a problem writing a signature to the lock file: %s"%e)
        return False

    return True

def test_step_temp_dir(test_step_object):
    '''
        Create a directory structure under tempdir location to store necessary test files that have been created.  Following this model:
            <tempdir>/<campaign_report_directory>/<test_case>/<test_step>
        PARAM test_step_object: test step object of the test step that is calling this function.
        Returns the directory path created.
    '''
    temp_path = tempfile.gettempdir()
    campaign_dir = test_step_object._device.get_report_tree().get_report_path().split(os.path.sep)[-1]
    testcase = test_step_object._testcase_name.split(os.path.sep)[-1]
    teststep = test_step_object._pars.id.lower()
    dest_dir = os.path.join(temp_path, campaign_dir, testcase, teststep)
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)
    return dest_dir


