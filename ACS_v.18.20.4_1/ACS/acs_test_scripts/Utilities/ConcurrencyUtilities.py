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
:summary: Implements methods for ACS concurrency tests
:since: 17/12/2013
:author: jabowen, cgsapp, others

Classes and Functions:
    read_content_versions(testDir)
    get_and_push_app_content(testDir, fileList, no_update=False, tc_parameters=None, device=None)
    install_apk_and_verify(apk_file_path, package_name, device=None)
    runscript(testDir, script, *testArgs, device=None)
    verifyscript(testDir, tc_parameters, cmdTimeout=120, device=None)
    verify_app_running(ps_match_str, max_attempts=1, wait_time_between_attempts=5)
    wake_and_unlock()
    issue_adb_cmd(cmd, parameters=(), timeout=10)
    run_adb_shell_nohup_cmd(cmd, cmd_timeout=10)
    verify_app_running(ps_match_str, max_attempts=1, wait_time_between_attempts=5)
    get_test_content_path()
    get_device_file_md5(file_path, fail_missing_file=False)
    push_test_content_to_device(test_content_path, device_path, decompress_tgz=False)
    fail_verdict_and_debug_print(message='')
    wait_for_file_in_device_dir(dir_on_device, file_names, timeout=60,
        polling_interval=10, start_time_device_file=None)
    fetch_artifactory_file(app_url)
    get_device_file_size(device_file_path)
    auto_timeout_adb_pull(device_file_path, local_path)
    pull_device_files_to_report(device_file_paths, report_paths=None, report_folder=None)
    execute_pass_fail_log_verify(device_log_dir, timeout_seconds=300)
    create_rand_bin_file(file_path, file_size_MB, seed=None)
    cellular_signal_properties(device="PHONE1")
    wifi_state(device="PHONE1")
    stop_android_app(package, device="PHONE1")
    start_android_app(package, activity="Main", action=None, device="PHONE1")
    verify_app_installed(package_name, device="PHONE1")
    campaign_config_param(param, default_value=None, default_cast_type=str)
    runtime_minutes(default_value=None)
    tc_log_dir(return_final_path=False)
    get_phone_number(device=None)
"""
# Python Imports
# import datetime
import time
import os
# import urllib2

import requests
from requests.exceptions import RequestException

from Core.PathManager import Folders

from UtilitiesFWK.Utilities import Global
import UtilitiesFWK.Utilities as Utils
from Device.DeviceManager import DeviceManager
from ErrorHandling.DeviceException import DeviceException
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT as LOGGER


class TestFailedException(Exception):
    pass


def get_phone_number(device=None):
    """
        The output of service call iphonesubinfo is like this:

        Result: Parcel(
        0x00000000: 00000000 0000000b 00350031 00320031 '........1.5.1.2.'
        0x00000010: 00300034 00350030 00370039 00000034 '4.0.0.5.9.7.4...')

        Objective: What we need to get is the phone number: 1-512-400-5974

        STEPS:
        0. It get the self.phone_number_string from the adb command.
        1. It splits the phone_number str
        2. It looks for this character: '
        3. It appends these two items:
           '........1.5.1.2.' and '4.0.0.5.9.7.4...'
           to phone_list.
        4. It erases phone_number variable to re-use it.
        5. Then it joins both items into phone_number
        6. It uses list fucntion to create a list_of_character
        7. Then it tries to convert each character to int
        8. If the character is not a number it will be ignored with PASS
           And it will not be part of the list_of_numbers
        9. list_of_numbers will contain the phone number
           splitted into a list.
        10. Finally it will join those items in phone_number
            variable
        12. If something fails it will return None.

        PARAM device=None: The device to target. Can be the device name in the Bench Config, a
            device object, or None and the first device in the device list will be used.
    """
    phone_number = None
    # STEP 0
    (return_code, phone_number) = issue_adb_cmd('shell service call iphonesubinfo 6', timeout=20, device=device)
    phone_list = []
    # If service call iphonesubinfo gave us some information, then continue
    # else just return a None phone number.
    if phone_number != None:
        # STEP 1
        for item in phone_number.split():
            # STEP 2
            if '\'' in item:
                # STEP 3
                phone_list.append(item)
        # STEP 4
        phone_number = ""
        try:
            # STEP 5
            phone_number = "".join([phone_list[0],phone_list[1]])
            # STEP 6
            list_of_characters = list(phone_number)
            list_of_numbers = []
            for item in list_of_characters:
                # STEP 7
                try:
                    # STEP 9
                    list_of_numbers.append(str(int(item)))
                except:
                    # STEP 8
                    pass
            phone_number = ""
            # STEP 10
            phone_number = "".join(list_of_numbers)
        except:
            # STEP 12
            phone_number = None
    return phone_number


def read_content_versions(testDir):
    """
       Reads version strings from the file TEST_CONTENT_PATH/testDir/VERSION and from
       ARTIFACTORY/testDir/VERSION. Each line of the VERSION file should be of the form
           FILE_NAME,VERSION_INTEGER
       This function returns a tuple with two dictionaries that look like this:
           ( LOCAL_VERSION_FILE[FILENAME] = VERSION_INTEGER,
             ARTIFACTORY_VERSION_FILE[FILENAME] = VERSION_INTEGER )

       PARAM testDir: The path from TEST_CONTENT_PATH of the folder where a VERSION file might exist.
       RETURNS Tuple of the form described above.
    """
    debug_print = LOGGER.debug

    testPath = get_test_content_path()
    if not testPath:
        raise DeviceException(DeviceException.OPERATION_FAILED, "read_content_versions failed due to invalid content path")

    local_test_dir = os.path.join(testPath, testDir)
    versions = dict()
    new_versions = dict()
    local_version_file = os.path.join(local_test_dir, "VERSION")
    #debug_print("read_content_versions: Looking for old VERSION file: " + local_version_file)
    if os.path.exists(local_version_file):
        #read in list of versions previously downloaded
        version_file = open(local_version_file)

        #Handle the case where we somehow ended up with a junk file left behind from a previous run.
        #This should not happen since we check for junk when downloading from Artifactory, but there may
        #have been a glitch in the process that left one behind.
        line = version_file.readline()
        if line[:6] == "<html>" or line[:6] == "<HTML>":    #Artifactory will feed us an HTML page saying "File not found" if the URL is non-existent
            version_file.close()
            os.remove(local_version_file)   #Delete the junk file that existed
            return(None, None)
        version_file.seek(0)

        #Read in each line of comma-separated values and populate a list for easy searching later
        for line in version_file:
            if line.isspace():  #handle case where user put a blank line at the end
                break
            parts = line.split(',')
            try:
                versions[parts[0]] = int(parts[1])
            except (ValueError, IndexError) as e:
                debug_print("read_content_versions: error processing line \"%s\" in old %s" % (line, local_version_file))
                debug_print(" -->  exception message: \"" + str(e) + "\"")
                raise DeviceException(DeviceException.OPERATION_FAILED, "read_content_versions failed due to invalid VERSION file.")
        version_file.close()

    #debug_print("read_content_versions: Looking for new VERSION file: " + local_version_file)
    #fetch new VERSION file from Artifactory and read in list of versions available for download
    local_version_file = fetch_artifactory_file(testDir + "/VERSION")
    if not local_version_file or local_version_file == "":
        debug_print("read_content_versions: No VERSION file in Artifactory")
        return(versions, new_versions)

    version_file = open(local_version_file)
    for line in version_file:
        if line.isspace():  #handle case where user put a blank line at the end
            break
        parts = line.split(',')
        try:
            new_versions[parts[0]] = int(parts[1])
        except (ValueError, IndexError) as e:
            debug_print("read_content_versions: error processing line \"%s\" in new %s" % (line, local_version_file))
            debug_print(" -->  exception message: \"" + str(e) + "\"")
            raise DeviceException(DeviceException.OPERATION_FAILED, "read_content_versions failed due to invalid VERSION file.")
    version_file.close()
    #debug_print("read_content_versions: finished reading VERSION files")
    return(versions, new_versions)


def get_and_push_app_content(testDir, fileList, no_update=None, tc_parameters=None, device=None):
    """
    Description: Perform generic setup for shell script tests
    Inputs
        testDir = directory on DUT under /data to which content should be pushed
        fileList = a list (array) of content files to download from Artifactory, with path relative to acs_test_artifacts/CONCURRENCY/<testDir> in Artifactory.  Same relative path will be used to put the file locally under <repos_root>/acs/src/_ExecutionConfig/CONC_CONTENT/<testDir>.
        no_update = True if you don't want it to download from Artifactory files that already exist locally
        tc_parameters = A test case parameters object. Typically obtained in a use case as self._tc_parameters. Required to read the no_update parameter from XML.
        device = The device to target. Can be the device name in the Bench Config, a device object, or None and the first device in the device list will be used.

    Format of fileList:
        filelist is a list with elements of this format:
                [<parent_dir_name>, <filename>, <action>]

    action="push|unzip|get-only"
        push = simply do an adb push
        unzip = adb push and then unzip on target
        get-only = just downloads from Artifactory, does not push to target.  Use this if you will install an APK,
                    or just use the file locally in a host-side script.

    """
    if no_update == None:
        # Get the parameter from our config files
        no_update = config_param("NO_CONTENT_UPDATE", default_value=False, default_cast_type='str_to_bool', tc_parameters=tc_parameters)
    debug_print = LOGGER.debug
    testPath = get_test_content_path()
    if not testPath:
        raise DeviceException( DeviceException.OPERATION_FAILED,
                "get_and_push_app_content failed due to invalid content path" )
    versions = dict()
    new_versions = dict()
    if not no_update:
        (versions, new_versions) = read_content_versions("CONCURRENCY/TESTS/"+testDir)
    parent_dir_properties = {
        # parent_dir:  ( rel_path,               target_dir,       media_file)
        "concurrency": ( "CONCURRENCY/TESTS/"+testDir, "/data/"+testDir, False ),
        "audio":       ( "CONCURRENCY/AUDIO",          "/sdcard/Music",  True  ),
        "video":       ( "CONCURRENCY/VIDEO",          "/sdcard/Movies", True  )
    }
    for list_line in fileList:
        parent_dir = list_line[0]
        app_file = list_line[1]
        action = list_line[2]
        if parent_dir not in parent_dir_properties:
            debug_print("get_and_push_app_content: Invalid parent '%s' specified in filelist" % parent_dir)
            continue
        (rel_path, target_dir, media_file) = parent_dir_properties[parent_dir]

        #form str representing the path to this file on the local drive, relative to the root of the non-ACS content
        local_rel_file = os.path.join(rel_path, app_file)
        #if on Windows, we need to make sure all forward slashes are converted to back slashes
        local_rel_file = os.path.normpath(local_rel_file)
        local_file = os.path.join(testPath, local_rel_file)
        result = "bogus"    #just a placeholder str so that we can do a single check after this conditional block
            #to see if we successfully fetched a file, and not fail when there was no attempt to fetch one.
        if not os.path.exists(local_file):
            result = fetch_artifactory_file(rel_path + "/" + app_file)
        else:
            if not no_update and not media_file:    #media files should not go through multiple versions, so don't re-download
                if versions is not None and app_file in versions:
                    if versions[app_file] < new_versions[app_file]:
                        debug_print("get_and_push_app_content: Downloading NEWER version of " + app_file)
                        result = fetch_artifactory_file(rel_path + "/" + app_file)
                    else:
                        debug_print("get_and_push_app_content: Skipping download of %s because it's already up-to-date" % app_file)
                else:
                    debug_print("get_and_push_app_content: Unknown version for %s, so overwriting it" % app_file)
                    result = fetch_artifactory_file(rel_path + "/" + app_file)
            else:
                debug_print("get_and_push_app_content: skipping download of %s because it already exists, No update = %s, and media type = %s." % (app_file, str(no_update), str(media_file)))
        if not result:  #fetch_artifactory_file returns None if we try to fetch a non-existent file
            raise DeviceException( DeviceException.OPERATION_FAILED, "get_and_push_app_content: Artifactory does not have '" + rel_path + "/" + app_file + "'")
        bresult = True  #in case we are doing get-only
        if action == "push":
            skip_push = False
            if media_file:
                (return_code, ls_output) = issue_adb_cmd(["shell","ls " + target_dir], timeout=10, device=device)
                for filename in ls_output.split('\n'):
                    if filename == app_file:
                        skip_push = True
                        debug_print("get_and_push_app_content: skipping push of %s because it already exists" % app_file)
                        break
            if not skip_push:
                bresult = push_test_content_to_device(local_rel_file, target_dir, False, device=device)
        elif action == "unzip":
            bresult = push_test_content_to_device(local_rel_file, target_dir, True, device=device)
        elif action != "get-only":
            debug_print("get_and_push_app_content: Invalid action '%s' was specified.  Assuming 'get-only'." %action)

        if not bresult:
            raise DeviceException( DeviceException.OPERATION_FAILED, "push_test_content_to_device failed on " + local_rel_file)


def install_apk_and_verify(apk_file_path, package_name, device=None):
    """
        Perform generic set up for apk files
        Input:
            apk_file_path: file path and filename for APK relative to the directory under
                        the path returned by get_test_content_path()
            package_name: name seen for this package in the output of the "pm list packages" command in Android
            phone: name of the specific device in which the apk will be installed. The default value is PHONE1
    """
    testPath = get_test_content_path()
    if not testPath:
        raise DeviceException( DeviceException.OPERATION_FAILED,
                "get_and_push_app_content failed due to invalid content path")
    local_file = os.path.join(testPath, 'CONCURRENCY' , 'TESTS', apk_file_path)
    #Remove app verifiication. If the device is connected to Wifi or 3G any time we try to install
    # an apk and if the verify apps is enabled on the device, it will prompt to ask if we want Google to
    # verify the application, we do not want to do that, we are skipping that verification.
    device = get_device(device)
    device.get_uecmd("PhoneSystem").set_verify_application(False)
    (return_code, cmd_output) = issue_adb_cmd(['install','-r',local_file], timeout=60, device=device)
    # If the apk has been installed successfully, verify if was installed correctly
    (return_code, cmd_output) = issue_adb_cmd(['shell', 'pm list packages | grep '+package_name], timeout=10, device=device)
    if package_name not in cmd_output:
       raise DeviceException( DeviceException.OPERATION_FAILED,
               package_name +' package was not installed properly.')


def runscript(testDir, script_run_cmd, device=None):
    """
        Run the test script on the Android platform
    """
    # Run test with all parameters
    return run_adb_shell_nohup_cmd('cd /data/%s; nohup ./%s'%(testDir,script_run_cmd), device=device)


def verifyscript(testDir, tc_parameters, cmdTimeout=120, device=None):
    """
        Verify the test run
    """
    execute_pass_fail_log_verify(device_log_dir='/data/'+testDir, timeout_seconds=cmdTimeout, device=device, tc_parameters=tc_parameters)


def wake_and_unlock(device=None):
    device = get_device(device)
    phone_system_api = device.get_uecmd("PhoneSystem")
    system_api = device.get_uecmd("System")
    phone_system_api.wake_screen()
    phone_system_api.set_phone_lock(0)
    issue_adb_cmd('remount', device=device)


def issue_adb_cmd(cmd, timeout=10, device=None):
    """
        Issues an adb command with optional parameters and updates VERDICT and
        OUTPUT provided in global_vars. The adb command is formed with the equation
        ADB Command = "adb " + cmd + " ".join(parameters)

        PARAM cmd: The adb command to execute
        PARAM timeout=10: The max amount of time to wait for the command to complete.
        PARAM device=None: Can be a device object or a string for the device to issue the command to. If None, the first device
          in DeviceManager()'s list will be used. If no device can be resolved, an exception is raised.
        RETURN return_code and cmd_output will be false if phone instance is None. Otherwise,
               one will return the code and the other the command line output.
    """
    device = get_device(device)
    # Lets make sure cmd is a proper list
    if isinstance(cmd, str) or isinstance(cmd, unicode):
        cmd = [cmd]
    if not isinstance(cmd,list):
        cmd = list(cmd)
    if cmd[0].startswith('adb')==False:
        cmd.insert(0, 'adb')
    # The run_cmd function doesn't currently support passing a list of arguments
    adb_cmd = ' '.join(cmd)
    print("Executing "+adb_cmd)
    (return_code, cmd_output) = device.run_cmd(adb_cmd,timeout)
    if return_code != Global.SUCCESS:
        raise DeviceException(DeviceException.OPERATION_FAILED, "The command '%s' returned an error code (%d) and error message '%s'."%(adb_cmd, return_code, cmd_output))
    return return_code, cmd_output


def get_device(device=None):
    """
       Obtains a device object based on the following resolution order:
        1. If device is of IDevice type, use it.
        2. If device is of str or unicode type, look for a device with this name.
        3. Grab the first device in DeviceManager's list.
       If no device can be resolved, a DeviceException is raised.

       PARAM device=None: A device object, string, or None.
       RETURNS a device object or raises DeviceException.

    """
    from Device.Model.IDevice import IDevice

    if isinstance(device, IDevice):
        return device
    # If a string is provided, look for a device with that name
    if isinstance(device, (str, unicode)):
        return DeviceManager().get_device(device)
    all_devices = DeviceManager().get_all_devices()
    if len(all_devices) > 0:
        return all_devices[0]
    raise DeviceException(DeviceException.INVALID_PARAMETER, "In get_device(device=%s) - No device found."%device)


def run_adb_shell_nohup_cmd(cmd, cmd_timeout=10, device=None):
    """
        Intended to run a command on a device using the 'adb shell' command
        with nohup. This function is primarily intended to
        be used to workaround an issue where nohup commands do not get executed
        on the device correctly.

        PARAM cmd: The command to issue on the device. This str will be concatenated with 'adb shell' to form the adb command.
        PARAM cmd_timout=10: The timeout to be used for each attempt at running cmd.
    """
    try:
        # Redirect stderr to stdout.  stdout should already append to the created nohup.out file.
        cmd = cmd + " 2>&1"
        (return_code, cmd_output) = issue_adb_cmd(['shell',cmd], timeout=cmd_timeout, device=device)
        return (Global.SUCCESS, cmd_output)
    except:
        return (Global.SUCCESS, '')


def verify_app_running(ps_match_str, max_attempts=1, wait_time_between_attempts=5, device=None):
    """
       Verifies that ps_match_str is in the output of the ps command. Use this
       function to verify that an application started.

       PARAM ps_match_str: The str to look for in the ps output
       PARAM max_attempts=1: The number of times to look for ps_match_str in the ps output
       PARAM wait_time_between_attempts=5: The number of seconds to wait between attempts
    """
    LOGGER.debug("Looking for '%s' in the ps output"%str(ps_match_str))
    for i in range(max_attempts):
        (return_code, ps_output) = issue_adb_cmd(['shell','ps | grep '+ps_match_str], device=device)
        LOGGER.debug("PS Grepped Output:\n"+str(ps_output))
        if ps_output!='':
            return True
        if i<max_attempts-1:
            time.sleep(wait_time_between_attempts)
    # We've finished all of our max_attempts and never saw the output
    LOGGER.debug("Unable to find ps match string in ps output. FAIL")
    raise DeviceException(DeviceException.OPERATION_FAILED, 'Failed to find "%s" in the ps output.\n'%str(ps_match_str))


def get_test_content_path():
    """
       Returns location of test content that's not checked into the ACS repo.
       Default location is <your_repos_root>/acs/src/_ExecutionConfig/CONC_CONTENT.
       This can be overridden by setting this environment variable:
            TEST_CONTENT_PATH
    """
    import os
    env_path = os.getenv("TEST_CONTENT_PATH")
    if env_path:
        if os.path.exists(env_path):
            return env_path
    path = os.path.join(Folders.EXECUTION_CONFIG, "CONC_CONTENT")
    if not os.path.exists(path):
        os.makedirs(path)
    return path


def get_device_file_md5(file_path, fail_missing_file=False, device=None):
    """
       Executes 'md5sum file_path' on the device and returns the resulting MD5.
       Returns None if the file is not found or there are issues with executing
       the command.

       PARAM file_path: The path to the file on the device.
       PARAM fail_missing_file: Bool indicating if the test should be failed for non-existent files. Currently not implemented.
    """
    LOGGER.debug("Calculating MD5 for file on device at "+str(file_path))
    (return_code, md5_output) = issue_adb_cmd(['shell','if [ -f %s ]; then md5sum %s; fi;'%(file_path,file_path)], device=device)
    print(md5_output)
    if md5_output=='':
        return None
    md5_output_split = md5_output.split(' ')
    if len(md5_output_split)<2:
        return None
    return md5_output_split[0]


def push_test_content_to_device(test_content_path, device_path, decompress_tgz=False, device=None):
    """
       Pushes a file or directory from the test content folder to a device. Error
       checking is performed to make sure the test content can be found and the device
       path exists.

       RETURNS True on successful push of the file(s). False on failure.

       PARAM test_content_path: The path to the files to push. Relative to the root of
           test content folder. The root is equivalent to
           \\amr\ec\proj\debug\TNG\Users\osv_concurrency\.
       PARAM device_path: The location to push the files to.
       PARAM decompress_tgz: Boolean indicating if the file should be decompressed
           after pushing.
    """
    debug_print = LOGGER.debug
    debug_print("Pushing test content from %s to %s. Decompress: %s"%(
        str(test_content_path), str(device_path), str(decompress_tgz) ))
    # Try to get the path to our test content
    test_content_folder_path = get_test_content_path()
    if test_content_folder_path==None: # There was an issue getting the test content path
        # We have no chance of loading the test content. Give up now.
        fail_verdict_and_debug_print("Unable to obtain the test content path. Giving up.")
        return False
    import os
    test_content_file_path = os.path.join(test_content_folder_path, test_content_path)
    if not os.path.exists(test_content_file_path):
        # We have no chance of loading the test content. Give up now.
        fail_verdict_and_debug_print(
            "Test content does not exist: %s. Giving up."%str(test_content_file_path) )
        return False
    test_content_size = "Unknown"
    est_timeout = 100000 # Default to 100 seconds
    if os.path.isfile(test_content_file_path):
        est_timeout = Utils.compute_timeout_from_file_size(test_content_file_path, 10)
    debug_print('Test Content Path: %s. File Size: %s. Push timeout: %d'%(str(test_content_file_path), str(test_content_size), est_timeout))
    # Make sure our device path exists. Create it if it doesn't exist.
    # Also, check first whether there is a FILE of the same name as the desired directory, and delete it if so.
    (return_code, output_msg) = issue_adb_cmd(
        ["shell", "if [ -f %s ]; then rm %s; fi; if [ ! -d %s ]; then mkdir -p %s; fi;"%(device_path,device_path,device_path,device_path)],
        timeout=50, device=device )
    if return_code!=0:
        # The issue_adb_cmd function takes care of failing the verdict
        return False
    # Push the files
    (return_code, output_msg) = issue_adb_cmd(
        ["push",test_content_file_path,device_path],
        timeout=est_timeout, device=device )
    if return_code!=0:
        # The issue_adb_cmd function takes care of failing the verdict
        return False
    (return_code, output_msg) = issue_adb_cmd(['shell','chmod -R 777 '+device_path], timeout=est_timeout, device=device)
    if return_code!=0:
        # The issue_adb_cmd function takes care of failing the verdict
        return False
    if decompress_tgz:
        #We originally tried to have 'tar -zxf' take care of decompressing and un-tarring in one step.
        #Some large files, however, seem to make the Busybox version of tar complain of an invalid tar header checksum.
        #GNU tar handles them just fine, but Android does not come with GNU tar.
        #Splitting into 2 steps, gunzip and then un-tar, seems to work around the problem.
        simple_filename = os.path.basename(test_content_file_path)
        if simple_filename[-4:].lower() != ".tgz":
            debug_print("push_test_content_to_device: unsupported filename extension '%s'.  Expected a .tgz file.  Skipping decompression."%simple_filename[-4:])
            return False
        (return_code, output_msg) = issue_adb_cmd(
            ["shell","cd "+device_path+"; gunzip -f "+ simple_filename],
            timeout=est_timeout, device=device)
        if return_code!=0:
            # The issue_adb_cmd function takes care of failing the verdict
            return False
        unzipped_name = simple_filename[:-4] + ".tar"
        (return_code, output_msg) = issue_adb_cmd(
            ["shell","cd "+device_path+"; tar -xf "+os.path.basename(unzipped_name)],
            timeout=est_timeout, device=device)
        if return_code!=0:
            # The issue_adb_cmd function takes care of failing the verdict
            return False
    return True


def fail_verdict_and_debug_print(message=''):
    """Raises a TestFailedException with message."""

    raise TestFailedException(DeviceException.OPERATION_FAILED, message)


def wait_for_file_in_device_dir(dir_on_device, file_names, timeout=60,
        polling_interval=10, start_time_device_file=None, device=None,
        ps_search_string=None):
    """
       Waits for one of the file names in file_names to appear in the directory,
       dir_on_device on the device. If the file doesn't appear by timeout seconds, an
       exception is raised. When a file is found, that filname is returned.
    """
    if start_time_device_file==None:
        start_time = time.time()
    else:
        # Read from a file on the device
        (return_code, start_time) = issue_adb_cmd(
            ['shell','if [ ! -f %s ]; then cat %s; fi;'%(start_time_device_file,start_time_device_file)],
            timeout=10, device=device )
        if return_code!=0 or start_time=='':
            raise DeviceException(DeviceException.OPERATION_FAILED, "File %s does not contain a starting timestamp."%str(start_time_device_file))
        try:
            start_time = int(start_time)
        except:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to convert timestamp '%s' into an integer."%str(start_time))
    current_time = start_time
    while current_time-start_time<timeout:
        (return_code, dir_listing) = issue_adb_cmd(
            ['shell','ls '+dir_on_device], timeout=10, device=device )
        if return_code!=0:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Failed to obtain a directory listing. See failure notes for more details.")
        # Check if any of our files are found
        for file_name in file_names:
            if file_name in dir_listing:
                return file_name
        if ps_search_string != None:
            # Check that we're still running
            (return_code, ps_grep_output) = issue_adb_cmd( ['shell','ps | grep "%s"'%ps_search_string],
                                                           timeout=10, device=device )
            if ps_grep_output == '':
                message = "Expected files did not appear and string '%s' was not found in the ps output."%ps_search_string
                fail_verdict_and_debug_print(message, raise_exception=True)
        if start_time_device_file==None:
            current_time = time.time()
        else:
            (return_code, current_time) = issue_adb_cmd(["shell","date +%s"], timeout=10, device=device)
            if return_code!=0:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Error getting time from device.")
            current_time = int(current_time)
        time.sleep(polling_interval)
    raise DeviceException(DeviceException.TIMEOUT_REACHED, "Expected files did not appear within timeout given.")


def fetch_artifactory_file(app_url):
    """
    Retrieve a file from artifactory
    """
    debug_print = LOGGER.info
    response = None

    # artifact_url_base = "https://tlsstor001.tl.intel.com/artifactory/acs/acs_dev/"
    # artifact_url_base = "https://mcg-depot.intel.com/artifactory/acs/acs_dev/"

    artifact_url_base = "https://tlsstor001.tl.intel.com/artifactory/acs_test_artifacts/"
    artifact_url = artifact_url_base + app_url

    local_filename = os.path.join(get_test_content_path(), app_url)

    #URL has forward slashes, so if we're running on Windows we need to convert to backslashes when
    #incorporating it into a local path.
    local_filename = os.path.normpath(local_filename)

    debug_print("Downloading {0} to {1}".format(artifact_url, local_filename))
    if not os.path.exists(os.path.dirname(local_filename)):
        os.makedirs(os.path.dirname(local_filename))  # os.makedirs recursively creates
                                                    # all necessary directories in the path
    try:
        # TODO: Use SSL Certificate instead of verify=False
        response = requests.get(artifact_url, proxies={'http': ''}, verify=False)
        response.raise_for_status()

        content_length = float(response.headers['content-length'])
        progress = 0

        with open(local_filename, "wb") as f:
            for chunk in response.iter_content(chunk_size=100000000):
                if chunk:
                    progress += len(chunk)
                    percent = int(progress / content_length * 100)
                    debug_print("Progress of downloading {0} file: {1}%".format(artifact_url, percent))
                    f.write(chunk)

    except RequestException as exc:
        local_filename = ""
        debug_print("fetch_artifactory_file: "
                    "exception {0} when executing requests.get for {1}.".format(exc, artifact_url))
        if response:
            debug_print("Request Status code is : " + str(response.status_code))

    if local_filename:
        with open(local_filename, 'r') as artifact_file:
            line = artifact_file.readline()

        if line[:6] == "<html>" or line[:6] == "<HTML>":  # Artifactory will feed us an HTML page,
                                                          # saying "File not found" if the URL is non-existent

            # These next lines are an attempt to fix an issue where the os.remove() is
            #  called below but the file isn't closed yet, resulting in an exception.
            #  After enough time has elapsed to prove these next lines fix the issue,
            #  this note can be removed.

            start_time = time.time()
            while artifact_file.closed is False:
                if time.time() - start_time < 5:
                    debug_print("Timeout waiting for our temporary file (%s) to be closed. "
                                "Will not remove it.")
                    break
            if artifact_file.closed:
                os.remove(local_filename)  # Delete the junk file that we just received
            debug_print("WARNING: The file '{0}' does not exist in Artifactory.".format(artifact_url))
            return None
        debug_print("Finished downloading {0} to {1}".format(artifact_url, local_filename))

#    (root, ext) = os.path.splitext(app_url)
#    if ext == "zip":
#        z = zipfile.ZipFile(local_filename, mode="r")
#        z.extractall(os.path.dirname(local_filename))
#        z.close()

    return local_filename


def get_device_file_size(device_file_path, device=None):
    """
       Returns the file size (in bytes) of a file on the device.

       PARAM device_file_path: The path to the file on the device.
    """
    # Use ls -l to get the file size. If someone has a better command,
    #  please upgrade this function.
    (return_code, ls_output) = issue_adb_cmd(
        ['shell','ls -l '+device_file_path], timeout=10, device=device )
    if return_code!=0 or "No such file or directory" in ls_output:
        raise DeviceException( DeviceException.INVALID_PARAMETER,
            "Unable to execute ls -l on the file %s. Command output: %s."%(str(device_file_path),ls_output))
    # An ls output looks like this:
    #  -rwxrwxrwx root     root       652566 2014-01-15 11:11 file_name
    return long(ls_output.split()[3])


def auto_timeout_adb_pull(device_file_path, local_path, device=None):
    """
       Pulls a file from device_file_path to local_path and auto-calculates the timeout
       based on the file size.

       PARAM device_file_path: Path to the file on the device.
       PARAM local_path: The path to pull the device to on the PC.
    """
    # Calculate the timeout based on the file size
    file_size = get_device_file_size(device_file_path, device=device)
    # This calculation should probably be modified. This is a first guess at
    #  a value that is 99.9% safe to not give a false failure.
    # File size is in bytes. Timout is in milliseconds.
    pull_timeout = file_size # Transfer must be above 1 kilobyte per second
    if pull_timeout<10:
        pull_timeout = 10
    (return_code, pull_output) = issue_adb_cmd(['pull',device_file_path,local_path], device=device, timeout=pull_timeout)
    if "Permission denied" in pull_output:
        get_device(device).enable_adb_root()
        issue_adb_cmd(['pull',device_file_path,local_path], device=device, timeout=pull_timeout)


def pull_device_files_to_report(device_file_paths, report_paths=None, report_folder=None, device=None):
    """
       Pulls files from a device and places them in the report folder to be
       uploaded with the test report.

       PARAM device_file_paths: A str or a list of strings with the paths to files
           on the device that need to be pulled.
       PARAM report_paths=None: Paths to copy files to. If None is provided, the
           default report path will be used.
       PARAM device=None: The device to target.
    """
    import os
    if isinstance(device_file_paths, str):
        device_file_paths = [device_file_paths]
    if isinstance(report_paths, str):
        report_paths = [report_paths]
    if report_paths==None:
        report_paths = list()
        for i in range(len(device_file_paths)):
            report_paths.append(None)
    # Calculate our default report path
    default_report_path = report_path()
    if report_folder!=None:
        default_report_path = os.path.join(default_report_path, report_folder)
    if os.path.exists(default_report_path)==False:
        os.makedirs(default_report_path)
    for file_path_index in range(len(device_file_paths)):
        # Determine where we will be pulling the file to
        if len(report_paths)<file_path_index or report_paths[file_path_index]==None:
            path_to_report = default_report_path
        else:
            path_to_report = report_paths[file_path_index]
        auto_timeout_adb_pull(device_file_paths[file_path_index], path_to_report)


def execute_pass_fail_log_verify(device_log_dir, tc_parameters, timeout_seconds=300, device=None, ps_search_string=None):
    """
       Executes a generic set of verify steps that many CV tests use. This function
       waits for a pass.log or a fail.log file to appear on the device in
       device_log_dir and then uploads the file to the report. If the file doesn't
       appear in timeout_seconds, an exception is raised.

       PARAM device_log_dir: The directory on the device where pass.log or fail.log will appear.
       PARAM tc_parameters: A test case parameters object. Typically obtained in a use case as self._tc_parameters.
       PARAM timeout_seconds=300: The number of seconds to wait for the log file to appear.
       PARAM device=None: The device to target.
       PARAM ps_search_string=None: A search string to use to verify the test is still running
    """
    # Clean up our device_log_dir to not have a / at the end
    result = "PASS"
    if device_log_dir[-1]=='/':
        device_log_dir = device_log_dir[:-1]
    try:
        log_file = wait_for_file_in_device_dir( device_log_dir,
            ["pass.log","fail.log"], timeout=timeout_seconds,
            polling_interval=30, device=device, ps_search_string=ps_search_string )
        if log_file=="fail.log":
            result = "FAIL"
    except DeviceException as e:
        if e.get_generic_error_message() == DeviceException.TIMEOUT_REACHED:
            result = "TIMEOUT"
        else:
            import traceback
            fail_verdict_and_debug_print("Exception while waiting for pass.log or fail.log to appear: "+traceback.format_exc())
    finally:
        # Collect our logs. Collect anything with *.log, *.out, *.txt
        (return_code, ls_output) = issue_adb_cmd(['shell','ls '+device_log_dir], timeout=10, device=device)
        log_file_list = list()
        collected_file_extensions = [".log", ".out", ".txt", ".png", ".mp4"]
        for filename in ls_output.split('\n'):
            for file_extension in collected_file_extensions:
                if filename.endswith(file_extension):
                    log_file_list.append(device_log_dir+'/'+filename)
                    break
        if len(log_file_list)==0:
            LOGGER.debug("No log files found in %s to collect."%device_log_dir)
        else:
            report_folder = os.path.join(tc_log_dir(tc_parameters), device_log_dir.split('/')[-1])
            pull_device_files_to_report( device_file_paths=log_file_list, report_folder=report_folder, device=device )
    # If we failed, we want to raise an exception
    if result=="FAIL":
        raise DeviceException(DeviceException.OPERATION_FAILED, "File %s/fail.log appeared, indicating failure."%device_log_dir)
    if result=="TIMEOUT":
        raise DeviceException(DeviceException.TIMEOUT_REACHED, "Timeout waiting for pass.log or fail.log to appear in %s."%device_log_dir)
    return log_file


def execute_local_pass_fail_log_verify(local_log_dir, tc_parameters, timeout_seconds=300, polling_interval=30):
    """
       This function is almost identical to execute_pass_fail_log_verify() but it
       looks for the log files to appear in a directory local to the host PC. This
       function executes a generic set of verify steps that many CV tests use and
       waits for a pass.log or a fail.log file to appear in local_log_dir. If the file
       doesn't appear in timeout_seconds, an exception is raised.

       PARAM local_log_dir: The directory on this PC where pass.log or fail.log will appear.
       PARAM tc_parameters: A test case parameters object. Typically obtained in a use case as self._tc_parameters.
       PARAM timeout_seconds=300: The number of seconds to wait for the log file to appear.
       PARAM polling_interval=30: The number of seconds to sleep between checking for a log file.

       RETURNS String containing the path to the log file found or None if no log file appeared.
    """
    start_time = time.time()
    log_file = None
    fail_file_path = os.path.join(local_log_dir, 'fail.log')
    pass_file_path = os.path.join(local_log_dir, 'pass.log')
    LOGGER.debug("Waiting %s seconds for a log file to appear in %s."%(str(timeout_seconds),local_log_dir))
    while time.time()-start_time<timeout_seconds:
        if os.path.exists(fail_file_path):
            log_file = fail_file_path
            break
        if os.path.exists(pass_file_path):
            log_file = pass_file_path
            break
        LOGGER.debug("No log file found. Sleeping %s seconds before trying again."%str(polling_interval))
        time.sleep(polling_interval)
    if '_Reports' not in os.path.realpath(local_log_dir):
        import shutil
        if local_log_dir.endswith(os.sep):
            local_log_dir = local_log_dir.rsplit(os.sep,1)[0]
        if os.sep in local_log_dir:
            testDir = local_log_dir.rsplit(os.sep,1)[1]
        else:
            testDir = local_log_dir
        report_folder = os.path.join(tc_log_dir(tc_parameters), testDir)
        if not os.path.exists(report_folder):
            os.makedirs(report_folder)
        # Copy all log and debug files to the report_folder
        for testFile in os.listdir(local_log_dir):
            if testFile.endswith('.log') or testFile.endswith('.txt'):
                shutil.copy(os.path.join(local_log_dir,testFile),report_folder)
            elif os.path.isdir(os.path.join(local_log_dir,testFile)):
                shutil.copytree(os.path.join(local_log_dir,testFile),os.path.join(report_folder,testFile))
    if log_file == None: # We timed out
        fail_verdict_and_debug_print("Timeout waiting for pass.log or fail.log to appear in %s."%local_log_dir)
        return None
    if log_file.endswith('fail.log'):
        fail_verdict_and_debug_print("File fail.log appeared, indicating failure. Failing test.")
    return log_file


def tc_log_dir(tc_parameters, return_final_path=False):
    """
       Returns the path to the test case log directory. If this is a multiple app
       scenario that has created a 'current_tc_report' directory in the REPORT_PATH,
       that folder is returned. Otherwise, its a folder with this structure:
       REPORT_PATH/YYYY-MM-DD_HHhMM.SS_TESTCASENAME.

       PARAM tc_parameters: A test case parameters object. Typically obtained in a use case as self._tc_parameters.
       PARAM return_final_path=False: If true, the current_tc_report directory is ignored.
    """
    if os.path.exists(os.path.join(report_path(), 'current_tc_report')) and return_final_path!=True:
        return os.path.join(report_path(), 'current_tc_report')
    return os.path.join( report_path(), tc_name(tc_parameters))
        #datetime.datetime.today().strftime("%Y-%m-%d_%Hh%M.%S")+"_"+tc_name(tc_parameters) )


def report_path():
    """Returns the path to the report folder"""
    return DeviceManager().get_global_config().campaignConfig.get("campaignReportTree").get_report_path()


def tc_name(tc_parameters):
    """
       Returns the name of the current test case.

       PARAM tc_parameters: A test case parameters object. Typically obtained in a use case as self._tc_parameters.
    """
    return os.path.basename(tc_parameters.get_name())


def runtime_minutes(default_value=None, tc_parameters=None):
    """
       Looks for the number of minutes that a test case should run from the following
       locations in the following order:
           1. A TCRunTimeMinutes parameter in the campaign XML.
           2. A MAX_RUN_TIME_MINUTES paramter in the test case XML.

       PARAM default_value=None: The default value to return if no value is found in the XML.
    """
    runtime = config_param("TCRunTimeMinutes", default_value=None, default_cast_type=int, tc_parameters=tc_parameters)
    if runtime==None:
        # Check our old parameter name
        runtime = config_param("MAX_RUN_TIME_MINUTES", default_value=None, default_cast_type=int, tc_parameters=tc_parameters)
    if runtime==None:
        runtime = default_value
    return runtime


def bench_config_param(param, default_value=None, default_cast_type=str, device=None):
    """
       Obtains a parameter from the bench configuration XML.

       PARAM param: The name of the parameter in the config XML.
       PARAM default_value=None: The value to return if the parameter isn't found in the XML.
       PARAM default_cast_type=str: The type for the data to be returned.
       PARAM device=None: The name of the device to check in bench config. If not specified, the first device is checked.
    """
    return get_device(device).get_config(param, default_value=default_value, default_cast_type=default_cast_type)


def campaign_config_param(param, default_value=None, default_cast_type=str):
    """
       Obtains a parameter from the campaign configuration XML.

       PARAM param: The name of the parameter in the config XML.
       PARAM default_value=None: The value to return if the parameter isn't found in the XML.
       PARAM default_cast_type=str: The type for the data to be returned.
    """
    return Utils.get_config_value(DeviceManager().get_global_config().campaignConfig,
               "Campaign Config", param, default_value, default_cast_type )


def config_param(param, default_value=None, default_cast_type=str, device=None, tc_parameters=None):
    """
       Looks for a configuration parameter in all of the configuration files. Searches
       in the following order:
         1. Bench_Config File
         2. Campaign Config File
         3. Test Case Config File

       PARAM param: The name of the parameter in the config XML.
       PARAM default_value=None: The value to return if the parameter isn't found in the XML.
       PARAM default_cast_type=str: The type for the data to be returned.
       PARAM device=None: The name of the device to check in bench config. If not specified, the first device is checked.
    """
    param_value = bench_config_param( param, default_value=None, default_cast_type=default_cast_type,
                                      device=device )
    if param_value == None:
        param_value = campaign_config_param( param, default_value=None, default_cast_type=default_cast_type)
    if param_value==None and tc_parameters!=None:
        # Check the test case XML
        param_value = tc_parameters.get_param_value(param, default_value=None, default_cast_type=default_cast_type)
    if param_value == None:
        param_value = default_value
    return param_value


def create_rand_bin_file(file_path, file_size_MB, seed=None):
    """
       Creates a file filled with random data at file_path with size file_size. If seed
       is provided, that seed will be used to create the random data to allow
       repeatability. Returns the seed used to create the data.

       PARAM file_path: The path to where the random data file should be created.
       PARAM file_size_MB: The size of the file to create in megabytes.
       PARAM seed=None: A seed to use for generating the random data. If no seed is
           provided, a random seed will be created and returned.
    """
    import random, sys, struct
    # If no seed was provided create a seed with 50 random characters
    if seed==None:
        seed_ascii_bucket = range(ord('A'),ord('Z')+1)
        seed_ascii_bucket.extend(range(ord('0'),ord('9')+1))
        seed = ""
        for i in range(50):
            seed += chr(random.choice(seed_ascii_bucket))
    random.seed(seed)
    rand_file = open(file_path, 'wb')
    # It would take way too long to make the entire file random, considering
    #  it takes ~10 seconds per MB. Lets create 1 MB of random data and repeat it.
    rand_data = ""
    for i in range(250*1024):
        rand_data += struct.pack('I', random.randint(0,sys.maxint))
    # Repeatedly write our data to the file
    for i in range(file_size_MB):
        rand_file.write(rand_data)
    rand_file.close()
    return seed


def verify_app_installed(package_name, device=None):
    """
       Verifies an app is installed by looking for package_name in the output of the
       command 'pm list packages'.

       PARAM package_name: The name of the package. For example, com.pandora.android.
       PARAM device: The name of the device to issue the command to.
    """
    (return_code, cmd_output) = issue_adb_cmd( ['shell', 'pm list packages | grep '+package_name],
        timeout=10, device=device)
    if return_code==Global.FAILURE or package_name not in cmd_output:
        fail_verdict_and_debug_print("Unable to find package %s in the output of 'pm list packages'."%package_name)
        return False
    return True


def start_android_app(package, activity="Main", action=None, device=None, timeout=10):
    """
       Starts an android application. To obtain the parameters to use in this function,
       look in the logcat output while opening the app. The parameters are combined to
       form a command like this: am start -a android.intent.action.ACTION -n PACKAGE/PACKAGE.ACTIVITY

       PARAM package: The package name. Example: com.pandora.android
       PARAM activity: The activity to start.
       PARAM action: The intent action type.
       PARAM device: The device to target.
    """
    adb_shell_params = ["am", "start"]
    if action != None:
        adb_shell_params.extend(["-a", "android.intent.action."+action])
    adb_shell_params.extend(["-n", package+"/"+package+"."+activity])
    issue_adb_cmd( ['shell'," ".join(adb_shell_params)], timeout=timeout, device=device)


def stop_android_app(package, device=None):
    """
       Stops an android application using the am force-stop command.

       PARAM package: The package name. Example: com.pandora.android
       PARAM device: The device to target.
    """
    issue_adb_cmd( ['shell','am force-stop '+package], timeout=10, device=device)


def wifi_state(device=None):
    """
       Returns the current state of the wifi connection. Possible return values and
       definitions are as follows:
           Disabled = Wifi is not turned on.
           Disconnected = Wifi is turned on but hasn't completely connected to a wireless network.
           Connected = Wifi is on and connected to a wireless network.
           Unknown = We didn't find any key words to determine the state of the wifi connection.

       PARAM device: The name of the device to use.
    """
    (return_code, dumpsys_output) = issue_adb_cmd(['shell','dumpsys wifi | grep curState'], device=device)
    if "ApStaDisabledState" in dumpsys_output:
        return 'Disabled'
    if "NotConnectedState" in dumpsys_output:
        return 'Disconnected'
    if "OnlineState" in dumpsys_output:
        return 'Connected'
    return 'Unknown'


def cellular_signal_properties(device=None):
    """
       Returns a dictionary of properties about the cellular signal.

       PARAM device: The name of the device to use.
    """
    (return_code, dumpsys_output) = issue_adb_cmd(['shell','dumpsys telephony.registry | grep mSignalStrength'], device=device)
    # The dumpsys_output should be a str of numbers separated by spaces like this:
    #  mSignalStrength=SignalStrength: 99 -1 -120 -160 -120 -1 -1 99 -114 -12 60 2147483647 gsm|lte
    strength_props = dumpsys_output.split()
    if len(strength_props)<14:
        raise Exception("Unknown output from command 'dumpsys telephony.registry | grep mSignalStrength'. Expected 13 items separated by whitespace.")
    return_dict = dict()
    return_dict['GSM Signal Strength'] = int(strength_props[1])
    return_dict['GSM Bit Error Rate'] = int(strength_props[2])
    return_dict['CDMA dBm'] = int(strength_props[3])
    return_dict['CDMA Ec/Io'] = int(strength_props[4])
    return_dict['EVDO dBm'] = int(strength_props[5])
    return_dict['EVDO Ec/Io'] = int(strength_props[6])
    return_dict['EVDO SNR'] = int(strength_props[7])
    return_dict['LTE Signal Strength'] = int(strength_props[8])
    return_dict['LTE RSRP'] = int(strength_props[9])
    return_dict['LTE RSRQ'] = int(strength_props[10])
    return_dict['LTE RSSNR'] = int(strength_props[11])
    return_dict['LTE CQI'] = int(strength_props[12])
    return_dict['Connection Type'] = strength_props[13]
    return return_dict


import logging


def create_logger(name="", file_path=None, console_logging_level=logging.DEBUG):
    """
       Creates a Python logger object that prints to the screen and logs to a file
       (if file_path is supplied). To use the logger object that is returned, use
       these functions and more (see the Python documentation):
           log.debug("This is my debug message")
           log.info("This is my info message. Info is less detailed than debug")
           log.warning("Warning is more important than info.")
           log.error("Error is more important than warning.")

       PARAM name="": The name of the logger. Allows you to retrieve a logger previously created.
       PARAM file_path=None: Where to save the log file. If no path is provided, no file will be used.
    """
    log = logging.getLogger(name)
    log.setLevel(logging.DEBUG)
    log_format = logging.Formatter('%(asctime)s - %(name)s - %(message)s',"%Y-%m-%d %H:%M:%S")
    if file_path != None:
        if os.path.exists(os.path.dirname(file_path))==False:
            os.makedirs(os.path.dirname(file_path))
        log_file_handler = logging.FileHandler(filename=file_path)
        log_file_handler.setLevel(logging.DEBUG)
        log_file_handler.setFormatter(log_format)
        log.addHandler(log_file_handler)
    console_log_handler = logging.StreamHandler()
    console_log_handler.setLevel(console_logging_level)
    console_log_handler.setFormatter(log_format)
    log.addHandler(console_log_handler)
    return log


def capture_screenshot(save_path, delete_image_file_from_device=True, device=None):
    """
       Captures a screenshot and saves the image to save_path.

       PARAM save_path: The file path where the screenshot should be saved.
       PARAM delete_image_file_from_device=True: Indicates if the image file
           should be deleted from /data/screenshots after its pulled to save_path.
    """
    device_screencap_file_path = "/data/screenshots/screenshot.png"
    issue_adb_cmd(["shell","if [ ! -d %s ]; then mkdir %s; fi;"%("/data/screenshots","/data/screenshots")],
            timeout=10, device=device )
    issue_adb_cmd(["shell","screencap -p %s"%device_screencap_file_path], timeout=30, device=device)
    auto_timeout_adb_pull(device_screencap_file_path, save_path, device=device)
    if delete_image_file_from_device:
        issue_adb_cmd(["shell","rm %s"%device_screencap_file_path], timeout=10, device=device)


def get_focused_input_window_name(device=None):
    """
       Obtains the name of the window that currently has input focus.
    """
    import re
    (return_code, dumpsys_output) = issue_adb_cmd(["shell","dumpsys input | grep FocusedWindow"], device=device)
    re_match = re.match(".* (.*/.*)}", dumpsys_output)
    if re_match==None or len(re_match.groups())<1:
        LOGGER.debug('Unable to find a app/activity name in the dumpsys input output: '+dumpsys_output)
        return ''
    return re_match.group(1)


def set_focus_lock(appIdentity='Test_App', focusLockFile=os.path.join(Folders.EXECUTION_CONFIG,'gui_focus_lock'), timeout_sec=120):
    """
        Check to see if the GUI focus-lock file already exists. If it doesn't,
        "set" it by creating the lock file. If the focus-lock file does already
        exist, then wait for it to be released.
    """
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
    """
        Release the GUI focus-lock by deleting the lock file to signal that
        other apps can take focus safely.
    """
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
    """
        Delete any focus-lock files that may be lingering on the system during
        setup or teardown
    """
    debug_print = LOGGER.debug

    if os.path.exists(focusLockFile):
        try:
            os.remove(focusLockFile)
        except Exception, e:
            debug_print("%s found a lingering focus-lock from a previous run, but failed to release it! --> %s"%(appIdentity,e))
            return False
    # The lock file either didn't exist or it was removed successfully
    return True


def get_lock_signature(focusLockFile=os.path.join(Folders.EXECUTION_CONFIG, 'gui_focus_lock')):
    """
        Read the contents of the existing lock file, specified by the
        focusLockFile parameter
    """
    debug_print = LOGGER.debug

    try:
        lockFile = open(focusLockFile,'r')
        lockFileSignature = lockFile.read()
        lockFile.close()
    except Exception, e:
        debug_print("There was a problem reading a signature from the lock file: %s"%e)
        return ''

    return lockFileSignature


def set_lock_signature(lockSignature, focusLockFile=os.path.join(Folders.EXECUTION_CONFIG, 'gui_focus_lock')):
    """
        Create the focus lock file and write a signature to it so that the app
        that set it can be identified later
    """
    debug_print = LOGGER.debug

    try:
        lockFile = open(focusLockFile, 'w')
        lockFile.write(lockSignature)
        lockFile.close()
    except OSError, e:
        # A race condition must have occurred. Try again.
        debug_print("It seems that a race condition prevented {0} "
                    "from setting the the focus lock. Trying again...".format(lockSignature))
        return False
    except IOError as e:
        # Somehow, the lock file was not fully released, even though
        # os.path.exists reported that it did not exist
        debug_print("The focus lock was not fully released. Trying again...")
        return False
    except Exception as e:
        debug_print("There was a problem writing a signature to the lock file: {0}".format(e))
        return False

    return True
