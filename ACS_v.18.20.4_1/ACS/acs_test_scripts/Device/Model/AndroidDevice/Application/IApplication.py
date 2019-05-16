"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: This script implements the interface for applications
:since: 18/02/2013
:author: pbluniex
"""
import os
import time
import math
import numpy
import scipy.stats
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
from lxml import etree
from UtilitiesFWK.Utilities import global_to_bool


class IApplication(object):
    """
    Abstract class that defines the basis operations of application file
    """

    __device = None

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        self.__device = device
        self._logger = device.get_logger()
        self._touch_screen_event_file = device.get_touchsreen_event_file()
        self._global_config = None
        self._dut_config = None
        self._execution_config_path = None
        self._additionnals = None
        self._arguments = None
        self._application_uri = None
        self._path_ref = None
        self._path_base = None
        self._phonesystem = device.get_uecmd("PhoneSystem")
        self._networking = device.get_uecmd("Networking")
        self._display = device.get_uecmd("Display")
        self._keyevent = device.get_uecmd("KeyEvent")
        self._system_api = device.get_uecmd("System")
        self._app_api = device.get_uecmd("AppMgmt")
        self._results = {}
        self.is_lower_better = True
        self._benchmark_name = self.__class__.__name__.lower()
        result_folder = "%s_result" % self._benchmark_name
        report_dir = self._get_report_tree()
        report_dir.create_subfolder(result_folder)
        self._result_file = None
        self._run_no = 0
        self.__report_path = report_dir.get_subfolder_path(result_folder)
        equipment_mgmt = EquipmentManager()
        self.io_cards = equipment_mgmt.get_io_cards()
        self.io_cards = equipment_mgmt.get_io_cards()
        self.hard_reboot = False
        self._additionnals = None
        self._sysdebug_apis = None
        self._need_restart = False
        for key in self.io_cards.keys():
            if "IO_CARD" in key and self.io_cards[key] is not None:
                self.hard_reboot = True
                self.io_card = self.io_cards[key]
                break

    def __split_file_extension(self, filename):
        """
        Split the basename and the extension of a file

        :type filename: str
        :param filename: The filename (basename) of a apk/zip/tar/tgz/tar.gz or
                         tar.bz2 file

        :rtype: tuple
        :return: (basename, extension) tuple of filename
        """
        double_extensions = ["tar.gz", "tar.bz2"]

        root, ext = os.path.splitext(filename)
        if any([filename.endswith(x) for x in double_extensions]):
            root, first_ext = os.path.splitext(root)
            ext = first_ext + ext

        return root, ext

    def __get_device_informations(self):
        """
        Get the information of the device (platform and OS version)

        :rtype: tuple
        :return: (platform, osversion) tuple of the device
        """
        dmodel = self._dut_config.get("Name").split("-")
        platform = dmodel[0]
        os_version = ""
        if len(dmodel) > 2:
            os_version = "-".join(dmodel[2:])

        return platform, os_version

    def _get_device_logger(self):
        """
        Get the device logger

        :rtype: Logger
        :return: The logger of the device
        """
        return self.__device.get_device_logger()

    def _get_report_tree(self):
        """
        Get the report tree directory

        :rtype: CampaignReportTree object
        :return: the instance of the campaign report tree
        """
        return self.__device.get_report_tree()

    def _fetch_file(self):
        """
        Get score for benchmark
        """
        if self._result_file is None:
            return None

        localfile = os.path.join(self.__report_path,
                                 "result_%s_r%d.txt" %
                                 (self._benchmark_name, self._run_no))

        self.adb_pull("%s" % self._result_file, localfile, 60)
        resfile = open(localfile)
        return resfile.read()

    def _fetch_result(self):
        """
        Fetch result for single run : To be overloaded
        """
        pass

    def _wait_for_application(self, name, timeout):
        """
        Wait until the application ends

        :type name: str
        :param name: The name of the application

        :type timeout: integer
        :param timeout: The timeout beyond the application should end
        """
        # Get Pids of processes
        output = self.adb_shell("ps|grep %s|sed 's/[[:space:]]\+/ /g' |cut -d' ' -f2" % name, 2)
        pids = output.splitlines()
        wait_time = math.sqrt(timeout)
        wait_end = time.time() + timeout
        while len(pids) > 0 and time.time() < wait_end:
            output = self.adb_shell("ps %s|tail -n1|sed 's/[[:space:]]\+/ /g' |cut -d' ' -f2" % pids[0], 3)
            # If output is 'PID', the watched process ends up. It is removed
            # from the list.
            if output == "PID":
                pids.pop(0)
            else:
                time.sleep(wait_time)

        if len(pids) > 0:
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Timeout reached for %s" % name)

    def wait(self, _timeout):
        """
        Wait until the application ends

        :type timeout: integer
        :param timeout: Time in second beyond the application should end
        """
        pass

    def app_is_alive(self):
        """
        Checks if the application is still alive and can be killed
        """
        return True

    def go_back_home(self):
        """
        Go back to the home screen of the application.
        """
        pass

    def drive(self):
        """
        Drive the application
        """
        pass

    def start(self):
        """
        Start the application
        """
        pass

    def stop(self):
        """
        Stop the application
        """
        pass

    def fetch_score(self):
        """
        Get score of benchmark
        """
        self._fetch_result()

    def run_begin(self, _parameters=None):
        """
        begin running the application asynchronously
        Can be overloaded to specialize behavior

        """
        self.start()
        self.drive()

    def run_end(self, _parameters=None):
        """
        stop running the application
        """
        self.stop()

    def run(self, tc_parameters=None):
        """
        run the application.
        default pattern here is to loop on {drive wait fetch_results}
        based on tc_parameters. Can be overloaded to modify behaviour

        :type tc_parameters:  Core.TCParameters
        :param tc_parameters: Test case parameters
        """

        if tc_parameters is None:
            timeout = 60
            thermal_timeout = 180
            measure_nb = 1
            loop_mode = "LOOP_MODE"
        else:
            timeout = tc_parameters.get_param_value("TIMEOUT", 60, int)
            thermal_timeout = tc_parameters.get_param_value("THERMAL_TIMEOUT", 180, int)
            measure_nb = tc_parameters.get_param_value("MEASURE_NB", 1, int)
            failures_nb = tc_parameters.get_param_value("FAILURES_ALLOWED_NB", 1, int)
            loop_mode = tc_parameters.get_param_value("LOOP_MODE")

        measure = 0
        measure_ko = 0
        if (loop_mode is not None) and ("no_restart" in loop_mode.lower()):
            self.start()
            self._need_restart = False
            try:
                while measure < measure_nb:
                    measure += 1
                    try:
                        self._logger.debug("IApplication - Iteration start")
                        # wait for cooling down
                        self.wait_for_cooldown(thermal_timeout)
                        self.drive()
                        self.wait_and_check(timeout)
                        self.fetch_score()
                    except DeviceException as ex:
                        measure_ko += 1
                        measure -= 1
                        if measure_ko > failures_nb:
                            raise ex
                        else:
                            self._logger.error("Exception catched on iteration %d, but we go on :" % (measure + measure_ko))
                            self._logger.error(str(ex))
                    finally:
                        # after last run, application must not be relaunched
                        if measure < measure_nb:
                            if self._need_restart:
                                self.start()
                                self._need_restart = False
                            else:
                                self.go_back_home()
                        self._logger.debug("IApplication - Iteration end")
            finally:
                self.stop()

        else:
            while measure < measure_nb:
                measure += 1
                try:
                    self._logger.debug("IApplication - Iteration start")
                    self.wait_for_cooldown(thermal_timeout)
                    self.start()
                    self.drive()
                    self.wait_and_check(timeout)
                    self.fetch_score()
                except DeviceException as ex:
                    measure_ko += 1
                    measure -= 1
                    if measure_ko > failures_nb:
                        raise ex
                    else:
                        self._logger.error("Exception catched on iteration %d, but we go on :" % (measure + measure_ko))
                        self._logger.error(str(ex))
                finally:
                    self.stop()
                    self._logger.debug("IApplication - Iteration end")

    def wait_and_check(self, timeout):
        try:
            self.wait(timeout)
        except DeviceException as ex:
            # raise a different exception if there is a timeout and the app is not running
            error_msg = str(ex)
            if "timeout" in error_msg.lower():
                if self.app_is_alive():
                    self._logger.error("Timeout but the application is still alive")
                    raise ex
                else:
                    self._need_restart = True
                    self._logger.error(error_msg)
                    msg = "Application exited before returning the result"
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                self._logger.error(error_msg)
                raise ex

    def get_score(self, stat_type="MEDIAN"):
        """
        Get the score of the application
        """
        if stat_type == "ARITHMETIC_MEAN":
            functor = numpy.mean
        elif stat_type == "GEOMETRIC_MEAN":
            functor = scipy.stats.mstats.gmean
        else:
            stat_type = "MEDIAN"
            functor = numpy.median

        # let's make sure there is at least one result named score
        if "score" not in self._results.keys():
            if len(self._results.keys()) > 1:
                # duplicate first entry as "score"
                key0 = self._results.keys()[0]
                self._results["score"] = self._results[key0]
            else:
                # rename first entry as "score"
                key0 = self._results.keys()[0]
                self._results["score"] = self._results[key0]
                del self._results[key0]

        # now we can build our report
        xmlresult = etree.Element("scores")
        keys = sorted(self._results.keys())
        if "score" in keys:
            keys.remove("score")
            keys.insert(0, "score")

        for key in keys:
            value = self._results[key]

            xmlnode = etree.Element("item")
            result = str(functor(value))
            runs = ";".join([str(x) for x in value])
            xmlnode.attrib["name"] = key
            xmlnode.attrib["value"] = str(result)
            xmlnode.attrib["runs"] = runs
            self._logger.info("%s: %s => %s" % (key, str(value), result))

            xmlresult.append(xmlnode)

            if key == "score":
                self._logger.info("Result %s score : %s (%s)"
                                  % (type(self).__name__, result, stat_type))
        return xmlresult

    def pre_install(self, execution_config_path, global_config, dut_config, sysdebug_apis=None):
        """
        Pre installation actions

        :type execution_config_path: str
        :param execution_config_path: The path of the configuration of ACS

        :type global_config: Dictionary
        :param global_config: Global configuration of ACS

        :type dut_config: Dictionary
        :param dut_config: The configuration of the DUT
        """
        self._logger.debug("IApplication - pre_install begins")
        self._execution_config_path = execution_config_path
        self._logger.debug("IApplication - Execution config path: %s" % str(self._execution_config_path))
        self._global_config = global_config
        self._dut_config = dut_config
        self._logger.debug("IApplication - DUT config: %s" % str(self._dut_config))
        self._sysdebug_apis = sysdebug_apis
        self._logger.debug("IApplication - Sysdebug API: %s" % str(self._sysdebug_apis))
        self._path_base = self._dut_config.get("ApplicationDirectory")
        self._logger.debug("IApplication - Base path: %s" % str(self._path_base))
        self._logger.debug("IApplication - pre_install ends")

    def post_install(self):
        """
        Pre installation actions
        """
        pass

    def install(self, appuri, additionnals=None, arguments=None, url=None, _destination=None):
        """
        Install the application on the device

        :type appuri: String
        :param appuri: The full path to the application file

        :type additionals: String
        :param additionals: The full path of additionals elements to run the
                             application

        :type arguments: String
        :param arguments: The arguments of the application. May be everything
                          the application need to run.

        :type destination: String
        :param destination: The directory where the application will be installed
        """
        self._logger.debug("IApplication - install begins")
        if self._path_ref is None:
            self._logger.debug("IApplication - install: app URI: %s" % str(appuri))
            self._application_uri = appuri
        else:
            self._logger.debug("IApplication - install: path base: %s" % str(self._path_base))
            self._logger.debug("IApplication - install: path ref: %s" % str(self._path_ref))
            self._logger.debug("IApplication - install: appuri: %s" % str(appuri))
            self._application_uri = os.path.join(self._path_base, self._path_ref, appuri)
            self._logger.debug("IApplication - install: App URI: %s" % str(self._application_uri))

        self._logger.debug("IApplication - install: Additionals: %s" % str(additionnals))
        self._additionnals = additionnals
        self._logger.debug("IApplication - install: Arguments: %s" % str(arguments))
        self._arguments = arguments
        self._logger.debug("IApplication - install ends")

    def uninstall(self):
        """
        Uninstall an application
        """
        pass

    def adb_shell(self, command, timeout):
        """
        Facade to run command on the device
        """
        cmd = "adb shell %s" % command
        return self.__device.run_cmd(cmd, timeout)[1]

    def run_cmd(self, command, timeout):
        """
        Facade to run command on the device
        """
        return self.__device.run_cmd(command, timeout)[1]

    def push_file(self, filename, destination=None, timeout=0):
        """
        Push file on the device
        """
        self.__device.push(filename, destination, timeout)

    def adb_install(self, filename, destination=None):
        """
        Install file on the device
        :type filename: str
        :param filename: The file to install on the device

        :type destination: str
        :param destination: Device destination directory to install the file
        """
        # destination no more used
        status, msg = self._app_api.install_device_app(filename)
        if not global_to_bool(status):
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Failed to install application : '%s'" % msg)

    def adb_uninstall(self, name, timeout):
        """
        Uninstall application on the device using adb
        """
        cmd = "adb uninstall %s" % name
        self.__device.run_cmd(cmd, timeout)

    def adb_pull(self, device_file, local_file, timeout):
        """
        Pull file from device

        :type device_file: str
        :param device_file: Path to the file to retrieve from the device

        :type local_file: str
        :param local_file: Local path where to copy the file

        :type timeout: integer
        :param timeout: Timeoutbeyond the file should have been downloaded
        """
        self.__device.pull(device_file, local_file, timeout)

    def disable_jni(self):
        """
        Disable JNI on the DUT
        """
        cmd = "setprop dalvik.vm.checkjni false"
        self.adb_shell(cmd, 3)

    def clear_cache(self):
        """
        Clear cache on the DUT
        """
        cmd = "sync && echo 3 > /proc/sys/vm/drop_caches"
        self.adb_shell(cmd, 30)

    def is_device_booted(self):
        """
        return True if the board is booted
        """
        is_booted = False
        if self.__device:
            is_booted = self.__device.is_booted()
        return is_booted

    def shutdown_device(self):
        """
        shutdown the device
        """
        if self.__device is None:
            return

        if self.__device.is_booted():
            self.__device.hard_shutdown(wait_for_board_off=True)

    def boot_device(self):
        """
        boot the device
        """
        if self.__device is None:
            return
        return self.__device.switch_on(simple_switch_mode=True)

    def unplug_usb(self):
        """
        unplug the usb cable
        """
        self._phonesystem.display_off()
        time.sleep(2)
        # self.__device.disconnect_board()
        self.io_card.usb_host_pc_connector(False)
        time.sleep(2)

    def plug_usb(self):
        """
        plug the usb cable
        """
        self.io_card.usb_host_pc_connector(True)
        time.sleep(2)
        # self.__device.connect_board()
        time.sleep(2)

    def short_press_power(self):
        """
        short press the power button
        """
        self._logger.info("++ short press !")
        start = time.time()
        self.io_card.press_power_button(0.3)
        end = time.time()
        return start + (end - start - 0.3)

    def reboot_device(self):
        """
        boot the device
        """
        if self.hard_reboot:
            self._logger.info("++ Hard reboot !")
            # Disconnect device before shutting down takes effect
            self.__device.disconnect_board()
            self.io_card.usb_host_pc_connector(False)
            time.sleep(1)
            self.io_card.press_power_button(5)
            time.sleep(10)
            start = time.time()
            self.io_card.press_power_button(0.1)
            end = time.time()
            self._logger.info("cmde duration = " + str(end - start))
            time.sleep(1)
            self.io_card.usb_host_pc_connector(True)
            self.__device.connect_board()
            self._logger.info("++ Hard reboot : END")
        else:
            self._logger.info("++ Soft reboot !")
            start = time.time()
            self.__device.reboot("MOS", True, 60, False, False)
            end = time.time()
        return start + (end - start - 0.1)

    def wait_for_cooldown(self, thermal_timeout):
        """
        Waiting for cooling down

        :return: true if we reach the temperature before timeout
                 flase if the timeout has been reached
        """
        if self._sysdebug_apis is not None:
            self._phonesystem.display_off()
            Timeout = time.time() + float(thermal_timeout)
            while (not self._sysdebug_apis.synchronize()) and (time.time() < Timeout):
                time.sleep(10)
            if time.time() >= Timeout:
                self._logger.info("Thermal Timeout !!!")
            self._phonesystem.display_on()
            stabilization_time = 5
            self._logger.debug("Wait for %s seconds after display_on." % str(stabilization_time))
            time.sleep(stabilization_time)
