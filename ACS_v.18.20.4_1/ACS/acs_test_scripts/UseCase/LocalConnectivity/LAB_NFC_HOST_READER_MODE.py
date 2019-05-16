"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: This file implements the LAB NFC HOST READER MODE UC
:since: 07/08/2012
:author: lpastor
"""

import threading
from acs_test_scripts.Utilities.LocalConnectivityUtilities import generate_random_string
from UtilitiesFWK.Utilities import Global, str_to_bool_ex
from acs_test_scripts.UseCase.LocalConnectivity.LAB_NFC_BASE import LabNfcBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
import time
import numpy
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabNfcHostReaderMode(LabNfcBase):

    """
    Lab NFC write/read tag test
    """

    _EMPTY_VALUE = "EMPTY"
    _RANDOM_VALUE = "RANDOM"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabNfcBase.__init__(self, tc_name, global_config)

        # Instantiate the Secondary Report object
        self._secondary_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())

        # status for the test setup
        self.__flight_mode_status = self._tc_parameters.get_param_value("FLIGHT_MODE_STATUS", "APM_NONE")

        # Get UECmdLayer
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._appmgmt_api = self._phone_ref.get_uecmd("AppMgmt")

        # Get tag to manage from test case xml file
        self._tag_type = str(self._tc_parameters.get_param_value("TAG_TYPE"))

        # Get RTD to write in tag from test case xml file
        self._record_type_description = str(self._tc_parameters.get_param_value("RTD_TYPE"))

        # Get DATA to write in tag from test case xml file
        self._data_to_write = str(self._tc_parameters.get_param_value("DATA"))

        # Get DATA to write in tag from test case xml file
        self._should_erase = str_to_bool_ex(self._tc_parameters.get_param_value("ERASE"))
        if self._should_erase is None:
            self._should_erase = False

        # Get NFC Tags application package name
        self._tag_app = str(self._dut_config.get("TagsAppPackageName"))
        # Get Browsers application package name (need to disable in order to be able to read URI tags)
        self._browser_app = str(self._dut_config.get("BrowserAppPackageName"))
        self._chrome_app = str(self._dut_config.get("ChromeAppPackageName"))

        self._nfc_robot_param = global_config.benchConfig.get_parameters("NFC_ROBOT1")

        # Get "Beam control check" from test case xml file (set ON, set OFF)
        self._beam_checked = self._get_beam_config()

        # Initialize up down robot values
        self._tag_up = "0"
        self._tag_down = "0"
        self._read_max_duration = "5"
        self._write_max_duration = "5"

        # Measure read time
        self._read_time_list = list()
        self._current_iteration_num = 0

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabNfcBase.set_up(self)

        # air plane mode status
        if self.__flight_mode_status is not "APM_NONE":

            if self.__flight_mode_status == "APM_ONOFF_NFC_OFF":
                self._nfc_api.nfc_disable()

            self._logger.info("Turn flight mode ON")
            self._networking_api.set_flight_mode(1)

            if not self._networking_api.get_flight_mode():
                msg = "Unable to activate flight mode"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if self.__flight_mode_status == "APM_NFC_ON":
                self._logger.info("Activate NFC after turning flight mode ON")
                self._nfc_api.nfc_enable()

            if self.__flight_mode_status == "APM_ONOFF_NFC_ON":
                #reactivate
                self._logger.info("Turn flight mode OFF")
                self._networking_api.set_flight_mode(0)

        if self._record_type_description not in ("RTD_TEXT", "RTD_SP", "RTD_URI"):
            msg = "Set wrong RTD type. Could only be RTD_TEXT, RTD_SP or RTD_URI. "
            msg += "Read Value: " + self._data_to_write
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Get the time in seconds during which the Robot presents the NFC tag to the DUT
        self._read_max_duration = self._nfc_robot_param.get_param_value("NfcReadTime", self._read_max_duration)
        self._write_max_duration = self._nfc_robot_param.get_param_value("NfcWriteTime", self._write_max_duration)

        # Get the TAG coordinates
        if self._tag_type == "TOPAZ":
            tag_x = self._nfc_robot_param.get_param_value("TopazX")
            tag_y = self._nfc_robot_param.get_param_value("TopazY")
            self._tag_up = self._nfc_robot_param.get_param_value("TopazUp")
            self._tag_down = self._nfc_robot_param.get_param_value("TopazDown")
        elif self._tag_type == "MIFARE_ULTRALIGHT":
            tag_x = self._nfc_robot_param.get_param_value("MifareUltralightX")
            tag_y = self._nfc_robot_param.get_param_value("MifareUltralightY")
            self._tag_up = self._nfc_robot_param.get_param_value("MifareUltralightUp")
            self._tag_down = self._nfc_robot_param.get_param_value("MifareUltralightDown")
        elif self._tag_type == "MULC":
            tag_x = self._nfc_robot_param.get_param_value("MifareUltralightCX")
            tag_y = self._nfc_robot_param.get_param_value("MifareUltralightCY")
            self._tag_up = self._nfc_robot_param.get_param_value("MifareUltralightCUp")
            self._tag_down = self._nfc_robot_param.get_param_value("MifareUltralightCDown")
        elif self._tag_type == "FELICA":
            tag_x = self._nfc_robot_param.get_param_value("FelicaX")
            tag_y = self._nfc_robot_param.get_param_value("FelicaY")
            self._tag_up = self._nfc_robot_param.get_param_value("FelicaUp")
            self._tag_down = self._nfc_robot_param.get_param_value("FelicaDown")
        elif self._tag_type == "DESFIRE_A":
            tag_x = self._nfc_robot_param.get_param_value("DesfireX")
            tag_y = self._nfc_robot_param.get_param_value("DesfireY")
            self._tag_up = self._nfc_robot_param.get_param_value("DesfireUp")
            self._tag_down = self._nfc_robot_param.get_param_value("DesfireDown")
        elif self._tag_type == "TYPE4_B":
            tag_x = self._nfc_robot_param.get_param_value("Type4BX")
            tag_y = self._nfc_robot_param.get_param_value("Type4BY")
            self._tag_up = self._nfc_robot_param.get_param_value("Type4BUp")
            self._tag_down = self._nfc_robot_param.get_param_value("Type4BDown")
        elif self._tag_type == "MIFARE_CLASSIC":
            tag_x = self._nfc_robot_param.get_param_value("MifareClassicX")
            tag_y = self._nfc_robot_param.get_param_value("MifareClassicY")
            self._tag_up = self._nfc_robot_param.get_param_value("MifareClassicUp")
            self._tag_down = self._nfc_robot_param.get_param_value("MifareClassicDown")
        elif self._tag_type == "MIFARE4K":
            tag_x = self._nfc_robot_param.get_param_value("MifareClassic4KX")
            tag_y = self._nfc_robot_param.get_param_value("MifareClassic4KY")
            self._tag_up = self._nfc_robot_param.get_param_value("MifareClassic4KUp")
            self._tag_down = self._nfc_robot_param.get_param_value("MifareClassic4KDown")
        elif self._tag_type == "LIBRARY_TAG":
            tag_x = self._nfc_robot_param.get_param_value("LibraryX")
            tag_y = self._nfc_robot_param.get_param_value("LibraryY")
            self._tag_up = self._nfc_robot_param.get_param_value("LibraryUp")
            self._tag_down = self._nfc_robot_param.get_param_value("LibraryDown")
        elif self._tag_type == "CENTER":
            tag_x = self._nfc_robot_param.get_param_value("CenterX")
            tag_y = self._nfc_robot_param.get_param_value("CenterY")
            self._tag_up = self._nfc_robot_param.get_param_value("CenterUp")
            self._tag_down = self._nfc_robot_param.get_param_value("CenterDown")
        else:
            msg = "Unknown tag type! Read value: " + self._tag_type
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # disable "Tags" and browsers built-in apps
        if self._tag_app in ["None", ""]:
            msg = "NFC not available on this device!"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.FEATURE_NOT_AVAILABLE, msg)

        self._appmgmt_api.app_enable_disable(self._tag_app, False)
        self._appmgmt_api.app_enable_disable(self._browser_app, False)
        self._appmgmt_api.app_enable_disable(self._chrome_app, False)

        self._robot_positioning(tag_x, tag_y, "null", "null")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabNfcBase.run_test(self)

        # Increase the iteration index
        self._current_iteration_num += 1

        if self._data_to_write == self._RANDOM_VALUE:
            self._logger.info("Generate random string")
            value_to_compare = generate_random_string(10)
        else:
            value_to_compare = self._data_to_write

        self._write_read_action(value_to_compare, True)

        if self._should_erase:
            self._write_read_action(self._EMPTY_VALUE)

        # If Beam to be checked
        if self._beam_used and self._beam_checked:
            read_nfc_beam_status = self._nfc_api.get_nfc_beam_status()

            # Compare beam status to the one desired
            if read_nfc_beam_status != self._beam_wished_value:
                msg = "Unexpected result! Read beam value is %s instead of %s" % (
                    str(read_nfc_beam_status),
                    str(self._beam_wished_value))
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabNfcBase.tear_down(self)

        # if flight mode is activate
        if self._networking_api.get_flight_mode():
            self._networking_api.set_flight_mode(0)

        # In case of failure, restore robot initial position
        self._robot_positioning("null", "null", self._tag_up, "null")

        # enable "Tags" and browser built-in apps
        self._appmgmt_api.app_enable_disable(self._tag_app, True)
        self._appmgmt_api.app_enable_disable(self._browser_app, True)
        self._appmgmt_api.app_enable_disable(self._chrome_app, True)

        return Global.SUCCESS, "No errors"

    def _write_read_action(self, data, read_time_measurement=False):
        """
        Perform a write action, and a read action on the TAG.

        :type data: String
        :param data: data to write. Can be "EMPTY" to erase the TAG.
        :type read_time_measurement: Boolean
        :param read_time_measurement: if true, the read time is added to the read time list
                                      in order to be able to compute the read time mean
        """
        # Initialize the write and the read threads
        self._logger.info("Positioning robot before launching test")
        self._robot_positioning("null", "null", self._tag_up, "null")
        writing_thread = threading.Thread(target=self._robot_positioning,
                                          args=('null', 'null', self._tag_down, 'null'))
        reading_thread = threading.Thread(target=self._robot_positioning,
                                          args=('null', 'null', self._tag_down, 'null'))

        report_msg = "Write (%s) test." % data

        try:
            self._logger.info("Write \"" + data + "\" in NFC tag")

            writing_thread.start()
            self._nfc_api.write_nfc_tag(self._record_type_description, data)
            writing_thread.join()

            self._robot_positioning("null", "null", self._tag_up, "null")

            self._logger.info("Read data from NFC tag")
            reading_thread.start()
            self._logger.info("Start read time measurement")
            start_time = time.time()
            read_data_from_tag = self._nfc_api.read_nfc_tag()
            stop_time = time.time()
            self._logger.info("Stop read time measurement")
            # Store the time to read the TAG
            if read_time_measurement:
                read_time = stop_time - start_time
                self._read_time_list.append(read_time)
                report_msg += " Read TAG time: %.2f sec." % read_time
                if self.get_b2b_iteration() == self._current_iteration_num:
                    # This is the last iteration of back to back test
                    report_msg += self.__compute_final_report_msg()
            reading_thread.join()

            self._logger.info("Compare read and written values...")
            msg = 'Read value is "' + read_data_from_tag + '" when written one is "' + data + '"'
            if read_data_from_tag != data:
                msg = "Unexpected result! " + msg
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            self._logger.info(msg)

            self._secondary_report.add_result(report_msg,
                                              SecondaryTestReport.verdict.PASS, "Test pass",
                                              self.get_name(), self.tc_order)
        except:
            if read_time_measurement and self.get_b2b_iteration() == self._current_iteration_num:
                # This is the last iteration of back to back test
                report_msg += self.__compute_final_report_msg()
            self._secondary_report.add_result(report_msg,
                                              SecondaryTestReport.verdict.FAIL, "Test fail",
                                              self.get_name(), self.tc_order)
            raise
        finally:
            writing_thread.join()
            try:
                reading_thread.join()
            except:
                pass

            self._logger.info("Ensure robot is at TOP position for the end")
            self._robot_positioning("null", "null", self._tag_up, "null")

    def __compute_final_report_msg(self):
        """
        Compute standard deviation, mean and verdict for the read time measurements
        """
        final_report = ""
        if self._read_time_list:
            mean = float(numpy.mean(self._read_time_list))
            std_deviation = float(numpy.std(self._read_time_list))
            min_value = float(min(self._read_time_list))
            max_value = float(max(self._read_time_list))
            final_report = " Mean: %.2f - Std dev: %.2f - Min: %.2f - Max: %.2f" \
                % (mean, std_deviation, min_value, max_value)
        return final_report
