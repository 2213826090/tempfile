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
:summary: This file implements the LIVE WIFI FTP UC
:since: 26/08/2010
:author: szhen11
"""
import os
import re
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global, is_number


class LabWifiFtp(LabWifiBase):

    """
    Lab Wifi ftp test.
    """
    _FTP_THROUGHTPUT_PARSING_PATTERN = r'throughput: *([0-9]*) *kBytes/sec'

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        # Read the DL_FILE value from UseCase xml Parameter
        self._dlfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("DL_FILE", ""))
        # Read the UL_FILE value from UseCase xml Parameter
        self._ulfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("UL_FILE", ""))
        self._xfer_timeout = int(self._tc_parameters.get_param_value("XFER_TIMEOUT"))

        self._xfer_40_to_20mhz_ratio = str(self._tc_parameters.get_param_value("XFER_40MHz_TO_20MHz_RATIO", ""))
        if is_number(self._xfer_40_to_20mhz_ratio):
            self._xfer_40_to_20mhz_ratio = float(self._xfer_40_to_20mhz_ratio)
        else:
            self._xfer_40_to_20mhz_ratio = 1.5

        self._xfer_mimo_to_siso_ratio = str(self._tc_parameters.get_param_value("XFER_MIMO_TO_SISO_RATIO", ""))
        if is_number(self._xfer_mimo_to_siso_ratio):
            self._xfer_mimo_to_siso_ratio = float(self._xfer_mimo_to_siso_ratio)
        else:
            self._xfer_mimo_to_siso_ratio = 1.5

        self.__1st_iteration = True

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up_without_connect(self)

        if self._direction not in ["DL", "UL"]:
            self._error.Msg = "%s is not a known xfer direction" % self._direction
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Ensure the AP configuration is correct and connect the DUT to the AP
        if not self.__1st_iteration:
            if self._bandwidth == "40":
                # Restore the Bandwidth to 40 MHz
                self._set_ap_bandwidth('40', False)
            if self._wifi_mimo:
                # Restore MIMO activation on AP
                self._set_ap_mimo(True, False)
        else:
            self.__1st_iteration = False

        # Connect the DUT to AP with connection check if specific feature(s) is(are) enabled (40MHz, MIMO)
        self._wifi_connect_dut()

        # Run the FTP tranfer with the parameters extracted from the XML TC file
        ftp_result = self._launch_ftp_transfer()

        # Retrieve the FTP throughput
        throughput = re.findall(self._FTP_THROUGHTPUT_PARSING_PATTERN, ftp_result[1])
        if len(throughput) > 0:
            throughput = int(throughput[0])
        else:
            throughput = -1

        # If enhanced features (like 40MHz and/or MIMO) are activated,
        # we need to compare throughput without these features
        self._check_enhanced_features(throughput)

        # Disconnect DUT from AP
        self._networking_api.wifi_disconnect(self._ssid)

        return ftp_result

#------------------------------------------------------------------------------

    def _launch_ftp_transfer(self):
        """
        Launch the FTP transfer

        :rtype: list
        :return: status of the FTP transfer. You can extract the ftp throughput from the 2nd element of the list
        """
        if self._direction == "DL":
            file2transfer = self._dlfilename
            direction = self._uecmd_types.XFER_DIRECTIONS.DL  # pylint: disable=E1101
        else:
            # UL
            file2transfer = self._ulfilename
            direction = self._uecmd_types.XFER_DIRECTIONS.UL  # pylint: disable=E1101

        # Retrieve IPV4 address
        ip_address = self._networking_api.get_wifi_ip_address()
        self._logger.info("FTP transfer " + self._direction + " for " + str(file2transfer) + "...")
        ftp_result = self._networking_api.ftp_xfer(direction,
                                                   self._ftp_ip_address,
                                                   self._ftp_username,
                                                   self._ftp_password,
                                                   file2transfer,
                                                   self._xfer_timeout,
                                                   self._device.get_ftpdir_path(),
                                                   client_ip_address=ip_address)
        return ftp_result

#------------------------------------------------------------------------------

    def _check_enhanced_features(self, throughput):
        """
        If enhanced features (like 40MHz and/or MIMO) are activated,
        we need to compare throughput with these features disabled.

        :type throughput: int
        :param throughput: throughput measured with the parameters defined in the XML TC file
        """
        # If one of the enhanced features is activated, we need to have a valid throughput to compare to
        if (self._bandwidth == "40" or self._wifi_mimo) and throughput < 0:
            msg = "Unable to retrieve throughput from ACS agent. Output: " + str(throughput)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if self._bandwidth == "40":
            # If initial test has been done in 40MHz, we need to compare to a 20MHz transfer
            throughput_20mhz = self._check_40mhz_throughtput(throughput)
        else:
            throughput_20mhz = throughput

        if self._wifi_mimo:
            # Compare MIMO with SISO (using 20MHz connection, as requested in the test spec)
            self._check_mimo_throughtput(throughput_20mhz)

#------------------------------------------------------------------------------

    def _check_40mhz_throughtput(self, throughput_40mhz):
        """
        Compare the FTP throughput using 40MHz connection with a 20MHz FTP transfer

        :type throughput_40mhz: int
        :param throughput_40mhz: throughput measured at 40 MHz
        :rtype: int
        :return: throughput measured at 20 MHz
        """
        # set ap to 20 MHz
        self._set_ap_bandwidth('20', True)

        # Run the FTP transfer
        result = self._launch_ftp_transfer()

        throughput_20mhz = re.findall(self._FTP_THROUGHTPUT_PARSING_PATTERN, result[1])

        if len(throughput_20mhz) == 0:
            msg = "Unable to retrieve throughput from ACS agent, after 20MHz transfer test"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        throughput_20mhz = int(throughput_20mhz[0])

        self._logger.info("Measured throughput at 40MHz " + str(throughput_40mhz) + " kBytes/sec")
        self._logger.info("Measured throughput at 20MHz " + str(throughput_20mhz) + " kBytes/sec")
        if throughput_40mhz <= self._xfer_40_to_20mhz_ratio * throughput_20mhz:
            msg = "40MHz throughput is not higher enough compared to 20MHz throughput"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return throughput_20mhz

#------------------------------------------------------------------------------

    def _check_mimo_throughtput(self, throughput_mimo):
        """
        Compare the FTP throughput using MIMO with a SISO FTP transfer
        Test spec says that we only compare MIMO throughput, using 20MHz connection.

        :type throughput_mimo: int
        :param throughput_mimo: throughput measured using MIMO.
        """
        # Disable MIMO
        self._set_ap_mimo(False, True)

        # Run the FTP transfer
        result = self._launch_ftp_transfer()

        throughput_single = re.findall(self._FTP_THROUGHTPUT_PARSING_PATTERN, result[1])

        if len(throughput_single) == 0:
            msg = "Unable to retrieve throughput from ACS agent, after SISO transfer test"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        throughput_single = int(throughput_single[0])

        self._logger.info("Measured throughput with MIMO " + str(throughput_mimo) + " kBytes/sec")
        self._logger.info("Measured throughput with SISO " + str(throughput_single) + " kBytes/sec")
        if throughput_mimo <= self._xfer_mimo_to_siso_ratio * throughput_single:
            msg = "MIMO throughput is not higher enough compared to SISO throughput"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

#------------------------------------------------------------------------------

    def _set_ap_bandwidth(self, bandwidth, do_connect):
        """
        Set the AP bandwidth to bandwidth

        :type bandwidth: str
        :param bandwidth: bandwidth to be set
        :type do_connect: boolean
        :param do_connect: Unconnect and reconnect before and after the AP setting
        """
        # Disconnect the DUT
        if do_connect:
            self._networking_api.wifi_disconnect(self._ssid)

        self._logger.info("Set bandwidth to AP: %s" % bandwidth)

        # Reconfigure the AP
        self._ns.init()
        self._ns.set_wifi_bandwidth(bandwidth, self._standard)
        self._ns.enable_wireless()
        if self._auto_channel:
            # If using auto channel, retrieve the selected channel from the AP as it can changes with config update
            self._channel = self._ns.get_selected_channel()
        self._ns.release()

        # Connect back the DUT
        if do_connect:

            # Initialize the connection analysis process
            self._init_connection_analysis()

            # Connect the DUT
            self._networking_api.wifi_connect(self._ssid)

            # Check that the connection is well in the desired bandwidth
            self._run_connection_analysis_process(bandwidth=bandwidth)

#------------------------------------------------------------------------------

    def _set_ap_mimo(self, mimo_enable, do_connect):
        """
        Enable or disable MIMO feature on the AP

        :type mimo_enable: str
        :param mimo_enable: bandwidth to be set
        :type do_connect: boolean
        :param do_connect: Unconnect and reconnect before and after the AP setting
        """
        # Disconnect the DUT
        if do_connect:
            self._networking_api.wifi_disconnect(self._ssid)

        self._logger.info("%s WIFI MIMO on AP" % ("ENABLE" if mimo_enable else "DISABLE"))

        # Reconfigure the AP
        self._ns.init()
        self._ns.set_wifi_standard(self._standard, mimo_enable)
        self._ns.enable_wireless()
        if self._auto_channel:
            # If using auto channel, retrieve the selected channel from the AP as it can changes with config update
            self._channel = self._ns.get_selected_channel()
        self._ns.release()

        # Connect back the DUT
        if do_connect:

            # Initialize the connection analysis process
            self._init_connection_analysis()

            # Connect the DUT
            self._networking_api.wifi_connect(self._ssid)

            # Check that the connection uses the desired MIMO option
            self._run_connection_analysis_process(wifi_mimo=mimo_enable)
