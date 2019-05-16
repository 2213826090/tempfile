"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL QCTV
:summary: RF switch equipment.
:since: 19/01/2015
:author: mbrisbax

"""
from win32com import client
import platform
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from ErrorHandling.TestEquipmentException import TestEquipmentException


class RFMatrix(EquipmentBase):

    """
    This class configure and control Mini-Circuits USB-4SPDT-A18 RF matrix
    The RF matrix USB-4SPDT-A18 is composed of four RF switches A, B, C and D
    Each RF switch has a COM port that can be connected either to port 1 or port 2
    The RF matrix can be configured for:
     - SIMO (Single antenna or SIMO system (3G or 2G)):
        one input cable connected to callbox (for RX/TX)
        and up to 5 DUT connected to the RF matrix (SP5T configuration in RF matrix GUI/Documentation)
     - MIMO (Multiple input multiple output system (LTE)):
        two input cables connected to callbox (one for RX1/TX1 and the other for RX2/TX2)
        and up to 3 DUT connected to the RF matrix (2x SP3T configuration in RF matrix GUI/Documentation)

    For each DUT connected to RF matrix, a port is associated. It depends of the configuration from 1 to 5 for SIMO and from 1 to 3 for MIMO
    (see RF matrix GUI/Documentation for more information: port = out number)
    For more informations, see:
    http://www.minicircuits.com/softwaredownload/AN-49-002.pdf
    http://www.minicircuits.com/softwaredownload/Prog_Manual-2-Switch.pdf
    http://www.minicircuits.com/softwaredownload/RFSwitchController_Setup.zip
    http://www.minicircuits.com/softwaredownload/MCL_RF_Switch_Controller_dll.zip

    """
    WINDOWS = "Windows"
    """
    matrix state dictionary:
        For a given matrix config (MIMO or SIMO) and a port number (from 1 to 5 for SIMO and from 1 to 3 for MIMO)
        a bitfield value is associated. This bitfield status correspond to the RF switches physical states:
            - Each bit corresponds to a single switch, with the LSB to switch "A" and the MSB to switch "D".
            - Each bit can be 0 (COM to port 1) or 1 (COM to port 2).
        For example:
            Val=6 (binary 00000110) sets switches "B" and "C" for COM to port 2 and all other switches for COM to port 1.
    """
    MATRIX_STATE = {"SIMO1": 0,
                    "SIMO2": 8,
                    "SIMO3": 1,
                    "SIMO4": 4,
                    "SIMO5": 6,
                    "MIMO1": 0,
                    "MIMO2": 3,
                    "MIMO3": 15}

    def __init__(self, name, model, eqt_params):
        """
        Constructor
        """
        # Initialize class parent
        EquipmentBase.__init__(self, name, model, eqt_params)
        # Check OS requirement
        if platform.system() != self.WINDOWS:
            msg = "Operating System not recognized [%s] " % self._os
            self._logger.error(msg)
            self._logger.error("RF matrix uses a windows COM DLL so use a windows computer to control it")
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, msg)

        self._serial_number = eqt_params.get_param_value("SN", "")
        self._config = eqt_params.get_param_value("Config", "SIMO")
        if self._config not in ("MIMO", "SIMO"):
            msg = "RF matrix support only MIMO or SIMO configuration" % self._config
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

    def init(self):
        """
        Initialise RF Matrix
        """
        # Load RF switch control DLL
        try:
            self._rf_matrix = client.Dispatch("MCL_RF_Switch_Controller.USB_RF_Switch")
        except:
            self._logger.error("Cannot load RF matrix  DLL")

        self.connect()

    def connect(self):
        """
        Connect to RF Matrix
        """
        # Connect to matrix
        if not self._rf_matrix.Connect(self._serial_number):
            msg = "Connection to RF Matrix failed check RF Matrix USB connection and power supply"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)
        # Check connection status
        if not self._rf_matrix.GetConnectionStatus():
            msg = "Connection to RF Matrix lost check RF Matrix USB connection and power supply"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)
        # Check USB connection to matrix
        if not self._rf_matrix.GetUSBConnectionStatus():
            msg = "USB Connection to RF Matrix lost"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

    def switch_to_rf_port(self, port):
        """
        Switch to DUT connected on specified RF switch port

        :type port: int
        :param port: port on RF switch (from 1 to 5 for SIMO and from 1 to 3 for MIMO)

        :rtype: str
        :return: tethering interface name
        """
        self._logger.info("Enable RF Matrix port %s for %s configuration" % (port, self._config))
        if port not in range(1, 4) and self._config == "MIMO":
            msg = "port %s must be in [1,3] interval for MIMO configuration" % port
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        elif port not in range(1, 6) and self._config == "SIMO":
            msg = "port %s must be in [1,5] interval for SIMO configuration" % port
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        # if not connected to matrix, Connect to matrix
        if not self._rf_matrix.GetUSBConnectionStatus() or not self._rf_matrix.GetConnectionStatus():
            self.connect()
        # Get switches state bitfield corresponding to port and wanted matrix config
        switch_config = self._config + str(port)
        switch_config_val = self.MATRIX_STATE[switch_config]
        self._logger.debug("Apply RF matrix %s configuration, bitfield: %s" % (switch_config, bin(switch_config_val)))
        # Set RF matrix to desired state
        out = self._rf_matrix.Set_SwitchesPort(switch_config_val)
        self._logger.debug("Matrix Status: %s" % str(out))
        # Get matrix status and check its integrity
        out = self._rf_matrix.GetSwitchesStatus()
        # GetSwitchesStatus returns a tuple (connection status, switches state bitfield)
        # Check if connection is still active and switch state bitfield is the desired one
        if not out[0] or not out[1] == switch_config_val:
            self._logger.error("RF matrix new configuration has failed, connection status: %s, switches status: %s" % (str(out[0]), bin(str(out[1]))))

    def release(self):
        """
        Disconnect from RF Matrix
        """
        self._rf_matrix.Disconnect()
