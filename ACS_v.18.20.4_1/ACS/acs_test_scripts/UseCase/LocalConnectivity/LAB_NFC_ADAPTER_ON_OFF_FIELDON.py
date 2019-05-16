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
:summary: Check user can enable or disable NFC interface while DUT is placed in
            RF field
:since: 19/11/2012
:author: smaurelx

.. versionchanged:: 10/12/2012 - smaurelx RTC34549 - add a parameter in order to control vertical robot movement

"""


from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.LocalConnectivity.LAB_NFC_BASE import LabNfcBase


class LabNfcAdapterOnOffFieldOn(LabNfcBase):

    """
    Lab NFC Adapter on/off field on test
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabNfcBase.__init__(self, tc_name, global_config)

        self._reader_x = "0"
        self._reader_y = "0"
        self._reader_up = "0"
        self._reader_down = "0"

        self._nfc_robot_param = global_config.benchConfig.get_parameters("NFC_ROBOT1")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabNfcBase.set_up(self)

        self._reader_x = self._nfc_robot_param.get_param_value("ExternalReaderX")
        self._reader_y = self._nfc_robot_param.get_param_value("ExternalReaderY")
        self._reader_up = self._nfc_robot_param.get_param_value("ExternalReaderUp")
        self._reader_down = self._nfc_robot_param.get_param_value("ExternalReaderDown")

        # place DUT in front of reader
        self._robot_positioning(self._reader_x, self._reader_y, "null", "null")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        LabNfcBase.run_test(self)

        # --- ENABLE TEST ---
        # Put DUT on reader
        self._robot_positioning("null", "null", self._reader_down, "null")

        # Enable NFC
        self._nfc_api.nfc_enable()
        self._logger.info("Remove DUT from the reader and check NFC is enabled : Success")

        # Remove DUT from reader
        self._robot_positioning("null", "null", self._reader_up, "null")

        # --- DISABLE TEST ---
        # Put DUT on reader
        self._robot_positioning("null", "null", self._reader_down, "null")

        # Disable NFC
        self._nfc_api.nfc_disable()
        self._logger.info("Remove DUT from the reader and check NFC is disabled : Success")

        # Remove DUT from reader
        self._robot_positioning("null", "null", self._reader_up, "null")

        return Global.SUCCESS, "No errors"
