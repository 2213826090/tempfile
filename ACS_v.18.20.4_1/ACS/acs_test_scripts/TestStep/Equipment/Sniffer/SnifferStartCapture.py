"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: This file implements a Test Step for start sniffer capture
:since 30/07/2014
:author: jfranchx
"""
from datetime import datetime
import os
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase


class SnifferStartCapture(EquipmentTestStepBase):
    """
    Implements Sniffer Start Capture class
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        EquipmentTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._pathname = global_conf.campaignConfig.get("campaignReportTree").get_report_path()
        self._sniffer = None

    def run(self, context):
        """
        Run test step
        """
        EquipmentTestStepBase.run(self, context)
        self._sniffer = self._equipment_manager.get_sniffer(self._pars.eqt)

        # Initialize connection with sniffer
        self._sniffer.init()

        # Create file and path name for file
        filename = "capture-%s-channel%s.cap" % (datetime.now().strftime("%Hh%M.%S"), (str(self._pars.channel)))
        capture = os.path.join(self._pathname, filename)

        # We assume that on Windows the current drive is the ACS one
        if capture[1] == ":":
            # Remove the drive letter
            capture = capture[2:]

        if self._pars.device_mac_addr == "NOT_USED":
            device_mac_addr = ""
        else:
            device_mac_addr = self._pars.device_mac_addr
        if self._pars.ssid == "NOT_USED":
            ssid = ""
        else:
            ssid = self._pars.ssid

        # Start capturing
        self._sniffer.start_capture(self._pars.channel, capture, self._pars.save_sniff_ascii, device_mac_addr, ssid)

        # Save output file name with OUTPUT_FILE_SAVE_AS
        context.set_info(self._pars.output_file_save_as, capture)