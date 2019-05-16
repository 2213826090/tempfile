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
:summary: This file implements the KK-specifics Localconnectivity UEcmd for Android phone
:since: 07/04/2014
:author: fdonx
"""

from acs_test_scripts.Device.UECmd.Imp.Android.JB_MR1.LocalConnectivity.LocalConnectivity import \
    LocalConnectivity as LocalConnectivityJB_MR1
from acs_test_scripts.Device.UECmd.UECmdDecorator import need

class LocalConnectivity(LocalConnectivityJB_MR1):
    _NFC_READERMODE_ACTIVITY = "acscmd.connectivity.nfc.NfcReaderModeActivity"

    def __init__(self, phone):
        """
        Constructor
        """
        LocalConnectivityJB_MR1.__init__(self, phone)

    @need('nfc')
    def read_nfc_tag_reader_mode(self):
        """
        Read data from NFC tag using reader mode

        :rtype: String
        :return: read_data
        """
        self._logger.info("Read NFC tag using reader mode")
        method = "readTagReaderMode"
        read_data = self._internal_exec_v2(self._NFC_READERMODE_ACTIVITY, method, is_system=True)
        return read_data["result"]

    @need('nfc')
    def write_nfc_tag_reader_mode(self, rtd_type, data):
        """
        Write data in tag using reader mode

        :type rtd_type: str
        :param rtd_type: "RTD_TEXT", "RTD_SMARTPOSTER" or "RTD_URI"

        :type data: String
        :param data

        :return: None
        """
        self._logger.info("Write " + data + " in tag using reader mode")
        method = "writeTagReaderMode"
        args = "--es rtd_type '%s' --es data '%s'" % (rtd_type, data)
        self._internal_exec_v2(self._NFC_READERMODE_ACTIVITY, method, args, is_system=True)

