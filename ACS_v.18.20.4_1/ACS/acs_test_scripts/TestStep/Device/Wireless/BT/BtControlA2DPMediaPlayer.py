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
:summary: This file implements a Test Step to wait for a sequence of AVRCP button events
:since 27/05/2014
:author: jfranchx
"""

import posixpath
from TestStep.Device.Wireless.BT.Base import BtBase
from ErrorHandling.AcsConfigException import AcsConfigException


class BtControlA2DPMediaPlayer(BtBase):
    """
    Implements a test step to start/stop A2DP media player
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """

        BtBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        if self._pars.timeout is None:
            self._pars.timeout = "3600"

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        BtBase.run(self, context)

        if not isinstance(self._pars.timeout, int):
            msg = "Error parameter TIMEOUT is not integer : %s" % str(type(self._pars.timeout))
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        full_path = posixpath.join(self._device.multimedia_path, self._pars.filename)

        if self._pars.action_control == "START_PLAYER":
            self._api.start_a2dp_media_player(full_path, self._pars.timeout)
        elif self._pars.action_control == "STOP_PLAYER":
            self._api.stop_a2dp_media_player()
        elif self._pars.action_control in ["PLAY","PAUSE","STOP","NEXT_TRACK","PREVIOUS_TRACK","VOLUMEDOWN","VOLUMEUP"]:
            self._api.control_a2dp_media_player(self._pars.action_control)
        else:
            msg = "Error parameter ACTION_CONTROL is unknown : %s" % str(self._pars.action_control)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
