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
:summary: IOCard test helper (contains utility methods to test IO cards behavior)
:since: 20/02/2014
:author: fbongiax
"""
import time
import mock


class IOCardSutTestHelper(object):
    """
    IOCard test helper class
    """
    def __init__(self, sut):
        """
        Constructor
        """
        time.sleep = mock.MagicMock("sleep")
        self._sut = sut
        self._iocard = sut._iocard
        self._iocard.get_bench_params().get_param_value.return_value = "USB_RLY08"
        self._sut._bench_params.has_parameter = mock.MagicMock(side_effect=self._side_effect)
        self._sut._bench_params.get_param_value = mock.MagicMock(side_effect=self._side_effect)
        self._bench_params = {}

    def _side_effect(self, key):
        """
        Method called when _bench_params has_parameter() or get_param_value() are called
        """
        if key in self._bench_params:
            return self._bench_params[key]
        return None

    def set_bench_params(self, params):
        """
        Sets fake bench param values
        """
        self._bench_params = params

    def set_buttons_info(self, values):
        """
        Sets buttons info in the fake bench config
        """
        for key in values:
            self._bench_params[key] = values[key]

    def when_iocard_get_bench_params_return(self, return_value):
        """
        Set what to return when iocard.get_bench_params() will be called
        """
        self._iocard.get_bench_params().get_param_value.return_value = return_value

    def assert_iocard_line_activated(self, line, duration):
        """
        Asserts the rele was enabled / disabled for the given duration
        """
        self._iocard.press_relay_button.assert_called_with(duration, line)
