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
:summary: This script implements the interface of sysdebug uecmd.
:since: 27/02/2013
:author: pbluniex
"""
from ErrorHandling.DeviceException import DeviceException


class ISysDebug():

    """
    Abstract class that defines the interface to be implemented
    by device system operations handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def synchronize(self):
        """
        Synchronize SysDebug module on the device

        :type config: str
        :param config: The configuration of the synchronization step
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def reset(self, delay_s=0):
        """
        Reset module before unplug it
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def init(self, _config=None):
        """
        Initialization of the module.
        At this step, the device must be connected.

        :type config: str
        :param config: The configuration of the module
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start(self):
        """
        Start the measure.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop(self):
        """
        Stop the measure.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def fetch(self):
        """
        Fetch sysdebug informations.
        This is done just after replug the device.

        :type wait: Integer
        :param wait: Tempo to wait before fetching information on the device.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def report(self):
        """
        Return the result using lxml.etree.Element class

        :rtype: lxml.etree.Element
        :return: The report of sysdebug information during the measure.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def report_current_data(self):
        """
        Return statistic view for results of sysdebug modules at the current time (without having called stop)

        :rtype: etree.Element
        :return: The Xml tree of Sysdebug modules
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
