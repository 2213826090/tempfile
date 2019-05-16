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
:summary: Monitor S3 failures due to active wakeup source
:since: 21/01/2015
:author: sdubrayx
"""
from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug


class ActiveWakeupSource(SysDebug):

    """
    :summary: System UEcommands operations for Android platforms
              Fetching SysDebug informations on active wakeup source (S3 failure).
    """

    def __init__(self, device, _config=None):
        """
        Constructor.

        :type config: dictionary
        :param config: Configuration of the module. Unused for this module
        """
        SysDebug.__init__(self, device)

        self._retrieve_aplog_regexps.append("active wakeup source")
        # Trigger group :
        # Key is the name of the group
        # Value is a list of [group, prefix, pattern, suffix] where pattern
        # will be fetched in groups by re.search
        self._trigger_group = [["wakeupsource", ".*active wakeup source:\s*", ".*", "$"]]

    def __toxml(self, dicStats):
        """
        Transform dictionary to lxml.etree structure

        :type dicStats: dictionary
        :param dicStats: Statistic representation of informations.
        :rtype: etree.Element
        :return: The Xml tree vue of statistic results of the module
        """
        xmltree = etree.Element("ActiveWakeupSource")
        for failure in dicStats:
            current = etree.Element("failure")
            current.attrib["wakeupsource"] = failure["wakeupsource"]
            current.attrib["date"] = failure["date"]
            xmltree.append(current)

        return xmltree

    def stats(self, start, stop):
        """
        Return active wakeup source statistics which have been triggered between
        start_trigger and stop_trigger.

        :type start: Integer
        :param start: The timestamp of the beginning of the measure

        :type stop: Integer
        :param stop: The timestamp of the end of the measure

        :rtype: etree.Element
        :return: The Xml tree vue of statistic results of the module
        """
        SysDebug.stats(self, start, stop)

        self._logger.debug("ActiveWakeupSource stats : %s" % self._sys_debug_infos)
        return self.__toxml(self._sys_debug_infos)
