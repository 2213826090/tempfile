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

:organization: INTEL MCG PSI
:summary: This file interact with application log generated
:since: 15/09/2014
:author: vgomberx
"""
from ErrorHandling.DeviceException import DeviceException


class IAplog(object):

    """
    :summary: Aplog related UEcommands for Android platforms
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def parse_shutdown_reason(self, txt):
        """
        parse the shutdown reason

        :warning: due to a problem with several log that look the same,
                    we need to parse the whole list instead of one element

        :type txt: str
        :param txt: text to parse

        :rtype: str
        :return: the shutdown reason found
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def parse_battery_info(self, txt_list):
        """
        parse the battery info available in aplog and return
        a list of extracted battery info in the same order of the input

        :type txt_list: str or list
        :param txt_list: text or list to parse

        :rtype: tuple
        :return: list of tuple (capacity, voltage)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def inject_tag(self, tag):
        """
        inject a tag that may be used to delimit a zone where
        you want to search for a value

        :type tag: str
        :param tag: tag to inject on aplog
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def find_txt_between_tag(self, start_tag, stop_tag, txt, nb_matching_result=0, raise_error=True):
        """
        search for a text that is between a start and stop tag
        and return the a list of element found between them that contains txt.
        the result list element 0 is the closest to start tag.

        :type start_tag: str or list
        :param start_tag: first element where we start to search from

        :type stop_tag: str or list
        :param stop_tag: last element after which we want to stop searching
                         stop tag will be after start_tag

        :type txt: str or list
        :param txt: text that we are looking for

        :type nb_matching_result: int
        :param nb_matching_result: number of matching result you want to iter on
                                by default will return all result between the tag

        :rtype: list
        :return: list of str that contains all line matching the result
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def parse_thermal_cooling_event(self, intent):
        """
        Parse the thermal cooling event and return several information about it

        :type intent: str
        :param intent: thermal cooling intent

        :rtype: tuple
        :return: a tuple (zone, level, temperature) or None if nothing is found
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
