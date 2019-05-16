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
:summary:
:since: 20/10/2010
:author: dgonzalez

"""


class FrequencyMgmt(object):

    """
    Frequency Management class.
    Can't be directly used. Use a derived class.
    """

    def __init__(self, ul_formula, dl_formula):
        """
        FrequencyMgmt class constructor.
        """
        self.__ul_formula = ul_formula
        self.__dl_formula = dl_formula

    def contains(self, channel):
        """
        Verify if channel is allowed in this instance.
        """
        pass

    def apply_formula(self, channel, direction):
        """
        apply uplink or downlink formula.
        """

        # 'n' variable will be used by 'eval' function
        n = channel  # @UnusedVariable #pylint:disable=W0612
        direction = str(direction).upper()
        if direction == 'UL':
            return eval(self.__ul_formula)
        elif direction == 'DL':
            return eval(self.__dl_formula)
        raise Exception('Unknown direction')

    def get_dl_formula(self):
        """
        Getter giving downlink formula
        """
        return self.__dl_formula

    def get_ul_formula(self):
        """
        Getter giving uplink formula
        """
        return self.__ul_formula

    def get_all_channels(self):
        """
        Retrieves all possible channel as a list.

        :rtype: list
        :return: list of integer defining all possible channels.
        """
        pass


class FrequencyRangeMgmt(FrequencyMgmt):

    """
    Frequency Range Management class.
    """

    def __init__(self, values_range, ul_formula, dl_formula):
        """
        FrequencyRangeMgmt class constructor.
        """
        FrequencyMgmt.__init__(self, ul_formula, dl_formula)
        end_1_num = values_range.find('-')
        self.__min = int(values_range[:end_1_num])
        self.__max = int(values_range[end_1_num + 1:])

    def contains(self, channel):
        return self.__min <= channel <= self.__max

    def __str__(self):
        """
        Function used when printing an instance of FrequencyRangeMgmt.
        """
        return '[' + str(self.__min) + ';' + str(self.__max) + '] UL: '\
            + self.get_ul_formula() + ', DL: ' + self.get_dl_formula()

    def get_all_channels(self):
        """
        Retrieves all possible channel as a list.

        :rtype: list
        :return: list of integer defining all possible channels.
        """
        possible_values = range(self.__min, self.__max, 1)
        return possible_values


class FrequencySetMgmt(FrequencyMgmt):

    """
    Frequency Set Management class.
    """

    def __init__(self, values_set, ul_formula, dl_formula):
        """
        FrequencySetMgmt class constructor.
        """
        FrequencyMgmt.__init__(self, ul_formula, dl_formula)
        self.__values_set = eval(values_set)

    def contains(self, channel):
        return channel in self.__values_set

    def __str__(self):
        """
        Function used when printing an instance of FrequencySetMgmt.
        """
        return str(self.__values_set) + ' UL: ' + self.get_ul_formula()\
            + ', DL: ' + self.get_dl_formula()

    def get_all_channels(self):
        """
        Retrieves all possible channel as a list.

        :rtype: list
        :return: list of integer defining all possible channels.
        """
        return self.__values_set
