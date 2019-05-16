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
:summary: Utilities class for Measure Throughput
:since: 25/12/2014
:author: gcharlex
"""
import numbers


class Unit:
    """
    Structure that represent a unit of measurement
    """
    def __init__(self, rep, ratio):
        self.rep = rep
        self.ratio = ratio

    def __str__(self):
        """
        Human readable representation of the instance

        :rtype: str
        :return: Human readable representation of this instance
        """
        return self.rep[0]


class DuplexThroughputMeasure:
    """
    Structure that represent a duplex (UL/DL) throughput measure
    """
    def __init__(self):
        self.ul_throughput = ThroughputMeasure()
        self.dl_throughput = ThroughputMeasure()

class ThroughputMeasure:
    """
    Structure that represent a throughput measure
    """
    UNITS = (BPS_UNIT, KBPS_UNIT, MBPS_UNIT, GBPS_UNIT) = (Unit(['Bits/sec', 'bits/sec', 'Bps', 'bps'], 1),
                                                Unit(['Kbits/sec', 'kbits/sec', 'Kbps', 'kbps'], 1e3),
                                                Unit(['Mbits/sec', 'mbits/sec', 'Mbps', 'mbps'], 1e6),
                                                Unit(['Gbits/sec', 'gbits/sec', 'Gbps', 'gbps'], 1e9))
    @staticmethod
    def parse_unit(string):
        """
        Parse a throughput unit

        :type string: str
        :param string: String to parse

        :rtype: Unit
        :return: Throughput unit parsed
        """
        for unit in ThroughputMeasure.UNITS:
            if string in unit.rep:
                return unit
        raise ValueError("Unit not found : %s " % string)

    @staticmethod
    def from_string(string):
        """
        Parse throughput measure from string

        :type string: str
        :param string: String to parse

        :rtype: ThroughputMeasure
        :return: Throughput measure parsed
        """
        string.strip()
        string_split = string.split()
        value = float(string_split[0])
        unit = ThroughputMeasure.parse_unit(string_split[1])
        return ThroughputMeasure(value, unit)

    def __init__(self, value=None, unit=None):
        if value is None:
            value = 0
        if unit is None:
            unit = ThroughputMeasure.KBPS_UNIT

        self.value = value
        self.unit = unit

    def __str__(self):
        """
        Human readable representation of the instance

        :rtype: str
        :return: Human readable representation of this instance
        """
        return "%.3f %s" % (self.value, str(self.unit))

    """Mathematical operator"""
    def __abs__(self):
        return ThroughputMeasure(abs(self.value), self.unit)

    def __add__(self, other):
        if isinstance(other, ThroughputMeasure):
            return ThroughputMeasure(self.value + other.to(self.unit).value, self.unit)
        else:
            return NotImplemented

    def __sub__(self, other):
        if isinstance(other, ThroughputMeasure):
            return ThroughputMeasure(self.value - other.to(self.unit).value, self.unit)
        else:
            return NotImplemented

    def __mul__(self, other):
        if isinstance(other, numbers.Number):
            return ThroughputMeasure(self.value * other, self.unit)
        else:
            return NotImplemented

    def __div__(self, other):
        if isinstance(other, numbers.Number):
            return ThroughputMeasure(self.value / other, self.unit)
        else:
            return NotImplemented

    """Comparison operators"""
    def __eq__(self, other):
        return self.value == other.to(self.unit).value

    def __ne__(self, other):
        return self.value != other.to(self.unit).value

    def __gt__(self, other):
        return other < self

    def __ge__(self, other):
        return not self < other

    def __le__(self, other):
        return not other < self

    def __lt__(self, other):
        return self.value < other.to(self.unit).value

    def set(self, value, unit):
        """
        Set value and unit

        :type value: float
        :param value: Value to set
        :type unit: Unit
        :param unit: Unit of the value
        """
        self.value = value
        self.unit = unit

    def to(self, unit):
        """
        Create a egual throughput measure converted to a specific unit
        :type unit: Unit
        :param unit: target unit
        :rtype: ThroughputMeasure
        :return: New throughput measure converted
        """
        result = ThroughputMeasure(self.value, self.unit)
        result.convert_to(unit)
        return result

    def convert_to(self, unit):
        """
        Converts this throughput measure to a specific unit
        :type unit: Unit
        :param unit: target unit
        """

        self.value = self.value / (unit.ratio / self.unit.ratio)
        self.unit = unit

    def best_unit(self):
        """
        Finds the best unit to represent this throughput measure
        """
        if abs(self).to(ThroughputMeasure.MBPS_UNIT).value > 1.0:
            return ThroughputMeasure.MBPS_UNIT
        elif abs(self).to(ThroughputMeasure.KBPS_UNIT).value > 1.0:
            return ThroughputMeasure.KBPS_UNIT
        else:
            return ThroughputMeasure.BPS_UNIT

    def to_best_unit(self):
        """
        Create a egual throughput measure converted the best unit to represent it
        """
        return self.to(self.best_unit())

    def convert_to_best_unit(self):
        """
        Converts to the best unit to represent this throughput measure
        """
        self.convert_to(self.best_unit())