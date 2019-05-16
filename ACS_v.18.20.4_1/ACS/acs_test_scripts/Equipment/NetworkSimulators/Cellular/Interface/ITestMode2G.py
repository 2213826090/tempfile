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
:summary: virtual interface of 2G test mode functionalities for cellular network
simulators
:since: 05/04/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class ITestMode2G(object):

    """
    ITestMode2G class: virtual interface of 2G test mode functionalities for cellular
    network simulators.
    """

    def configure_ber_measurement(self, nb_frames, ref_level):
        """
        Configures bit error rate measurement (BER)
        :type nb_frames: integer
        :param nb_frames: number of frames to use for the BER measurement.
        :type ref_level: double
        :param ref_level: the cell power reference to use for the measurement
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_ber(self):
        """
        Gets actual bit error rate (BER)
        :rtype: double
        :return: the measured BER (Bit Error Rate)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_rssi(self):
        """
        Gets actual received signal strength indicator (RSSI)
        :rtype: long
        :return: the measured RSSI
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_rssi_connectivity(self, target, margin):
        """
        Checks DUT to network simulator connectivity through RSSI measurement.
        The RSSI value retrieved is compared to target +/- margin.
        :type target: integer
        :param target: value of the expected RSSI (in dBm).
        :type margin: integer
        :param margin: acceptable margin.
        :raise TestEquipmentException: an internal problem occurred.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_modulation_spectrum_limits(self, modulation, limits):
        """
        Sets limits for modulation spectrum measurements.
        :type modulation: str
        :param modulation: the modulation type to use
            - "GSM"
            - "EDGE"
            - "ALL": sets limits for all available modulation
        :type limits: dict
        :param limits: a dictionary containing the limits for each following keys:
            - "0,10MHZ", "0,20MHZ", "0,25MHZ", "0,60MHZ", "0,80MHZ", "1,00MHZ",
              "1,20MHZ", "1,40MHZ", "1,60MHZ","1,80MHZ"
            - values: tuple containing 3 float in that order:
                - reference power in dBm (--> value parameter)
                - low limit in dB (--> minPower parameter)
                - high limit in dB (--> maxPower parameter)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_switching_spectrum_limits(self, modulation, limits):
        """
        Sets the limits for spectrum switching measurements
        :type modulation: str
        :param modulation: the modulation type to use
            - "GSM"
            - "EDGE"
            - "ALL" : sets the limits for all available modulations
        :type limits: dict
        :param limits: a dictionary containing the limits to set for each power level number
            - keys are tuple containing 2 integers: (power level number, power level).
            - values: list of 5 values:
                - the first is a str corresponding to 'enable' parameter
                    - "OFF"
                    - "ON"
            - the four other values are integers representing limit for 0.40MHz, 0.60MHz,
              1.20MHz and 1.80MHz.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_gsm_power(self):
        """
        This function starts a single shot measurement and returns the results
        :rtype: dict
        :return: a dictionary containing the following measurements:
            - key : "AVG_BURST_POWER", value type: float
            - key : "PEAK_BURST_POWER", value type: float
            - key : "PCL", value type: integer
            - key : "TIMING_ADV_ERROR", value type: float
            - key : "BURST_OUT_OF_TOLERANCE", value type: float
            - key : "BURST_MATCHING", value type: Global.SUCCESS or Global.FAiLURE
            - key : "AVG_BURST_POWER", value type: float
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_edge_power(self):
        """
        This function starts a single shot measurement and returns the results
        :rtype: dict
        :return: a dictionary containing the following measurements:
            - key : "AVG_BURST_POWER", value type: float
            - key : "PEAK_BURST_POWER", value type: float
            - key : "PCL", value type: integer
            - key : "TIMING_ADV_ERROR", value type: float
            - key : "BURST_OUT_OF_TOLERANCE", value type: float
            - key : "BURST_MATCHING", value type: Global.SUCCESS or Global.FAiLURE
            - key : "AVG_BURST_POWER", value type: float
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_gsm_avg_modulation_meas(self):
        """
        This function starts a single shot measurement and returns the results.
        :rtype: dict
        :return: a dictionary containing the following keys
            - "PHASE_ERR_PEAK": integer
            - "PHASE_ERR_RMS": integer
            - "OFFSET": float
            - "IQ_IMBALANCE": float
            - "FREQUENCY_ERR": float
            - "AVG_BURST_POWER": float
            - "BURST_OUT_OF_TOL": float
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_gsm_modulation_spectrum_verdict(self):
        """
        Reads spectrum modulation measurement results and compares them with tolerance values
        :rtype: tuple
        :return:
            - global spectrum modulation verdict:
                - Global.SUCCESS
                - Global.FAILURE
            - reference power as a float
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_gsm_modulation_spectrum(self):
        """
        Starts single shot measurement and return results
        :return:
            a dictionary containing the power level strings for each following key:
                - "0MHZ"     | "-0,10MHZ" | "-0,20MHZ" | "-0,25MHZ" | "-0,40MHZ" |
                  "-0,60MHZ" | "-0,80MHZ" | "-1,00MHZ" | "-1,20MHZ" | "-1,40MHZ" |
                  "-1,60MHZ" | "-1,80MHZ" | "0,10MHZ"  | "0,20MHZ"  | "0,25MHZ"  |
                  "0,40MHZ"  | "0,60MHZ"  | "0,80MHZ"  | "1,00MHZ"  | "1,20MHZ"  |
                  "1,40MHZ"  | "1,60MHZ"  | "1,80MHZ"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_gsm_switching_spectrum_verdict(self):
        """
        Reads spectrum switching measurement results and compares them with tolerance values
        :rtype: tuple
        :return:
            - global spectrum modulation verdict:
                - Global.SUCCESS
                - Global.FAILURE
            - reference power as a float
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_gsm_switching_spectrum(self):
        """
        Starts single shot measurement and return results
        :return:
        - dict: a dictionary containing the power strings for each following key:
            - "-1800KHZ", "-1200KHZ", "-600KHZ", "-400KHZ", "0KHZ", "400KHZ",
              "600KHZ", "1200KHZ", "1800KHZ"
            - "NAN" is returned at the disabled points
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_edge_avg_modulation_meas(self):
        """
        Starts single shot measurements and return results
        :rtype: dict
        :return: a dictionary containing the following keys:
            - key: "95EVM"            => double: error vector magnitude
            - key: "EVM_PEAK"         => double: error vector magnitude peak
            - key: "EVM_RMS"          => double: error vector magnitude RMS
            - key: "OFFSET"           => double: offset
            - key: "IQ_IMBALANCE"     => double: IQ imbalance
            - key: "FREQUENCY_ERR"    => double: frequency error
            - key: "AVG_BURST_POWER"  => double: average burst power current
            - key: "BURST_OUT_OF_TOL" => double: burst out of tolerance
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_edge_modulation_spectrum_verdict(self):
        """
        Reads spectrum modulation measurement results and compares them with tolerance values
        :rtype: tuple
        :return:
            - global spectrum modulation verdict:
                - Global.SUCCESS
                - Global.FAILURE
            - reference power as a float
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_edge_modulation_spectrum(self):
        """
        Starts single shot measurement and return results
        :return:
            a dictionary containing the power level strings for each following key:
                - "0MHZ"     | "-0,10MHZ" | "-0,20MHZ" | "-0,25MHZ" | "-0,40MHZ" |
                  "-0,60MHZ" | "-0,80MHZ" | "-1,00MHZ" | "-1,20MHZ" | "-1,40MHZ" |
                  "-1,60MHZ" | "-1,80MHZ" | "0,10MHZ"  | "0,20MHZ"  | "0,25MHZ"  |
                  "0,40MHZ"  | "0,60MHZ"  | "0,80MHZ"  | "1,00MHZ"  | "1,20MHZ"  |
                  "1,40MHZ"  | "1,60MHZ"  | "1,80MHZ"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_edge_switching_spectrum_verdict(self):
        """
        Reads spectrum switching measurement results and compares them with tolerance values
        :rtype: tuple
        :return:
            - global spectrum modulation verdict:
                - Global.SUCCESS
                - Global.FAILURE
            - reference power as a float
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_edge_switching_spectrum(self):
        """
        Starts single shot measurement and return results
        :return:
        - dict: a dictionary containing the power strings for each following key:
            - "-1800KHZ", "-1200KHZ", "-600KHZ", "-400KHZ", "0KHZ", "400KHZ",
              "600KHZ", "1200KHZ", "1800KHZ"
            - "NAN" is returned at the disabled points
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
