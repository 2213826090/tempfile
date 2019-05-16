"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL CCG
:summary: This file implements the class to parse data from Saleae and export them for MPTA and ACS verdict computation.

STEP 4

:since: 2015-01-27
:author: emarchan
"""

from lxml import etree
import time
import os
from Constants import DATA_DIRECTION_C2M, RT_FIELDS_TYPES, hex_to_bin_str
from LogicDataDebug import LogicDataDebug, log_tag


class LogicDataParser():  # called FormatLogicDataForVerdict.py@23
    """
    Gets the formatted logic analyzer data as input and produces an MPTA and ACS output

    Reference is 'Core Spec Addendum 3 rev 2.pdf'
    WIRELESS COEXISTENCE INTERFACE 2 (WCI-2) TRANSPORT SPECIFICATION starts at page 111


    """
    def __init__(self):
        self._input_data = None
        self._logger = LogicDataDebug()

    def get_input_data_from_file(self, input_file):  # called FormatLogicDataForVerdict.py@155
        """
        Get the input data to parse from a file
        :param input_file: LogicDataExporter XML data
        :type input_file: string
        """
        if (os.path.isfile(input_file)):
            self._logger.info(log_tag() + "Getting input data from %s" % input_file)
            start_time = time.time()
            self._input_data = etree.parse(input_file)

            stop_time = time.time()

            self._logger.info(log_tag() + "Data loaded in %d s" % (stop_time - start_time))
        else:
            self._logger.error(log_tag() + "Can't open input file %s" % input_file)

    def get_input_data_from_variable(self, input_data):  # unreferenced
        """
        Get the input data to parse from a variable
        :param input_data: LogicDataExporter XML data
        :type input_data: string in LogicDataExporter XML format.
        """
        self._input_data = input_data

    def print_input_data(self):  # commented FormatLogicDataForVerdict.py@157
        """
        Prints the data loaded.
        """
        self._logger.info(etree.tostring(self._input_data, pretty_print=True))
        print type(self._input_data)

    def set_framing_error(self, root, element):
        assert(element.attrib['framing_error'] == "1")

        data_root = etree.Element("DATA", type="ERROR", \
            direction=element.attrib['direction'], \
            time_stamp=str(element.attrib['time_stamp']))
        etree.SubElement(data_root, "ERROR_REASON").text = "UART framing error"
        root.append(data_root)

        return data_root

    def get_data_matching_direction(self, direction):  # called FormatLogicDataForVerdict.py@161
        """
        Returns the line matching the given direction
        :param direction: the Direction to match (MC or CM)
        :type direction: string

        :return: a list containing the matching element
        :rtype: List
        """
        out_list = []
        # Parse the list
        for element in self._input_data.iter("DATA"):
            # Get matching elements
            if element.attrib['direction'] == direction:
                # Compute output
                new_data = [  # should add element.attrib['bit'],
                    element.attrib['time_stamp'],
                    element.attrib['direction'],
                    element.attrib['value']]
                out_list.append(new_data)

        return out_list

    def get_rt_type_and_data_from_input_value(self, element):  # called LogicDataParser.py@144
        """
        Bits [2:0] = Type
        Bits [7:3] = Data
        """
        value = element.attrib['value']
        val_input = hex_to_bin_str(value)
        # Note: Strings are parsed from left to right, this is why we've got the value 5 and not 3.
        rt_type = int(val_input[5:], 2)
        assert(rt_type <= 7)
        rt_type = RT_FIELDS_TYPES[rt_type]
        data = val_input[0:5]

        is_C2M = element.attrib['direction'] == "CM"
        assert(is_C2M or element.attrib['direction'] == "MC")

        # Fill the data element
        data_root = etree.Element("DATA", \
            type=rt_type, \
            direction=element.attrib['direction'], \
            time_stamp=str(element.attrib['time_stamp']))

        return (rt_type, data, is_C2M, data_root)

    def set_m2c_real_time(self, data_root, data):  # page 117
            etree.SubElement(data_root, "MWS_PATTERN").text = str(data[3:5])
            etree.SubElement(data_root, "MWS_TX").text = str(data[2])
            etree.SubElement(data_root, "MWS_RX").text = str(data[1])
            etree.SubElement(data_root, "FRAME_SYNC").text = str(data[0])

    def set_standard_m2c(self, rt_type, data_root, data):
        if rt_type == "REAL_TIME":  # page 117
            self.set_m2c_real_time(data_root, data)
        elif rt_type == "TRANSPORT_CONTROL":  # page 117
            etree.SubElement(data_root, "RESEND_REAL_TIME").text = str(data[0])
            etree.SubElement(data_root, "RFU").text = str(data[1:5])
        elif rt_type == "TRANSPARENT_DATA":  # page 118
            etree.SubElement(data_root, "NIBBLE_POSITION").text = str(data[0])
            etree.SubElement(data_root, "DATA").text = str(data[1:5])
        elif rt_type == "MWS_INACTIVITY_DURATION":  # page 119
            etree.SubElement(data_root, "DURATION").text = str(data)
        elif rt_type == "MWS_SCAN_FREQUENCY":  # page 119
            etree.SubElement(data_root, "FREQ").text = str(data)

    def set_standard_c2m(self, rt_type, data_root, data):
        if rt_type == "REAL_TIME":  # page 117
            etree.SubElement(data_root, "RFU").text = str(data[4])
            etree.SubElement(data_root, "IS_802_TX_ON").text = str(data[3])
            etree.SubElement(data_root, "IS_802_RX_PRI").text = str(data[2])
            etree.SubElement(data_root, "BT_TX_ON").text = str(data[1])
            etree.SubElement(data_root, "BT_RX_PRI").text = str(data[0])
        elif rt_type == "TRANSPORT_CONTROL":
            etree.SubElement(data_root, "RESEND_REAL_TIME").text = str(data[0])
        elif rt_type == "TRANSPARENT_DATA":
            etree.SubElement(data_root, "NIBBLE_POSITION").text = str(data[0])
            etree.SubElement(data_root, "DATA").text = str(data[1:5])

    def get_data_standard(self):  # called by FormatLogicDataForVerdict.py@199
        """
        Returns the data to be analyzed by ACS.
        """
        # Create top element
        root = etree.Element("DATAS", mode="STANDARD")

        # Parse input data
        for element in self._input_data.iter("DATA"):
            # Check if the message is valid.
            if element.attrib['framing_error'] == "1":
                data_root = self.set_framing_error(root, element)
            else:
                # Valid message, extract values
                (rt_type, data, is_C2M, data_root) = self.get_rt_type_and_data_from_input_value(element)

                if is_C2M:
                    self.set_standard_c2m(rt_type, data_root, data)
                else:
                    self.set_standard_m2c(rt_type, data_root, data)

                # Attach the newly created element to root
                root.append(data_root)

        return root

    def set_lnp_m2c(self, rt_type, data_root, data):
        if rt_type == "REAL_TIME":  # page 117
            self.set_m2c_real_time(data_root, data)
        elif rt_type == "TRANSPORT_CONTROL":  # page 117
            etree.SubElement(data_root, "RESEND_REAL_TIME").text = str(data[0])
        elif rt_type == "TRANSPARENT_DATA":
            if str(data[0]) == "0":  # CNV_SAFE_TX_POWER[0] | 00 | FROM_BT_802 | 0
                etree.SubElement(data_root, "FROM_BT_802").text = str(data[1])
                etree.SubElement(data_root, "CNV_SAFE_TX_POWER").text = str(data[4])
            else:  # CNV_SAFE_TX_POWER[4:1] | 1
                etree.SubElement(data_root, "CNV_SAFE_TX_POWER").text = str(data[1:5])
        elif rt_type == "MWS_INACTIVITY_DURATION":
            etree.SubElement(data_root, "DURATION").text = str(data[0:5])
        elif rt_type == "MWS_SCAN_FREQUENCY":  # EXP stands for !
            etree.SubElement(data_root, "EXP_MWS_SCAN_FREQUENCY_START_STOP").text = str(data[0])
            etree.SubElement(data_root, "EXP_MWS_SCAN_FREQUENCY_FORBID_RANGE_DIR").text = str(data[1])
            etree.SubElement(data_root, "EXP_MWS_SCAN_FREQUENCY_REF_POINT").text = str(data[0:3])
        elif rt_type == "VENDOR_SPECIFIC_6":
            etree.SubElement(data_root, "FRAME_SYNC_DL").text = str(data[0])
            etree.SubElement(data_root, "PUCCH_INDEX").text = str(data[1])
            etree.SubElement(data_root, "MWS_TX_LOW_PWR").text = str(data[2])
        elif rt_type == "VENDOR_SPECIFIC_7":
            etree.SubElement(data_root, "TX_ON_N_RX_PRI_RES").text = str(data[0])
            etree.SubElement(data_root, "RFU").text = str(data[1])
            etree.SubElement(data_root, "MWS_TX_OFF").text = str(data[2])
            etree.SubElement(data_root, "MWS_TX_BT_ALLOWED").text = str(data[3])
            etree.SubElement(data_root, "MWS_TX_WIFI_ALLOWED").text = str(data[4])

    def set_lnp_c2m(self, rt_type, data_root, data):
        if rt_type == "REAL_TIME":
            etree.SubElement(data_root, "CONNECTION_PRI").text = str(data[4])
            etree.SubElement(data_root, "IS_802_TX_ON").text = str(data[3])  # Volume 7, Part A, Section 2.1.7
            etree.SubElement(data_root, "IS_802_RX_PRI").text = str(data[2])
            etree.SubElement(data_root, "BT_TX_ON").text = str(data[1])
            etree.SubElement(data_root, "BT_RX_PRI").text = str(data[0])
        elif rt_type == "TRANSPORT_CONTROL":
            etree.SubElement(data_root, "RESEND_REAL_TIME").text = str(data[0])
        elif rt_type == "TRANSPARENT_DATA":
            if str(data[0]) == "0":  # CNV_MAX_INBAND_NOISE_POWER[0] | 00 | FROM_BT_802 | 0
                etree.SubElement(data_root, "FROM_BT_802").text = str(data[1])
                etree.SubElement(data_root, "CNV_MAX_INBAND_NOISE_POWER").text = str(data[4])
            else:  # CNV_MAX_INBAND_NOISE_POWERR[4:1] | 1
                etree.SubElement(data_root, "CNV_MAX_INBAND_NOISE_POWERR").text = str(data[1:5])
        elif rt_type == "VENDOR_SPECIFIC_6":
            etree.SubElement(data_root, "BT_TX_CH").text = str(data[0:5])
        elif rt_type == "VENDOR_SPECIFIC_7":
            if str(data[0]) == "0":  # BT_TX_POWER[3:0] | FROM_BT_802 = 0
                etree.SubElement(data_root, "BT_TX_POWER").text = str(data[1:4])
            else:  # RFU^2 | 802_TX_ON_IND | FROM_BT_802 = 0
                etree.SubElement(data_root, "IS_802_TX_ON_IND").text = str(data[1])

    def get_data_lnp(self):
        # Create top element
        root = etree.Element("DATAS", mode="LNP")

        # Parse input data
        for element in self._input_data.iter("DATA"):
            # Check if the message is valid.
            if element.attrib['framing_error'] == "1":
                data_root = self.set_framing_error(root, element)
            else:
                # Valid message, extract values
                (rt_type, data, is_C2M, data_root) = self.get_rt_type_and_data_from_input_value(element)

                if is_C2M:
                    self.set_lnp_c2m(rt_type, data_root, data)
                else:
                    self.set_lnp_m2c(rt_type, data_root, data)

                # Attach the newly created element to root
                root.append(data_root)


        # lookup etree.SubElement(data_root, "MWS_TX").text = str(data[2])

        return root

    def crfe_mws_scan_frequency(self, root):
        # rule: MWS_SCAN_FREQUENCY (0->x then x ->0)

        error_count = 0
        prev = None
        for current in root.iterfind(".//MWS_SCAN_FREQUENCY"):
            if prev == None:
                # first signal must be 0
                if int(current.text) != 0:
                    current.set("ERROR", "RISING_FALLING_EDGE_VIOLATION")
                    error_count += 1
                    continue
                prev = current
                continue

            # current must be different that previous
            if current.text == prev.text:
                current.set("ERROR", "RISING_FALLING_EDGE_VIOLATION")
                error_count += 1
                continue  # solution 1 below

            # After an error at 'step i', two solutions:
            # 1- 'step i+1' signal must be  compared with last valid previous
            # 2- 'step i+1' compared with 'step i+2'  !!!

            prev = current

        return error_count

    def crfe_mws_tx_rx_frame_sync(self, root):
        # MWS_TX rising edge must be followed by falling edge for:
        # MWS_TX, MWS_RX, FRAME_SYNC,
        #               _
        # FRAME_SYNC __| |________________
        #               __     __
        # MWS_TX     __|  |___|  |__________
        #                  ___    _____
        # MWS_RX     _____|   |__|     |____

        class ftr:
            fs = tx = rx = 0
            def set(self, element):
                self.fs = int(element.findtext("FRAME_SYNC"))
                self.tx = int(element.findtext("MWS_TX"))
                self.rx = int(element.findtext("MWS_RX"))
            def is_zero(self):
                return self.fs == 0 and self.tx == 0 and self.rx == 0

        prev = None
        current_ftr = ftr()
        for current in root.iterfind('.//DATA[@type="REAL_TIME"][@direction="MC"]'):
            current_ftr.set(current)
            if prev == None:
                if not current_ftr.is_zero():
                    current.set("ERROR", "RISING_FALLING_EDGE_VIOLATION")
                    continue
                prev = ftr()
                prev = current_ftr
                continue
            # TODO only first must be zero, all other cases have to be implemented ...


    def crfe_802_bt_tx_rx(self, root):
        # 802_TX_ON, 802_RX_PRI, BT_TX_ON, BT_RX_PRI,

        class lbtr:  # l stands for lte(802), b bluetooth, t tx, r rx
            lte_t = lte_r = bt_t = bt_r = 0
            def set(self, element):
                self.lte_t = int(element.findtext("IS_802_TX_ON"))
                self.lte_r = int(element.findtext("IS_802_RX_PRI"))
                self.bt_t = int(element.findtext("BT_TX_ON"))
                self.bt_r = int(element.findtext("BT_RX_PRI"))
            def is_zero(self):
                return self.lte_t == 0 and self.lte_r == 0 and self.bt_t == 0 \
                    and self.bt_r == 0

        prev = None
        current_lbtr = lbtr()
        for current in root.iterfind('.//DATA[@type="REAL_TIME"][@direction="CM"]'):
            current_lbtr.set(current)
            if prev == None:
                if not current_lbtr.is_zero():
                    current.set("ERROR", "RISING_FALLING_EDGE_VIOLATION")
                    continue
                prev = lbtr()
                prev = current_lbtr
                continue
            # TODO only first must be zero, all other cases have to be implemented ...


    def check_rising_falling_edge(self, root):
        # Check consistency over time for rising/falling edge, in order to insert error
        # MWS_TX rising edge must be followed by falling edge for:
        # MWS_TX, MWS_RX, FRAME_SYNC,
        # 802_TX_ON, 802_RX_PRI, BT_TX_ON, BT_RX_PRI,
        # MWS_SCAN_FREQUENCY (0->x then x ->0)

        # etree ref http://lxml.de/tutorial.html#elementpath

        # 'crfe' stands for 'check rising falling edge'
        # MWS_SCAN_FREQUENCY (0->x then x ->0)
        self.crfe_mws_scan_frequency(root)

        # MWS_TX, MWS_RX, FRAME_SYNC,
        self.crfe_mws_tx_rx_frame_sync(root)

        # 802_TX_ON, 802_RX_PRI, BT_TX_ON, BT_RX_PRI,
        # roughly similar with to crfe_mws_tx_rx_frame_sync above
        self.crfe_802_bt_tx_rx(root)

        return root

    def run(self):
        element = self._input_data.getroot()
        mode = element.attrib['mode']
        assert(mode == "STANDARD" or mode == "LNP")

        if mode == "STANDARD":
            return self.check_rising_falling_edge(self.get_data_standard())
        else:
            return self.check_rising_falling_edge(self.get_data_lnp())

