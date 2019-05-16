"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: SII on behalf INTEL MCG PSI
:summary: Cards implementation for EMT350 equipment
:author: vgomberx
:since: 15/01/2014
"""


class POWER_CARD():
    ROLE = ["AC_CHARGER", "EMULATED_BATTERY"]
    # there is 2*2 outputs , 2 per role
    OUTPUT_DETAILS = {"OUTPUT_AC": 2, "OUTPUT_EMU_BATT": 2}
    OUTPUT_AC = "OUTPUT_AC"
    OUTPUT_EMU_BATT = "OUTPUT_EMU_BATT"
    OUTPUT = 4
    TYPE = "POWER"
    BATTID_MIN = 0
    BATTID_MAX = 500000
    BPTERM_MIN = 0
    BPTERM_MAX = 500000
    # xml name of output  OUTPUT_AC_x and OUTPUT_EMU_BATT_x

    def configuration_validation(self, xml_params, dut_name):
        """
        validate that the mother board configuration from bench config
        match with this card requirement.

        :type xml_params: object
        :param xml_params: xml parameter from benchconfig

        :type dut_name: str
        :param dut_name: dut name associated to this card

        :rtype: tuple
        :return: (boolean True if error happen , error message, dictionary containing the card configuration)
        """
        card_conf = {}
        card_conf["TYPE"] = self.TYPE
        error_happen = False
        error_msg = ""

        # search for its outputs
        outputs_list = []
        for attr_name in xml_params.get_parameters_name():

            for output_type in self.OUTPUT_DETAILS.keys():
                output_name = attr_name.upper().strip()
                if output_type not in output_name:
                    continue
                # check if output is well formed
                # TODO: use reg exp to check the format
                output_no = output_name.rsplit("_", 1)[-1].strip()
                if not output_no.isdigit():
                    error_msg += "output not well formated: must be like following %s_1, found %s.\n" % (output_type, attr_name)
                    error_happen = True
                    continue
                output_no = int(output_no)

                # check that the output limit is respected
                if output_no > self.OUTPUT_DETAILS[output_type]:
                    error_msg += "the card type %s have only % output(s) for %s type, found  %s.\n" % (self.TYPE,
                                                                                     self.self.OUTPUT_DETAILS[output_type], output_type, attr_name)

                # check that there is not twice the same output
                if output_name in outputs_list:
                    error_msg += "it cant have 2 declaration of the same outputs for one daughter card, detected more than 1 occurrence of %s.\n" % output_name
                    error_happen = True
                    continue

                if str(dut_name).upper() == str(xml_params.get_param_value(attr_name, "NOT_SET")).upper():
                    outputs_list.append(output_name)
                    card_conf[output_name] = output_no

        return error_happen, error_msg, card_conf

    def validate_multi_use(self, client_conf, server_conf):
        """
        validate that the configuration of a client match the server configuration
        for this type of card on a same card slot.

        for dual used, we can have the same slot used for a battery and an ac charger

        :type client_conf: dict
        :param client_conf: configuration from the client, it takes the form of a dictionary
                            with following key :  TYPE, OUTPUT_AC_x, OUTPUT_EMU_BATT_x

        :type server_conf: dict
        :param server_conf: configuration from the server, it takes the form of a dictionary

        :rtype: tuple
        :return: (boolean True if error happen , error message)
        """
        error = False
        msg = ""
        # if we are here it means that we are in presence of 2 same card declaration on the slot
        for output in client_conf.keys():
            # skip TYPE key
            if output == "TYPE":
                continue

            # case we have the same output on both
            if output in server_conf:
                msg += "output [%s] is already used by another client.\n" % output
                error = True
                continue

            # case where we have the same output number but 2 different outputs types
            # this case is the only one where multi execution can be done
            # search for the port number
            elif client_conf[output] not in server_conf.values():
                msg += "cant control port 1 and 2 at the same time, you need to move all on the same port.\n"
                error = True
                continue

        return error, msg


class USB_CARD():
    ROLE = ["USB_SWITCH"]
    # there is 2*2 outputs , 2 per role
    OUTPUT_USB = "OUTPUT"
    OUTPUT = 2
    TYPE = "USB"
    # xml name of output like OUTPUT_x

    def configuration_validation(self, xml_params, dut_name):
        """
        validate that the mother board configuration from bench config
        match with this card requirement

        :type xml_params: object
        :param xml_params: xml parameter from benchconfig

        :type dut_name: str
        :param dut_name: dut name associated to this card

        :rtype: tuple
        :return: (boolean True if error happen , error message, dictionary containing the card configuration)
        """
        card_conf = {}
        card_conf["TYPE"] = self.TYPE
        error_happen = False
        error_msg = ""

        # search for its outputs
        outputs_list = []
        for attr_name in xml_params.get_parameters_name():
            output_name = attr_name.upper().strip()
            if not "OUTPUT" in output_name:
                continue
            # check if output is well formed
            # TODO: use reg exp to check the format
            output_no = attr_name.split("_")[-1].strip()
            if not output_no.isdigit():
                error_msg += "output must be named with OUTPUT_ and a digit like OUTPUT_1, found %s.\n" % attr_name
                error_happen = True
                continue

            output_no = int(output_no)
            # check that the output limit is respected
            if output_no > self.OUTPUT:
                error_msg += "the card type %s can have only  2 AC and 2 emulated outputs (4 totally), found output %s declaration.\n" % (self.TYPE,
                                                                                  attr_name)
            # check that there is not twice the same output
            if output_name in outputs_list:
                error_msg += "it cant have 2 declaration of the same outputs for one daughter card, detected more than 1 occurrence of %s.\n" % output_name
                error_happen = True
                continue

            outputs_list.append(output_name)
            # check if the output role is known
            if str(dut_name).upper() == str(xml_params.get_param_value(attr_name, "NOT_SET")).upper():
                card_conf[output_name] = output_no

        return error_happen, error_msg, card_conf

    def validate_multi_use(self, client_conf, server_conf):
        """
        validate that the configuration of a client match the server configuration
        for this type of card

        for dual used
        we cant have the same slot for 2 outputs

        :type client_conf: dict
        :param client_conf: configuration from the client, it takes the form of a dictionary
                            with following key :  TYPE, OUTPUT_AC_x, OUTPUT_EMU_BATT_x

        :type server_conf: dict
        :param server_conf: configuration from the server, it takes the form of a dictionary

        :rtype: tuple
        :return: (boolean True if error happen , error message)
        """
        error = False
        msg = ""

        if None not in [client_conf, server_conf]:
            # This error message could be change to be more talkative
            msg = "cant do parallel used for USB card on the same slots.\n"
            error = True

        return error, msg

SUPPORTED_DAUGHTER_CARD = {"USB": USB_CARD,
                            "POWER": POWER_CARD}
