"""
@summary: This test step will try to get phone number from phone's SIM card.
It will return either the phone number from SIM card or from bench config if
fetching from SIM card didn't happen or did not yield valid phone number information.

@since 1 July 2014
@author: Stephen Smith
@organization: INTEL PEG-SVE-DSV

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

from UtilitiesFWK.Utilities import Global
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException

class GetDevicePhoneNumber(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        DeviceTestStepBase.run(self, context)
        self._logger.debug("START OF GET_DEVICE_PHONE_NUMBER")
        # Min length of 10. Ex. 512-512-5125
        min_length_phone_number = 10
        device_phone_number = None
        # Do we check phone number on sim card?
        if(self._pars.check_sim_card):
            device_phone_number = self.get_phone_number(min_length_phone_number)
        # Don't have phone number yet, let's try to use bench config PhoneNumber for the device.
        if(device_phone_number == None):
            try:
                device_phone_number = self._config.get("PhoneNumber")
            except:
                error_msg = "PhoneNumber for {0} was not found in bench config".format(self._config.device_name)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        self._logger.debug("END of GET_DEVICE_PHONE_NUMBER")
        # We should now have at least 3 digit area code + 7 digit phone number to try.
        if (len(str(device_phone_number)) < min_length_phone_number):
            error_msg = ("PhoneNumber: {0} from GET_DEVICE_PHONE_NUMBER TestStep does not appear valid.  Failing the TestStep.".format(device_phone_number))
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)
        else:
            context.set_info(self._pars.retrieved_phone_number, device_phone_number)
            self.ts_verdict_msg = "GET_DEVICE_PHONE_NUMBER passed and found {0} as {1}'s phone number".format(device_phone_number , self._config.device_name)

    def get_phone_number(self, min_length_phone_number = 10):
        '''
            The output of service call iphonesubinfo is like this:

            Result: Parcel(
            0x00000000: 00000000 0000000b 00350031 00320031 '........1.5.1.2.'
            0x00000010: 00300034 00350030 00370039 00000034 '4.0.0.5.9.7.4...')

            Objective: What we need to get is the phone number: 1-512-400-5974

            STEPS:
            0. It get the self.phone_number_string from the adb command.
            1. It splits the phone_number str
            2. It looks for this character: '
            3. It appends these two items:
               '........1.5.1.2.' and '4.0.0.5.9.7.4...'
               to phone_list.
            4. It erases phone_number variable to re-use it.
            5. Then it joins both items into phone_number
            6. It uses list fucntion to create a list_of_character
            7. Then it tries to convert each character to int
            8. If the character is not a number it will be ignored with PASS
               And it will not be part of the list_of_numbers
            9. list_of_numbers will contain the phone number
               splitted into a list.
            10. Finally it will join those items in phone_number
                variable
            11. If something fails it will return None.

            PARAMS:
            self - Contains device object.
            min_length_phone_number - used to reject number from SIM if not enough digits.
        '''
        phone_number = None
        # STEP 0
        adbCmd = "adb shell service call iphonesubinfo 6"
        (return_code, phone_number) = self._device.run_cmd(cmd=adbCmd, timeout=20)
        phone_list = []
        # If service call iphonesubinfo gave us some information, then continue
        # else just return a None phone number.
        if phone_number != None:
            # STEP 1
            for item in phone_number.split():
                # STEP 2
                if '\'' in item:
                    # STEP 3
                    phone_list.append(item)
            # STEP 4
            phone_number = ""
            try:
                # STEP 5
                phone_number = "".join([phone_list[0],phone_list[1]])
                # STEP 6
                list_of_characters = list(phone_number)
                list_of_numbers = []
                for item in list_of_characters:
                    # STEP 7
                    try:
                        # STEP 9
                        list_of_numbers.append(str(int(item)))
                    except:
                        # STEP 8
                        pass
                phone_number = ""
                # STEP 10
                phone_number = "".join(list_of_numbers)
            except:
                # STEP 11
                phone_number = None
        # Does the number have enough digits?
        if (len(str(phone_number)) < min_length_phone_number):
            phone_number = None
        return phone_number
