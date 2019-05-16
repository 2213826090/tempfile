"""
:copyright: (c)Copyright 2016, Intel Corporation All Rights Reserved.
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

:organization: INTEL CCG CRD OPM PCWTE
:summary: Utilities class for VoIP Server control
:since: 04/03/2016
:author: fbelvezx
"""
import time


class VoIPServerCallUtilities:
    """
    Class handling SIP call control
    """
    sip_profiles = {"PLAYBACK": "7001", "RECORD": "7002", "STANDBY": "7003"}

    def __init__(self, VoIPServerComputer, VoIPServerConfig, logger=None):
        self._core_server = VoIPServerComputer
        self._dialplan_context = VoIPServerConfig.get_param_value("DialplanContext")  # "st_audiocomms_users"
        # In Asterisk, the application parameter determines the type of action to perform when calling a specific
        # number/SIP peer
        # SIP accounts are listed in the dialplan (see header)
        self.channel_status = {'call_id': [],
                               'sip_account': [],
                               'app': [],
                               'channel_nbr': 0,
                               'call_nbr': 0}
        self._logger = logger

    def send_cmd_to_asterisk(self, cmd):
        """
        Sends a command to Asterisk's CLI

        :param cmd: command to send to the CLI
        :type cmd: str
        """
        cmd = 'asterisk -rx \"%s\"' % cmd
        result = self._core_server.run_cmd(cmd)
        self._logger.debug('RES: \n' + result['std'])
        return result['std']

    def restart_asterisk(self):
        """
        Restarts the VoIP server

        :return: None
        """
        self._logger.info("Restarting the VoIP server now...")

        # First check if the asterisk process is running on the remote computer
        self._core_server.run_cmd("ps -ef | grep asterisk | awk '{print $2}' | xargs -n 1 kill -9")

        self.send_cmd_to_asterisk("core restart now")
        time.sleep(1)

    def get_channel_status(self, show_output=False):
        """
        Gets the complete status of the VoIP server

        :param show_output: If True, prints the current channel status
        :type show_output: bool
        """
        self.channel_status['call_id'] = []
        self.channel_status['sip_account'] = []
        self.channel_status['app'] = []

        # Get the output of core show channels cmd
        ret = self.send_cmd_to_asterisk('core show channels concise')

        # From the log, extract the name(s) of the current(s) call(s)
        try:
            ret_lines = filter(None, ret.split('\n'))
            if "Privilege escalation protection disabled!" in ret_lines:
                ret_lines = [x for x in ret_lines if "SIP" in x]
            if not ret_lines:
                self._logger.info('No active call at the moment')
                return 0
        except AttributeError as e:
            self._logger.error('Issue while parsing VoIP server output: %s' % e)
            return -1

        self.channel_status['call_nbr'] = len(ret_lines)

        try:
            for k in range(len(ret_lines)):
                self.channel_status['call_id'].append(ret_lines[k].split('!')[0].split('/')[1])
                self.channel_status['sip_account'].append(ret_lines[k].split('!')[0].split('/')[1].split('-')[0])
                self.channel_status['app'].append(ret_lines[k].split('!')[5])
        except IndexError as e:
            self._logger.error("Error while getting channel status: %s" % e)

        if show_output:
            self._logger.info('Call_ID: ' + self.channel_status['call_id'])
            self._logger.info('SIP Accounts: ' + self.channel_status['sip_account'])
            self._logger.info('Extensions: ' + self.channel_status['app'])

    def get_call_id(self, sip_account):
        """
        Returns the call_id corresponding to sip_account

        :param sip_account: SIP account ID
        :type sip_account: str

        @rtype: list
        @return: list of call_id matching the input sip_account
        """
        self.get_channel_status()
        call_id = []
        if self.channel_status['call_id']:
            for k in self.channel_status['call_id']:
                if sip_account in k:
                    call_id.append(k)
        else:
            self._logger.warning("sip account %s is not currently in call" % sip_account)
        return call_id

    def start_mt_call(self, sip_account, extension):
        """
        Starts a MT call from the VoIP server

        :param sip_account: SIP account
        :type sip_account: str
        :param extension: Type of extension for the call (see class header of AudioVoipServer
            for details on the concept of extension)
        :param extension: str
        """
        try:
            self._logger.info('MT call start with %s extension' %
                              self.sip_profiles.keys()[
                                  self.sip_profiles.values().index(extension)])
        except ValueError:
            self._logger.error("Invalid call extension %s! Will do nothing." % extension)
            return

        self.get_channel_status()
        if sip_account not in self.channel_status['call_id']:
            cmd = 'channel originate SIP/%s extension %s@%s' % (sip_account, extension, self._dialplan_context)
            self.send_cmd_to_asterisk(cmd)
        else:
            self._logger.error("SIP account %s already in communication" % sip_account)

    def release_call(self, sip_account, hangup_all_channels=False):
        """
        Releases a call from the VoIP server

        :param sip_account: SIP account
        :type sip_account: str
        :param hangup_all_channels: Optional parameter that hangs all channel at once
        :type hangup_all_channels: bool
        """

        call_id = self.get_call_id(sip_account)

        if call_id:
            for i in call_id:
                msg = "Releasing all calls"
                cmd = 'channel request hangup'
                if not hangup_all_channels:
                    msg = 'Releasing call. Hangup channel %s' % i
                    cmd += ' SIP/' + i

                self._logger.info(msg)
                self.send_cmd_to_asterisk(cmd)

        else:
            self._logger.error("%s is not currently in a call, or isn't a valid SIP account name" % sip_account)

    def check_call_status(self, sip_account, retry_nb=2):
        """
        The goal of this function is to check the status of the DUT with the 'sip_account' number

        :param sip_account: SIP account
        :type sip_account: str

        :param retry_nb: number of retry for get_channel_status()
        :type retry_nb: int

        @return: Call status of the channel associated with sip_account
        @rtype: str
        """
        self._logger.info('Check the status of %s' % sip_account)
        i = 0

        # Refresh the VoIP server status
        while self.get_channel_status() <= 0 and i <= retry_nb:
            i += 1

        for k, sacc in enumerate(self.channel_status['sip_account']):
            if sacc == sip_account:
                tmp_app = self.channel_status['app'][k]
                self._logger.info('Status of %s is IN_CALL, with %s extension' %
                                  (sip_account, tmp_app))

                return 'IN_CALL'

        self._logger.info('Status of %s is OUT_OF_CALL' % sip_account)
        return 'OUT_OF_CALL'

    def change_call_profile(self, sip_account, extension):
        """
        Switches from one channel/call_id to another, defined by extension input parameter

        :param sip_account: SIP account
        :type sip_account: str
        :param extension: extension to change to
        :type: str

        @return: None
        """
        try:
            self._logger.info(
                    'Redirect %s call to extension %s' % (sip_account,
                                                          self.sip_profiles.keys()[
                                                              self.sip_profiles.values().index(extension)]))
        except ValueError:
            self._logger.error("Invalid call extension %s! Will do nothing." % extension)
            return

        call_id = self.get_call_id(sip_account)

        if call_id:
            if len(call_id) > 1:
                self._logger.error(
                        "At least 2 active calls involve the same SIP account. Cannot change the call profile for account %s" % sip_account)
            else:
                # From the call ID and the profile, create the cmd
                cmd = 'channel redirect SIP/' + call_id[0] + ' ' + self._dialplan_context + ',' + extension + ',1'
                # Send the command to the VoIP server
                ret = self.send_cmd_to_asterisk(cmd)

        else:
            self._logger.error(
                    "Unable to change call profile, %s is not currently in a call, or isn't a valid SIP account name" % sip_account)


def get_sip_account_from_call_id(call_id, logger=None):
    """
    Returns the SIP account contained in the input call_id

    The parameter call_id is given by the return value of core show channels.
    For example: it can be SIP/6001-00000001, where 6001 is the actual SIP account ID
    In the Asterisk documentation, it is referred as a channel

    :param call_id: Asterisk/VoIP server call-ID
    :type call_id: str

    @return: SIP account contained in the input call_id
    @rtype: str
    """
    try:
        return call_id.split('-')[0]
    except IndexError as ind:
        logger.error(ind)
    except AttributeError as e:
        logger.error('Error while getting sip account from Call ID: %s' % e)
