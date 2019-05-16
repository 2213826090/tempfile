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
:summary: This file implements Now Sms Server equipment
:since: 03/07/2011
:author: lvacheyx
"""
import logging
import os
import re
import socket
import time
import urllib2

from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.AcsConfigException import AcsConfigException

from Core.Report.ACSLogging import ACS_LOGGER_NAME, EQT_LOGGER_NAME
from UtilitiesFWK.Utilities import Global, run_local_command


class NowSmsServer():
    """
    Implementation of configurable NowSmsServer
    """

    def __init__(self, url, user_name, user_password):

        # Instantiate the logger
        self._logger = logging.getLogger("%s.%s.%s" % (ACS_LOGGER_NAME, EQT_LOGGER_NAME, self.__class__.__name__,))
        # NOWSMS url
        self._url = url
        # NOWSMS user
        self._user = user_name
        # NOWSMS password
        self._password = user_password
        # NOWSMS MMSC MMS Sent Today
        self._orig_mms = 0
        # Replace URL IP with Hostname (Permits to apply proxy settings on hostname if proxy is use...)
        # No more need, due to Intel LAB network configuration changes. Keep the function in case... for the future.
        # self.__init_url()


    #------------------------------------------------------------------------------
    def __init_url(self):
        """
        Replace in self._url, server IP with its Hostname:
        It permits to apply proxy settings on hostname if proxy is use.
        """

        regex_search = re.search("^http://(.*):(.*)$", self._url)
        if regex_search is not None:
            url_ip = str(regex_search.group(1))
            server_host_name = socket.gethostbyaddr(url_ip)[0]
            # Force hostname to local domain instead tl.intel.com
            server_host_name = server_host_name.replace(".tl.intel.com", ".local")
            self._url = self._url.replace(url_ip, server_host_name)
            self._logger.debug("[NOWSMS_SERVER] replace url with:(%s)." % self._url)
        else:
            raise AcsConfigException(AcsConfigException.CRITICAL_FAILURE,
                                        "[NOWSMS_SERVER] Fail to replace server IP with its Hostname (%s)" % str(e))

    #------------------------------------------------------------------------------
    def read_mms_number(self):
        """
        Read MMS number from NowSMSServer and store it
        """

        self._logger.debug("[NOWSMS_SERVER] Try to read mms number on the server.")
        # Get Now SMS Server XML Status page
        status = self.read_now_sms_server_xmlstatus()
        # Parse the return XML page to found xml tags: <MMSSentMMSC><MessagesToday>
        status = status.replace("\r", " ").replace("\n", " ")
        m = re.search('^.*\<MMSSentMMSC\>.*\<MessagesToday\>(\d+)\<.*$', status)
        if m is not None:
            self._logger.debug("[NOWSMS_SERVER] MMS number on Server: " + m.group(1))
            self._orig_mms = m.group(1)
        else:
            raise AcsConfigException(AcsConfigException.CRITICAL_FAILURE,
                                    "[NOWSMS_SERVER] Can't read MMS number on Server")

    #------------------------------------------------------------------------------
    def send(self, media, sender_number, destination_number, subject, text, attachment_file):
        """
        Prepare and send MT MMS from NowSMSServer

        This MT MMS is send by NowSMSServer to the DUT:
        1- Local computer use JAVA with a sendmms.jar file to request the SMS server a
        MMS sending (via server API), using NowSMSServer credential from "SMS Users" account:
        $ java sendmms.jar "http://10.102.161.47:8800" "telephony" "telephony" "8960"
          "0123456789" "MMS subject" "This is my MMS text" "ACS_Sample_Picture.jpg"
        2- sendmms.jar application will return ok message and mms id if MMS creation success:
        "Response Code  ->OK"
        "MMSMessageID=20131213/15/06096742"
        3- In NowSMSServer SMSC HTTP connection settings for Agilent 8960,
        URL text and binary templates must be set as following:
        /sms/send/?PhoneNumber=@@PhoneNumber@@&TEXT=@@Text@@&SENDER=@@Sender@@
        /sms/send/?PhoneNumber=@@PhoneNumber@@&DATA=@@Data@@&UDH=@@UDH@@&PID=@@PIDdecimal@@&DCS=@@DCSdecimal@@&SENDER=@@Sender@@
        Routing is also add for phone number use in destination of MT MMS (ex: 0123456789)
        4- Finally on the DUT connected to the Agilent 8960 equipment, apn (apn=MMS, type=default,mms)
        must be set accordingly to NowSMSServer MMSC, with MMSC users credential and MMSC URL set like this:
        "http://mmsc_url_IP:port/login=password"

        The sendmms.java file could be download from:
        http://www.nowsms.com/doc/submitting-mms-messages/send-mms-message-with-java

        :type media : str
        :param media : Type of the media to be sent (picture, audio, video, text)

        :type sender_number : str
        :param sender_number : Phone number of the sender (MT MMS, sender is NowSMSServer)

        :type destination_number : str
        :param destination_number : Phone number of the recipient (DUT)

        :type subject : str
        :param subject : Subject of the MMS

        :type text : str
        :param text : Text of the MMS

        :type attachment_file : str
        :param attachment_file : File to be put in attachment in the MMS
        """
        regex_search = None
        strdata = ""
        attaches = attachment_file.split(";")
        self._logger.debug(len(attaches))

        # Check media parameter
        media_type = ["picture", "audio", "video", "text"]
        if media not in media_type:
            msg = "[NOWSMS_SERVER] Media type incorrect: %s (instead %s)" % (str(media), str(media_type))
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, msg)

        for attach in attaches:

            mms_attached_file = attach
            # Call the sendmms.jar to send mms
            jar_file_path = \
                os.path.normpath(os.path.join(os.getcwd(), "..", "..", "acs_test_scripts", "Lib", "sendmms.jar"))
            cmd = ["java", "-jar", jar_file_path,
                   self._url, self._user, self._password,
                   sender_number, destination_number,
                   subject, text, mms_attached_file]

            try:
                # Debug log
                self._logger.debug(
                    "[NOWSMS_SERVER] MT MMS is going to be sent using following JAVA cmd : %s" % str(cmd))

                p, q = run_local_command(cmd, False)
                data = p.communicate()
                strdata = data[0].decode()

                # Debug log
                self._logger.debug("[NOWSMS_SERVER] JAR output:'%s'" % strdata)

                # Extract Response Code and MMSMessageID
                cmd_result = strdata.replace("\r", " ").replace("\n", " ")
                regex_search = re.search("^.*Response Code.*>(\S*).*MMSMessageID=(\S*).*$", cmd_result)

                if regex_search is not None:
                    response_code = str(regex_search.group(1))
                    mms_message_id = str(regex_search.group(2))
                    # Debug log
                    self._logger.debug("[NOWSMS_SERVER] MT MMS, Response Code = '%s'" % response_code)
                    self._logger.debug("[NOWSMS_SERVER] MT MMS, MMSMessageID = '%s'" % mms_message_id)
                else:
                    msg = "Unable to found response code."
                    raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)

            except Exception as error:
                msg = "[NOWSMS_SERVER] Exception: %s" % str(error)
                raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)

            # Check result of the operation
            if response_code != "OK" or mms_message_id == "":
                msg = "[NOWSMS_SERVER] ERROR: " + strdata
                self._logger.error(msg)
                raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)
            else:
                self._logger.info("[NOWSMS_SERVER] MT MMS request sent: MMSMessageID = '%s'" % mms_message_id)

    #------------------------------------------------------------------------------
    def read(self, mms_timeout):
        """
        Wait for notification that the MMS content is received by the server

        :type mms_timeout : int
        :param mms_timeout : MMS reception timeout
        """
        # set timeout with parameter: mms_timeout
        start_time = time.time()
        end_time = start_time + int(mms_timeout)
        while time.time() <= end_time:
            # Get Now SMS Server XML Status page
            status = self.read_now_sms_server_xmlstatus()
            # Parse the return XML page to found xml tags: <MMSSentMMSC><MessagesToday>
            if status is not None:
                status = status.replace("\r", " ").replace("\n", " ")
                m = re.search('^.*\<MMSSentMMSC\>.*\<MessagesToday\>(\d+)\<.*$', status)
                if m is not None:
                    self._logger.debug("[NOWSMS_SERVER] Waiting for new MMS received. Actual index :" + m.group(1))

                    if self._orig_mms != m.group(1):
                        self._logger.info("[NOWSMS_SERVER] MMS RECEIVED")
                        break
            time.sleep(5)
        else:
            self._logger.error("[NOWSMS_SERVER] MMS reception timeout of %s has been reached" % mms_timeout)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, "[NOWSMS_SERVER] MMS reception Timeout")

        msg = "[NOWSMS_SERVER] MMS sent in %s secs." % (time.time() - start_time)
        self._logger.debug(msg)
        return (Global.SUCCESS, msg)

    #------------------------------------------------------------------------------
    def read_now_sms_server_xmlstatus(self, use_proxy=True):
        """
        Get and return Now SMS Server XML Status Web page

        :type use_proxy : bool
        :param use_proxy : True if connection use proxy (default value for Intel LAB network)

        :rtype: str
        :return: XML Status Web page
        """

        the_page = None

        # Construct urllib request
        url = self._url + "/admin/xmlstatus?user=" + self._user + "&password=" + self._password
        self._logger.debug("[NOWSMS_SERVER] Create request for url:(%s)." % url)
        if use_proxy:
            proxy_handler = urllib2.ProxyHandler({})
            opener = urllib2.build_opener(proxy_handler)
        req = urllib2.Request(url)

        try:
            # Send the request and read the response
            self._logger.debug("[NOWSMS_SERVER] Try to open and read socket URL:(%s)." % url)

            if use_proxy:
                response = opener.open(req)
            else:
                response = urllib2.urlopen(req)

            the_page = response.read()

        except urllib2.URLError as e:
            if hasattr(e, 'reason'):
                self._logger.error("[NOWSMS_SERVER] Failed to reach a server:(%s). Reason:%s" % (url, str(e.reason)))
            elif hasattr(e, 'code'):
                self._logger.error("[NOWSMS_SERVER] The server (%s) couldn\'t fulfill the request: HTTPError %s."
                                    % (url, str(e.code)))
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                         "[NOWSMS_SERVER] Connection to NowSMS server has failed (%s)" % str(e))

        return the_page
