"""

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

@summary: This file implements a Test Step to run a HTTP server
@since 8 July 2014
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV
"""

import time
import os
import logging
from Core.TestStep.DeviceTestStepBase import TestStepBase
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager
import acs_test_scripts.Utilities.ValidationFileServers as ValidationFileServers

class RunHttpServerHost(TestStepBase):
    """
    Install a file on a host
    """
    def run(self, context):
        """
        Run a HTTP server

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        self._device_manager = DeviceManager()
        report_path =  self._device_manager.get_global_config().campaignConfig.get("campaignReportTree").get_report_path()
        host_subdir = os.path.join(report_path, self._tc_parameters.get_name(), self._pars.dest_subdir) # Destination path

        self._logger.debug("Report path is {0} folder".format(report_path))
        self._logger.debug("TC name is {0} folder".format(self._tc_parameters.get_name()))

        #Create the destination directory
        if not os.path.exists(host_subdir):
            self._logger.debug("make {0} folder".format(host_subdir))
            os.makedirs(host_subdir)

        http_serve_path = os.path.join(host_subdir,'http_serve')
        if os.path.exists(http_serve_path)==False:
            os.makedirs(http_serve_path)
        # We will be spawning the http server off on another thread. We need a way to
        #  tell it to stop running. We'll use a file called .http_running
        http_running_file_path = os.path.join(host_subdir, '.http_running')
        if os.path.exists(http_running_file_path):
            # The file exists which means the server might be running. Remove the file
            #  and wait a short time period to make sure the server stops.
            self._logger.info("HTTP server running file (%s) found. Assuming an error in previous test run. Removing file to kill server."%http_running_file_path)
            os.remove(http_running_file_path)
            time.sleep(5)
        # Create a log file for our HTTP server so we can try and get some debug output

        httpd_log = self.create_logger( name="FILE_DOWNLOAD_HTTPD", file_path=os.path.join(host_subdir, 'httpd.log'), console_logging_level=logging.WARNING)
        http_server = ValidationFileServers.start_http_server(document_root=http_serve_path, run_file=http_running_file_path, port=None, logger=httpd_log)
        # Start our HTTP server
        httpd_log.info("Starting HTTP server on port %d with document root %s."%(http_server.server_address[1], http_serve_path))
        self._logger.debug("The HTTP server will continue to run until the file %s is removed."%http_running_file_path)

        if os.path.exists(os.path.join(host_subdir, '.http_running')):
            self._logger.info("HTTP server is started on port %d with document root %s."%(http_server.server_address[1], http_serve_path))
            context.set_info(self._pars.stored_httpd_root_path, http_serve_path)
        else:
            msg = "Failed to start HTTP server on port %d with document root %s."%(http_server.server_address[1], http_serve_path)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def create_logger(self, name="", file_path=None, console_logging_level=logging.DEBUG):
        '''
           Creates a Python logger object that prints to the screen and logs to a file
           (if file_path is supplied). To use the logger object that is returned, use
           these functions and more (see the Python documentation):
               log.debug("This is my debug message")
               log.info("This is my info message. Info is less detailed than debug")
               log.warning("Warning is more important than info.")
               log.error("Error is more important than warning.")

           PARAM name="": The name of the logger. Allows you to retrieve a logger previously created.
           PARAM file_path=None: Where to save the log file. If no path is provided, no file will be used.
        '''
        log = logging.getLogger(name)
        log.setLevel(logging.DEBUG)
        log_format = logging.Formatter('%(asctime)s - %(name)s - %(message)s',"%Y-%m-%d %H:%M:%S")
        if file_path != None:
            if os.path.exists(os.path.dirname(file_path))==False:
                os.makedirs(os.path.dirname(file_path))
            log_file_handler = logging.FileHandler(filename=file_path)
            log_file_handler.setLevel(logging.DEBUG)
            log_file_handler.setFormatter(log_format)
            log.addHandler(log_file_handler)
        return log