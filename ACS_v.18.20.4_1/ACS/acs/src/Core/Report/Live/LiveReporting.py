#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG/AMPS/SI/SIS/CTA
@summary: The module will represent live reporting module
@since: 24/11/14
@author: sfusilie
"""
# pylama: ignore=E501
import os

from Core.CampaignMetrics import CampaignMetrics
from Core.PathManager import Files
from Core.PathManager import Folders
from Core.PathManager import Paths
from Core.Report.ACSLogging import LOGGER_FWK
from Core.Report.Live.LiveReportingPlugin import LiveRerportingPluginLoader
from Core.Report.Live.ReportUtils.ReportApi import ReportApi
from Core.Report.Live.ReportUtils.ReportLogging import REPORT_FILE_NAME
from Core.Report.Live.TSMBLiveReporting import TSMBLiveReporting
from Core.Report.scripts.acsTCRZipPush import TCRpush
from UtilitiesFWK.Patterns import Singleton
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.Utilities import get_acs_release_version


@Singleton
class LiveReporting(object):

    """
    Class that will deal with live reporting event with AWR and TCR.
    """

    # Test Campaign Report instance (TCR)
    _tcr_instance = None
    # TSMB Live reporting instance (TSMB)
    _tsmb_instance = None
    # Live reporting plugin instance
    _reporting_plugin_instance = None

    _report_name = ""

    def _get_campaign_infos(self):
        campaign_metrics = CampaignMetrics.instance()
        campaign_info = {"execution_rate": campaign_metrics.execution_rate,
                         "pass_rate": campaign_metrics.pass_rate,
                         "fail_rate": campaign_metrics.fail_rate,
                         "blocked_rate": campaign_metrics.blocked_rate,
                         "valid_rate": campaign_metrics.valid_rate,
                         "invalid_rate": campaign_metrics.invalid_rate,
                         "inconclusive_rate": campaign_metrics.inconclusive_rate,
                         "stats": {"TotalBootCount": campaign_metrics.total_boot_count,
                                   "ConnectFailureCount": campaign_metrics.connect_failure_count,
                                   "MeanTimeBeforeFailure": campaign_metrics.mtbf,
                                   "TimeToCriticalFailure": campaign_metrics.time_to_first_critical_failure,
                                   "CriticalFailureCount": campaign_metrics.critical_failure_count,
                                   "UnexpectedRebootCount": campaign_metrics.unexpected_reboot_count,
                                   "BootFailureCount": campaign_metrics.boot_failure_count}
                         }
        return campaign_info

    @property
    def campaign_url(self):
        return self._tcr_instance.campaign_url if self._tcr_instance else ""

    def init(self,
             report_dir,
             enable_tcr=False,
             tcr_dev_report=False,
             enable_tsmb=False,
             campaign_uuid=None,
             live_reporting_plugin=None,
             rerun=False):
        """

        :param report_dir: report folder directory
        :param enable_tcr: enable or not TCR live reporting
        :param tcr_dev_report: specify if tcr is dev server
        :param enable_tsmb: enable or not TSMB publishing
        :param campaign_uuid: uuid of the campaign
        :param live_reporting_plugin: live reporting plugin to instantiate
        :param rerun: rerun mode
        """
        # Initialize TCR (Test Campaign reporting tool)
        if enable_tcr:
            self._tcr_instance = ReportApi(campaign_uuid=campaign_uuid,
                                           dev_report_mode=tcr_dev_report,
                                           report_dir=report_dir,
                                           test_framework="ACS",
                                           test_framework_version=get_acs_release_version(),
                                           rerun=rerun)
        # Initialize live reporting to send TSMB messages
        if enable_tsmb:
            self._tsmb_instance = TSMBLiveReporting(report_dir)

        if live_reporting_plugin:
            self._reporting_plugin_instance = LiveRerportingPluginLoader.load(live_reporting_plugin)
            self._reporting_plugin_instance.is_reporting_enabled = True

        if report_dir:
            self._report_name = report_dir

    @property
    def campaign_id(self):
        """
        Get the metacampaign result id
        """
        res_id = None
        if self._tcr_instance:
            res_id = self._tcr_instance.campaign_id

        return res_id

    def enable_live_reporting(self, enable):
        """
        Enable/Disable live reporting

        :param enable : bool
        :param enable : Specify if live reporting is enabled or not
        """
        if self._tcr_instance:
            self._tcr_instance.is_reporting_enabled = enable
        if self._tsmb_instance:
            self._tsmb_instance.is_reporting_enabled = enable
        if self._reporting_plugin_instance:
            self._reporting_plugin_instance.is_reporting_enabled = enable

    def send_start_campaign_info(self, test_suite_uuid, campaign_name, user_email="no.name@intel.com"):
        """
        Notify the remote script that a campaign just stopped.

        :type test_suite_uuid: str
        :param test_suite_uuid: Test suite execution unique identifier

        :type campaign_name: str
        :param campaign_name: The name of the relevant campaign

        :param user_email: user email
        :type user_email: str

        :rtype: None
        """
        if self._tcr_instance:
            self._tcr_instance.start_campaign(campaign_uuid=test_suite_uuid,
                                              campaign_name=campaign_name,
                                              email=user_email)
        if self._tsmb_instance:
            self._tsmb_instance.send_start_campaign_info(test_suite_uuid,
                                                         campaign_name,
                                                         user_email)
        if self._reporting_plugin_instance:
            self._reporting_plugin_instance.start_campaign(campaign_uuid=test_suite_uuid,
                                                           campaign_name=campaign_name,
                                                           email=user_email)

    def send_stop_campaign_info(self, verdict, status):
        """
        Notify the remote script that a campaign just stopped.

        :type verdict: campaign verdict
        :param verdict: str

        :type status: campaign execution status
        :param status: str
        """
        if self._tcr_instance:
            self._tcr_instance.stop_campaign(status, verdict, self._get_campaign_infos())
            if self._tcr_instance.campaign_url:
                self.create_url_shortcut(campaign_url=self._tcr_instance.campaign_url)
        # TCR is not yet ready to support campaign status, provide legacy verdict for now
        if self._tsmb_instance:
            self._tsmb_instance.send_stop_campaign_info(verdict)

        if self._reporting_plugin_instance:
            self._reporting_plugin_instance.stop_campaign(status, verdict, self._get_campaign_infos())

    def send_create_tc_info(self, tc_name, uc_name, tc_phase, tc_type, tc_domain, tc_order, is_warning, tc_parameters=None):

        tc_id = None
        # Update TCR Reporting tool via REST API interface
        if self._tcr_instance:
            tc_id = self._tcr_instance.create_testcase(tc_name,
                                                       uc_name,
                                                       tc_phase,
                                                       tc_type,
                                                       tc_domain,
                                                       tc_order,
                                                       is_warning,
                                                       tc_parameters,
                                                       get_acs_release_version())
            # Each method of ReportApi return a dictionary or None
            if isinstance(tc_id, dict):
                tc_id = tc_id.get('payload', {}).get('id')

        if self._reporting_plugin_instance:
            self._reporting_plugin_instance.create_testcase(tc_name,
                                                            uc_name,
                                                            tc_phase,
                                                            tc_type,
                                                            tc_domain,
                                                            tc_order,
                                                            tc_parameters,
                                                            get_acs_release_version())

        return tc_id

    def send_start_tc_info(self,
                           tc_name,
                           tc_order=None,
                           iteration=False,
                           device_info=None):
        """
        Notify the remote script that a test case just started.

        :type tc_name: str
        :param tc_name: test case name

        :type tc_order: int
        :param tc_order: Test case order in test suite execution

        :type iteration: (optional) bool
        :param iteration: Specify if this testcase is a sub test case of a top level one

        :param device_info: (optional)Additional device info of the dut.
        :type device_info: dict
        """

        device_info = device_info or {}

        # Update TCR Reporting tool via REST API interface
        if self._tcr_instance:
            self._tcr_instance.start_testcase(tc_name=tc_name,
                                              tc_order=tc_order,
                                              iteration=iteration,
                                              device_info=device_info.get("TCR", {}))
            if self._tcr_instance.campaign_url:
                self.create_url_shortcut(campaign_url=self._tcr_instance.campaign_url)

        # Update TCR Reporting tool via TSMB interface
        # BE CAREFUL: it is not up to date with TCR data sent via REST API
        if self._tsmb_instance:
            self._tsmb_instance.send_start_tc_info(tc_name, tc_order)

        if self._reporting_plugin_instance:
            self._reporting_plugin_instance.start_testcase(tc_name=tc_name,
                                                           tc_order=tc_order,
                                                           iteration=iteration,
                                                           device_info=device_info.get("TCR", {}))

    def create_bulk_tc(self, tc_data):
        """
        Notify the remote script that a bulk test case just created.

        :param payload: list of dict
        :type payload: list
        """
        # Split tc_data by 100 TestCases for avoid overload the TCR RestAPI
        tc_data_max_size_to_push = 100
        # Update TCR Reporting tool via REST API interface
        if self._tcr_instance and isinstance(tc_data, list):
            while len(tc_data) > 0:
                self._tcr_instance.create_bulk_testcases(tc_data[:tc_data_max_size_to_push])
                import time
                time.sleep(1)
                del tc_data[:tc_data_max_size_to_push]

    def update_running_tc_info(self,
                               crash_list=None,
                               test_info=None,
                               device_info=None,
                               iteration=False):
        """
        Update a test case only for framework call

        :type crash_list: crash id list
        :param crash_list: Current crash list

        :param test_info: (optional) Additional test case result info (can test step,
                                                                       external test case results, other ..)
                                    (Dictionary key = section name, data = json serializable value data)

        :param device_info: (optional) Additional device info of the dut.
        :type device_info: dict

        :type iteration: bool
        :param iteration: Specify if this testcase is an iteration of a top level one

        """

        device_info = device_info or {}
        test_info = test_info or {}
        crash_list = crash_list or []

        # In some cases, the call is done without any data
        # to push onto Reporting interface.
        has_some_data2push = any([crash_list, test_info, device_info])

        if self._tcr_instance and has_some_data2push:
            # Update TCR Reporting tool via REST API interface
            self._tcr_instance.update_testcase(crash_list=crash_list,
                                               test_info=test_info,
                                               device_info=device_info.get("TCR", {}),
                                               iteration=iteration)

        if self._tsmb_instance:
            # Update TCR Reporting tool via TSMB interface
            # BE CAREFUL: it is not up to date with TCR data sent via REST API
            self._tsmb_instance.update_running_tc_info(crash_list=crash_list,
                                                       test_info=test_info,
                                                       device_info=device_info.get("TCR", {}))

        if self._reporting_plugin_instance and has_some_data2push:
            self._reporting_plugin_instance.update_testcase(crash_list=crash_list,
                                                            test_info=test_info,
                                                            device_info=device_info.get("TCR", {}),
                                                            iteration=iteration)

    def send_test_case_resource(self, resource, display_name=None, retention="SHORT", iteration=False):
        """
        Push a resource onto (TCR Only) REST API for a given Test Case.

        :param str resource: Local resource to be pushed onto TCR at Test Case level.
        :param str display_name: (optional) Filename to be displayed in the UI
        :param str retention: (optional) "SHORT" or "LONG"
        :param bool iteration: (optional) True if the test case has at least two iterations

        """
        if self._tcr_instance:
            # Push a resource to TCR Reporting tool via REST API interface (Test case level)
            self._tcr_instance.send_testcase_resource(resource,
                                                      display_name=display_name,
                                                      retention=retention,
                                                      iteration=iteration)
        else:
            LOGGER_FWK.warning('Only TCR offers REST API for resources!')

    def send_test_case_chart(self, chart_info, iteration=False):
        """
        Attach a chart to the current test case

        :param dict chart_info: data to build the chart on the server (title, series, axis ...)
        :param bool iteration (optional): True if the test case has at least two iterations
        """
        if self._tcr_instance:
            # Push a resource to TCR Reporting tool via REST API interface (Test case level)
            self._tcr_instance.send_testcase_chart(chart_info, iteration=iteration)
        else:
            LOGGER_FWK.warning('Only TCR offers REST API for charts!')

    def send_campaign_resource(self, campaign_uuid, resource_info):
        """
        Push a resource onto (TCR Only) REST API for a given Test Case.

        :param dict campaign_uuid: id of the campaign.
        :param dict resource_info: Local resource to be pushed onto TCR at Test Case level.
        """
        if self._tcr_instance:
            # Archive file to push
            context_dict = {
                "acs_logfile_name": Files.acs_output_name,
                "TCR_LIVE_REPORTING": REPORT_FILE_NAME,
                "ROOT_DUT_NAME": Folders.ROOT_DUT_NAME,
                "AP_LOGS": Folders.AP_LOGS,
                "BP_LOGS": Folders.BP_LOGS,
                "PTI_LOGS": Folders.PTI_LOGS,
                "SERIAL_LOGS": Folders.SERIAL_LOGS,
                "LOGCAT_LOGS": Folders.LOGCAT_LOGS,
                "DEBUG_LOGS": Folders.DEBUG_LOGS,
                "REPORT_STYLE_FILENAME": Files.REPORT_STYLE
            }
            tcr_push = TCRpush(
                original_acs_report=resource_info["campaign_report_path"],
                metacampaign_uuid=campaign_uuid,
                user_mail=resource_info["user_email"],
                cache_reports=Paths.CACHE_PUSH_REPORTS,
                dev_campaign=resource_info["dev_campaign"],
                log_folders_and_files=context_dict
            )
            # Build archive to push
            status, msg = tcr_push.setup()
            if status == Global.SUCCESS:
                # Push a resource to TCR Reporting tool via REST API interface (Test case level)
                self._tcr_instance.send_campaign_resource(resource=tcr_push.zip_acs_report_tcr)
            else:
                LOGGER_FWK.error("Error when building campaign result archive! {}".format(msg))
        else:
            LOGGER_FWK.warning('Only TCR offers REST API for resources!')

    def send_stop_tc_info(self,
                          verdict,
                          execution_nb,
                          success_counter,
                          max_attempt,
                          acceptance_nb,
                          tc_comments=None,
                          tc_parameters=None,
                          tc_description=None,
                          tc_b2b_iteration=None,
                          tc_b2b_continuous=None,
                          tc_expected_results=None,
                          iteration=False,
                          device_info=None):
        """
        Notify the remote script that a test case just stopped.

        :type verdict: Global.Verdict
        :param verdict: The global verdict for this TC

        :type execution_nb: int
        :param execution_nb: Number of test execution

        :type success_counter: int
        :param success_counter: Number of success run

        :type max_attempt: int
        :param max_attempt: Maximum number of test execution attempt

        :type acceptance_nb: int
        :param acceptance_nb: Minimal pass number to have the verdict success

        :type tc_comments: list
        :param tc_comments: list of test execution comments

        :type tc_parameters: dict
        :param tc_parameters: dict of test execution parameters

        :type tc_description: string
        :param tc_description: Description of the tc

        :type tc_b2b_iteration: int
        :param tc_b2b_iteration: tc B2B iteration value

        :type tc_b2b_continuous: bool
        :param tc_b2b_continuous: tc B2B iteration value

        :type tc_expected_results: string
        :param tc_expected_results: the expected result for the test case.

        :type iteration: bool
        :param iteration: Specify if this test case is an iteration of a top level one

        :param device_info: (optional)Additional device info of the dut.
        :type device_info: dict
        """
        tc_properties = {
            "acceptanceCriteria": acceptance_nb,
            "maximumTries": max_attempt,
            "successCount": success_counter,
            "triesCount": execution_nb,
            "description": tc_description,
            "b2b iteration": tc_b2b_iteration,
            "b2b continuous": tc_b2b_continuous,
            "expected results": tc_expected_results,
        }

        if self._tcr_instance:
            self._tcr_instance.stop_testcase(verdict=verdict,
                                             execution_nb=execution_nb,
                                             tc_parameters=tc_parameters,
                                             tc_properties=tc_properties,
                                             tc_comments=tc_comments,
                                             iteration=iteration,
                                             device_info=(device_info or {}).get("TCR", {}))

        if self._tsmb_instance:
            self._tsmb_instance.send_stop_tc_info(verdict)

        if self._reporting_plugin_instance:
            self._reporting_plugin_instance.stop_testcase(verdict=verdict,
                                                          execution_nb=execution_nb,
                                                          tc_parameters=tc_parameters,
                                                          tc_properties=tc_properties,
                                                          tc_comments=tc_comments,
                                                          iteration=iteration,
                                                          device_info=(device_info or {}).get("TCR", {}))

    def get_testcases(self, campaign_id, iteration=False):
        '''
        get Campaign test cases

        :param campaign_id: campaign ID
        :iteration: whether to fetch iteration test case

        return a list of test case associated to this Campaign
        '''
        if self._tcr_instance:
            return self._tcr_instance.get_testcases(campaign_id, iteration)

    def is_rerun(self, tc_name):
        '''
        check if test case is rerun needed
        '''
        if self._tcr_instance:
            return self._tcr_instance.is_rerun(tc_name)
        return True

    def wait_for_finish(self):
        """
        Wait for Live reporting requests to be finished

        :rtype: bool
        :return: True if timeout hasn't been reached, False otherwise
        """
        if self._tsmb_instance:
            self._tsmb_instance.close()

    def create_url_shortcut(self, campaign_url):
        """
        Create a shortcut to open given url

        :rtype: tuple
        :return: Status and output log
        """
        try:
            if os.path.exists(Folders.REPORTS):
                output_path = os.path.join(Folders.REPORTS, self._report_name + ".html")

                if not os.path.isfile(output_path):
                    LOGGER_FWK.info("CREATE_URL_SHORTCUT: Creating url shortcut to campaign result")

                    html_content = "<html>\n"
                    html_content += "<head>\n"
                    html_content += "<meta http-equiv=\"refresh\" content=\"0; URL=%s\">\n" % campaign_url
                    html_content += "</head>\n"
                    html_content += "<body></body>\n"
                    html_content += "</html>"

                    with open(output_path, "w") as fd:
                        fd.write(html_content)

        except Exception as ex:  # pylint: disable=W0703
            error_msg = "CREATE_URL_SHORTCUT: Fail, " + str(ex)
            LOGGER_FWK.error(error_msg)
            return Global.FAILURE, error_msg

        msg = "CREATE_URL_SHORTCUT: Created link to %s" % str(campaign_url)
        return Global.SUCCESS, msg
