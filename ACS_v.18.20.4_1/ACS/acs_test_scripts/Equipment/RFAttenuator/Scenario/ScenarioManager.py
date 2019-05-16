"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG
:summary: This file implements the scenario parser to run a campaign with the RF Attenuator.
:since: 2014-08-12
:author: emarchan

"""

from csv import reader as csv_reader
from os import path as os_path
from os import makedirs as os_makedirs
from tempfile import NamedTemporaryFile
from urllib import urlretrieve
from time import sleep
from re import search as re_search
from threading import Timer, Event
from subprocess import call, check_output
from time import time, mktime
from datetime import datetime
from LogicDataDebug import LogicDataDebug

SHIFT_COLUMN = 3  # to avoid time stamp , long and lat columns
MISSING_ATTENUATORS = 0  # Eric's attenuator :)
DEFAULT_TIME_BETWEEN_SET = 1
TEST_APK = 'NONE'
event_all_threads_synchro = Event()

class ScenarioManager(object):
    """
    This class is intended to parse a scenario in CSV (separator ;).
    It will set the variable-attenuators values.
    """
    def __init__(self, scenario_uri, time_between_set):
        """
        Creator

        :type scenario_uri: string
        :param scenario_uri: location of the scenario. It can be a local file or an URI
        """
        self._scenario_uri = scenario_uri
        self._fixed_time_between_set = DEFAULT_TIME_BETWEEN_SET
        self._scenario_values = []
        self._num_of_aps = 0
        self._ap_list = None
        self._attenuators_insts = []
        self._line_number = 1  # starts by 1 because in RSSI file scenario starts from line 1
        self._save_line = []
        self._save_file = []
        abs_path = os_path.abspath(os_path.dirname(__file__))
        log_path = os_path.join(abs_path, "log_files")
        self._abs_log_name = os_path.join(log_path, 'HOST_UC_NAV3_' + str(datetime.now().strftime('%Y_%m_%d___%H_%M_%S')) + '.txt')

        # If logs folder doesn't exist, create it.
        if not os_path.exists(log_path):
            os_makedirs(log_path)

        self._abs_script_hostapd = os_path.join(abs_path, "Scripts_HOSTAP", 'edit_hostapd.sh')


        self._log = LogicDataDebug(None, self._abs_log_name)

    def set_fixed_time_between_set(self, time_between_set):
        """
        Sets the number of second between each attenuation change.

        :type time_between_set: int
        :param time_between_set: Number of second between each attenuation change.
        """
        self._fixed_time_between_set = time_between_set

    def create_attn_list(self):
        """
        Parses the CSV file containing the attenuation to apply during the test.
        """
        file_content = None
        if self._scenario_uri.startswith('http'):
            file_to_load = NamedTemporaryFile(delete=True)
            urlretrieve (self._scenario_uri, file_to_load.name)
            file_content = open(file_to_load.name, 'r')
            self._log.info("SCENARIO_FILE=%s" % file_to_load.name)
        else:
            if (os_path.isfile(self._scenario_uri)):
                file_content = open(self._scenario_uri, 'r')
                self._log.info("SCENARIO_FILE=%s" % os_path.basename(self._scenario_uri))
            else:
                self._log.error("Can't open the scenario file %s" % self._scenario_uri)
        if file_content is not None:
            self._log.info("Loading scenario...")
            reader = csv_reader(file_content, delimiter=';')  # semi_column
            for row in reader:
                if reader.line_num == 1:
                    self._num_of_aps = len(row) - SHIFT_COLUMN
                    self._log.info("Found %d APs" % self._num_of_aps)
                    self._ap_list = row[SHIFT_COLUMN:len(row)]
                    self._log.debug("APs list is: %s" % self._ap_list)
                else:
                    self._scenario_values.append(row)

    def instantiate_attenuators(self, attn_type, ip_base_addr):
        """
        Instantiate the attenuators that will be driven by the scenario.

        :type attn_type: class
        :param attn_type: Type of attenuator used
        :type ip_base_addr: string
        :param ip_base_addr: IP address for the 1st attenuator (assuming they are following each others in the range)
        """
        self._log.info("Instantiating the attenuators...")
        ip_split = re_search('^(.*\.)(\d{1,3})$', ip_base_addr)
        if ip_split is not None:
            ip_split = ip_split.groups()
            cur_ip_base = ip_split[0]
            cur_ip_ending = int(ip_split[1])
            if self._ap_list is None:
                self._log.error("Please parse the scenario file before instantiating the equipments!")
            else:
                for i in range(len(self._ap_list) - MISSING_ATTENUATORS):
                    cur_ip = cur_ip_base + str(cur_ip_ending + i)
                    cur_inst = attn_type(cur_ip)
                    self._attenuators_insts.append(cur_inst)
                    cur_inst.configure()

    def run_scenario(self):
        """
        Launch the scenario execution.
        """
        #---------------------sync DUT and PC : get curr time from PC and set it to DUT  ------------------------
        self._log.info ("----------------------------------------sync DUT and PC----------------------------------------")
        if TEST_APK == 'rx_networks_apk' :
            self._launch_rx_networks_apk()
        elif TEST_APK == 'gms_apk' :
            self._gms_apk()
        else :
            self._log.warning ("APK test is not specified, test setup is not using adb commands")

        self._log.info ("APK_TEST=%s" % TEST_APK)
        #---------------------start scenario------------------------
        t_previous = 0
        t0 = 0
        self._log.info ("Scenario execution starts. Duration will be %ds." % len(self._scenario_values) * self._fixed_time_between_set)
        t0_all = datetime.now()
        self._log.info("START_SCENARIO=%s" % mktime(t0_all.timetuple()))
        t0 = t0_all
        # Execute each line in in RSSI file
        for cur_values in self._scenario_values:
            t_previous = t0
            t0 = datetime.now()
            # Compute execution time for each line to ensure it will be exactly 1s.
            delta_line = t0 - t_previous

            self._log.info("HOST_LINE_NUM=%s" % self._line_number)
            self._log.info("HOST_TIMESTAMP=%d" % int(mktime(t0.timetuple())))
            self._log.info("LATTITUDE=%s , LONGITUDE=%s" % (cur_values[1], cur_values[2]))
            self._log.info("HOST_DURATION_BETWEEN_LINES=%s" % delta_line)
            self._log.info("attenuation values are: " + ", ".join(str(f) for f in cur_values[SHIFT_COLUMN:len(cur_values)]))
            thread_line = Timer(0, self._thread_line_execution, (cur_values, SHIFT_COLUMN, t0))
            thread_line.start()
            sleep(self._fixed_time_between_set)
            self._line_number = self._line_number + 1
        thread_line.join()
        if self._line_number != len(self._scenario_values):
                self._log.error("executed line number and input_scenario file number does not match, it's an error on the script that misses an execution line, expected execution lines are %s, and real executed lines are %s" % (self._line_number, len(self._scenario_values)))
        self._log.info("END_SCENARIO=%d" % int(time()))
        # send adb command to APK test to stop logging and to close.
        if TEST_APK == 'rx_networks_apk' :
            self._close_rx_networks_apk()
        elif TEST_APK == 'gms_apk' :
            self._close_gms_apk()
        else :
            self._log.warning ("APK test is not specified, APK test should stop logging at this time=%s" % datetime.now())

        # compute test duration and comparison with expected execution time
        t_end_all = datetime.now()
        delta_measured = float((t_end_all - t0_all).total_seconds())
        expected_duration = len(self._scenario_values) * self._fixed_time_between_set
        self._log.info ("*************************final results**********************************")
        self._log.info ("HOST_EXPECTED_DURATION %s" % expected_duration + " second")
        self._log.info ("HOST_MEASURED_DURATION %s" % delta_measured + " second")
        Delta_duration = delta_measured - expected_duration
        self._log.info ("Delta %f" % Delta_duration + " second")
        Deviation = 100 * Delta_duration / expected_duration
        self._log.info ("Deviation %f" % Deviation + " %")

        self._log.info("scenario start at %s" % mktime(t0_all.timetuple()))
        self._log.info("scenario end at %s" % mktime(t_end_all.timetuple()))
        self._log.info ("********************************************************************")

    def _launch_rx_networks_apk(self):
        # Set DUT timezone to UTC
        call("adb shell setprop persist.sys.timezone 'Etc/UTC'", shell=True)
        dut_timezone = check_output('adb shell getprop persist.sys.timezone', shell=True)
        if (dut_timezone[0:len(dut_timezone) - 2] == 'Etc/UTC'):
            self._log.info("DUT's time zone is set successfully to UTC timezone")
        else:
            self._log.warning("could not set DUT time zone, DUT current timezone is %s" % dut_timezone)

        self._log.info("set DUT new time")
        PC_current_time = datetime.fromtimestamp(time()).strftime('%Y%m%d.%H%M%S')
        call('adb shell date -s' + PC_current_time, shell=True)

        self._log.info("-------------------------compare timing--------------------------------------------------------------")
        time_zone = mktime((datetime.now()).timetuple()) - mktime((datetime.utcnow()).timetuple())
        DUT_current_time = float(check_output('adb shell date +%s', shell=True)) - time_zone
        PC_current_time = time()
        self._log.info("DUT_TIMESTAMP=%s" % DUT_current_time)
        self._log.info("HOST_TIMESTAMP=%s" % PC_current_time)
        Time_drift = PC_current_time - DUT_current_time
        if Time_drift > 1:
            self._log.warning("Drift between DUT and PC is greater than one second ")
        self._log.warning("PC_DUT_TIME_DRIFT=%s" % Time_drift)

        self._close_rx_networks_apk()
        launch_apk = check_output('adb shell am start -n com.rxnetworks.xybriddemoclient/com.rxnetworks.xybriddemoclient.DemoActivity', shell=True)
        self._log.info ("Launch RX networks apk-----------%s" % launch_apk)

    def _close_rx_networks_apk(self):
        close_apk = check_output('adb shell am force-stop com.rxnetworks.xybriddemoclient', shell=True)
        self._log.info("close Rx networks apk just in case it still open  %s" % close_apk)

    def _launch_gms_apk(self):
        # still under construction
        self._log.warning("PC_DUT_TIME_DRIFT=%s" % Time_drift)
        self._close_gms_apk()
        launch_apk = check_output('adb shell am start -n com.intel.locationtest/com.intel.locationtest.MainActivity', shell=True)
        self._log.info ("Launch RX networks apk-----------%s" % launch_apk)

    def _close_gms_apk(self):
        close_apk = check_output('adb shell am force-stop com.intel.locationtest', shell=True)
        self._log.info("close GMS apk just in case it still open  %s" % close_apk)

    #-----------------------Scheduling_task----------------------------------------
    def _thread_line_execution(self, cur_values, SHIFT_COLUMN, t0):
        event_all_threads_synchro.clear()
        for curr_aps in range (self._num_of_aps - MISSING_ATTENUATORS):
            # check the cell in RSSI file, if it is a value or string.
            is_attenuation_value = False
            try:
                cur_values_in_float = float(cur_values[curr_aps + SHIFT_COLUMN])
                is_attenuation_value = True
            except ValueError:
                pass
            # cell in RSSI file is a value, so "is_attenuation_value = True" and set an attenuation
            if is_attenuation_value is True:
                attenuation = cur_values[curr_aps + SHIFT_COLUMN]
                Timer(0, self._thread_attenuation, (attenuation, curr_aps)).start()
            else:
                # set max attenuation value
                attenuation = self._attenuators_insts[curr_aps]._max_attn
                Timer(0, self._thread_attenuation, (attenuation, curr_aps)).start()
                # AP handover
                Timer(0, self._thread_handover, (cur_values, SHIFT_COLUMN, curr_aps)).start()
            # Synchronize all threads starts
        event_all_threads_synchro.set()

    def _thread_attenuation(self, attenuation, curr_aps):
        # lock current thread until all  threads are created, so they start all simultaneously
        event_all_threads_synchro.wait()
        t_start_att = datetime.now()
        self._attenuators_insts[curr_aps].set_attenuation(attenuation)
        t_end_att = datetime.now()
        # delta_att : compute execution time of each attenuation
        delta_att = (t_end_att - t_start_att).total_seconds()
        self._log.debug("HOST_ATTENUATION_DURATION_%s=%s s, related to line %s, START_TIME at %s, END_TIME at %s, ATT_VALUE is %s"
                        % (curr_aps, delta_att, self._line_number, t_start_att, t_end_att, attenuation))

    def _thread_handover(self, cur_values, SHIFT_COLUMN, curr_aps):
        event_all_threads_synchro.wait()
        t0_handover = datetime.now()
        AP_charc = cur_values[curr_aps + SHIFT_COLUMN].split('|', 2);
        if (len(AP_charc) == 3) :
            SSID = AP_charc[0]
            MAC_address = AP_charc[1]
            channel = AP_charc[2]
            wlan = "wlan" + str(curr_aps + 1)
            call(['sudo', 'sh', self._abs_script_hostapd, str(wlan), str(SSID), str(MAC_address), str(channel)])

            # TODO check all script output

            delta_handover = (datetime.now() - t0_handover).total_seconds()
            self._log.debug("HOST_HANDOVER_DURATION_%s=%s, related to line %s, START_TIME at %s, END_TIME at %s" % (SSID, delta_handover, self._line_number, t0_handover, datetime.now()))
        elif AP_charc[0].lower() != "handover":
            self._log.error ('RSSI file not compatible, unknown value in cell %s,%s' % (curr_aps + SHIFT_COLUMN, self._line_number))
