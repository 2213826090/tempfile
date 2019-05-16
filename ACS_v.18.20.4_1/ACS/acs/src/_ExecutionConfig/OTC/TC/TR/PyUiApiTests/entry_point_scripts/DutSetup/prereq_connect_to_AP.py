import os
from _prerequisites import *
#from PyUiApi.common.uiautomator_utils import *
from PyUiApi.adb_helper.instrumentation_utils import *


input_params = dut_manager.acs_config[u'TC_PARAMETERS']

ap_name = input_params[u'apName']
ap_password = input_params[u'apPassword']
test_run_count = input_params[u'runCount']
ap_test_page = input_params[u'apTestPage']

wifi_ap_name = dut_manager.acs_config[u'PHONE1'][u'WiFi_Connection_Ap_Name']
if wifi_ap_name:
    ap_name = wifi_ap_name
wifi_ap_passwd = dut_manager.acs_config[u'PHONE1'][u'WiFi_Connection_Passwd']
if wifi_ap_passwd:
    ap_password = wifi_ap_passwd
wifi_connection_test_page = dut_manager.acs_config[u'PHONE1'][u'WiFi_Connection_TestPage']
if wifi_connection_test_page:
    ap_test_page = wifi_connection_test_page
if not ap_test_page.startswith("http://"):
    ap_test_page = "http://" + ap_test_page.strip()

# ==========================================================================================

wifi_configs_removed_ok = False
connect_to_ap_ok = False
global test_outcome


def run_test():
    global wifi_configs_removed_ok
    global connect_to_ap_ok
    result = SystemApiTestsInterface\
        .run_instrumentation(class_name="WifiTestsDriver",
                             method_name="testRemoveWifiStoredNetworks",
                             instrumentation_args=None,
                             runner_name="GenericArgumentPassingTestRunner")
    LOG.info("wifi remove preconfigured networks: " + result)
    wifi_configs_removed_ok = InstrumentationInterface.was_instrumentation_test_successful(result)

    instrumentation_args = Template('-e apName "$apname" -e apPassword $passwd -e TEST_RUN_COUNT $runcount -e apTestPage $page')
    ap_connect_test_args = instrumentation_args.substitute(apname=ap_name,
                                                           passwd=ap_password,
                                                           runcount=test_run_count,
                                                           page=ap_test_page)
    LOG.info("ap connect test args: " + ap_connect_test_args)
    ApiTestsInterface.args_template = Template('$args')
    result = ApiTestsInterface\
        .run_instrumentation(class_name="WiFiConnectionTest",
                             method_name="testMTBFConnectToAP",
                             instrumentation_args=ap_connect_test_args,
                             runner_name="WifiEncryptionTestRunner")
    LOG.info("connect to ap:" + result)
    connect_to_ap_ok = InstrumentationInterface.was_instrumentation_test_successful(result)

# ==========================================================================================
try:
    run_test()
finally:
    AdbUtils.kill_python_uiautomator_rpc_server_on_dut()

if wifi_configs_removed_ok and connect_to_ap_ok:
    print "TEST_PASSED"
    test_outcome = True
else:
    test_outcome = False
    sys.exit("FAIL")