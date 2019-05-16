from usb_apps_with_protection_enabled import *

#VERDICT = SUCCESS

test_result = SingleMethodRunner.run_single_test(USBProtectionEnabled,
                                                 "test_adb_install_usb_protection")

if test_result.wasSuccessful():
    #VERDICT = SUCCESS
    print "TEST PASSED"
else:
    # VERDICT = FAILURE
    print "TEST FAILED"