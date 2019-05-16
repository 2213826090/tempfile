from usb_apps_with_protection_enabled import *

test_result = SingleMethodRunner.run_single_test(USBProtectionEnabled,
                                                 "test_adb_install_usb_protection_disabled")

if test_result.wasSuccessful():
    print "TEST PASSED"
else:
    print "TEST FAILED"