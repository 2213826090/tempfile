from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.api_tests_wrapper import *

#instrumentation used
StorageUSBApiTestsWrapper.instrumentation_runner_name = "GenericArgumentPassingTestRunner"
StorageUSBApiTestsWrapper.instrumentation_method_name = "testCreateFile"
StorageUSBApiTestsWrapper.instrumentation_class_name = "SystemStorageUSBTestsDriver"
StorageUSBApiTestsWrapper.instrumentation_args = "createDir:" + EnvironmentUtils.get_emulated_storage_path() + " nrOfFiles:10000 fileSizeKB:200"
test_result = SingleMethodRunner.run_single_test(StorageUSBApiTestsWrapper, "test_instrumentation")


if test_result.wasSuccessful():
    print "PASS"
else:
    sys.exit("FAIL")
