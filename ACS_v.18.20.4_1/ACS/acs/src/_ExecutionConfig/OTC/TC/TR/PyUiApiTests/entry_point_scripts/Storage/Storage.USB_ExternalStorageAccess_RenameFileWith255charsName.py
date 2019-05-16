from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.api_tests_wrapper import *
from PyUiApi.tests.storage_usb_tests.generic import *
from PyUiApi.common.environment_utils import *
import os


test_result = True

#prereq - format to portable
test_result1 = SingleMethodRunner.run_single_test(StorageUSBTests, "test_format_as_portable")
test_result = test_result and test_result1

#instrumentation used
StorageUSBApiTestsWrapper.instrumentation_runner_name = "GenericArgumentPassingTestRunner"
StorageUSBApiTestsWrapper.instrumentation_method_name = "testRenameFile"
StorageUSBApiTestsWrapper.instrumentation_class_name = "SystemStorageUSBTestsDriver"
test_file_name = reduce(lambda x, y: x+y, ["a" for i in range(250)])
partial_args = " fileName:old_file fileSizeKB:2 newFileName:" + test_file_name + ".txt notCreateTest:true"

#rename on emulated outside Android
StorageUSBApiTestsWrapper.instrumentation_args = "createDir:" + EnvironmentUtils.get_emulated_storage_path() + partial_args
test_result2 = SingleMethodRunner.run_single_test(StorageUSBApiTestsWrapper, "test_instrumentation")
test_result = test_result and test_result2.wasSuccessful()

#rename on emulated inside Android
StorageUSBApiTestsWrapper.instrumentation_args = "createDir:" + EnvironmentUtils.get_emulated_storage_path() + Environment.api_tests_data_cache_dir_sdcard + partial_args
test_result3 = SingleMethodRunner.run_single_test(StorageUSBApiTestsWrapper, "test_instrumentation")
test_result = test_result and test_result3.wasSuccessful()

#rename on Sdcard outside Android
StorageUSBApiTestsWrapper.instrumentation_args = "createDir:" + EnvironmentUtils.get_sd_card_path() + partial_args
test_result4 = SingleMethodRunner.run_single_test(StorageUSBApiTestsWrapper, "test_instrumentation")
test_result = test_result and not(test_result4.wasSuccessful())

#rename on Sdcard inside Android
StorageUSBApiTestsWrapper.instrumentation_args = "createDir:" + EnvironmentUtils.get_sd_card_path() + Environment.api_tests_data_cache_dir_sdcard + partial_args
test_result5 = SingleMethodRunner.run_single_test(StorageUSBApiTestsWrapper, "test_instrumentation")
test_result = test_result and test_result5.wasSuccessful()


if test_result:
    print "PASS"
else:
    #TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")




