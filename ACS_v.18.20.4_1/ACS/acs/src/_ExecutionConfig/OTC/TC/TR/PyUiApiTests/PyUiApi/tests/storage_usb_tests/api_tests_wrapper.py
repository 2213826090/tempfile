from PyUiApi.tests.storage_usb_tests.with_api.base import *


class StorageUSBApiTestsWrapper(unittest.TestCase):
    instrumentation_args = None
    instrumentation_class_name = None
    instrumentation_method_name = None
    instrumentation_runner_name = None

    def check_instrumentation_passed(self, result):
        self.assertTrue(StorageUSBTestsWithApi.instrumentation_pass_msg in result)

    def test_instrumentation(self):
        result = ApiTestsInterface\
            .run_instrumentation(class_name=StorageUSBApiTestsWrapper.instrumentation_class_name,
                                 method_name=StorageUSBApiTestsWrapper.instrumentation_method_name,
                                 instrumentation_args=StorageUSBApiTestsWrapper.instrumentation_args,
                                 runner_name=StorageUSBApiTestsWrapper.instrumentation_runner_name)
        LOG.info(result)
        self.check_instrumentation_passed(result)
