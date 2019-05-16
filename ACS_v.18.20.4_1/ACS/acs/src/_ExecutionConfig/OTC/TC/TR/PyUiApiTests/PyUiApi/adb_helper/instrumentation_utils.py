from PyUiApi.common.uiautomator_utils import *
from string import Template
import subprocess

api_tests_logcat = '''logcat -d | grep API'''


class InstrumentationInterface(object):
    instrumentation_one_test_pass_output = "OK (1 test)"
    shell_cmd_template = Template('''
            am instrument -e class $classarg $extraargs -w $runnerarg
            ''')

    @staticmethod
    def run_instrumentation(class_arg, extra_args, runner_arg):
        shell_cmd = InstrumentationInterface\
            .shell_cmd_template.substitute(classarg=class_arg, extraargs=extra_args,
                                           runnerarg=runner_arg)
        LOG.info("running instrumentation command: " + shell_cmd)
        # output = subprocess.check_output(shell_cmd, shell=True)
        output = AdbUtils.run_adb_cmd(shell_cmd)
        return output

    @staticmethod
    def was_instrumentation_test_successful(test_output):
        return InstrumentationInterface.instrumentation_one_test_pass_output in test_output


class ApiTestsInterface(InstrumentationInterface):
    class_template = Template('com.intel.test.apitests.tests.$classname#$method')
    args_template = Template('-e args "$args"')
    extra_args_template = Template('-e argsExtra "$args"')
    runner_template = Template('com.intel.test.apitests/com.intel.test.apitests.runners.$runner')

    @staticmethod
    def run_instrumentation(class_name="SystemStorageUSBTestsDriver", method_name="testCreateFile",
                            instrumentation_args=None, instrumentation_extra_args=None,
                            runner_name="GenericArgumentPassingTestRunner"):
        class_string = ApiTestsInterface\
            .class_template.substitute(classname=class_name, method=method_name)
        args_string = ""  # pass this to base class if there are no extra args
        if instrumentation_args is not None:
            args_string = ApiTestsInterface\
                .args_template.substitute(args=instrumentation_args)
        if instrumentation_extra_args is not None:
            args_string += " " + ApiTestsInterface\
                .extra_args_template.substitute(args=instrumentation_extra_args)
        runner_string = ApiTestsInterface\
            .runner_template.substitute(runner=runner_name)
        output = InstrumentationInterface.run_instrumentation(class_string, args_string, runner_string)
        return output


class SystemApiTestsInterface(InstrumentationInterface):
    class_template = Template('com.intel.test.systemapitests.tests.$classname#$method')
    args_template = Template('-e args "$args"')
    runner_template = Template('com.intel.test.systemapitests/com.intel.test.systemapitests.runners.$runner')

    @staticmethod
    def run_instrumentation(class_name="FileSystemTestsDriver", method_name="testCreateFile",
                            instrumentation_args=None, runner_name="GenericArgumentPassingTestRunner"):
        class_string = SystemApiTestsInterface\
            .class_template.substitute(classname=class_name, method=method_name)
        args_string = ""  # pass this to base class if there are no extra args
        if instrumentation_args is not None:
            args_string = SystemApiTestsInterface\
                .args_template.substitute(args=instrumentation_args)
        runner_string = SystemApiTestsInterface\
            .runner_template.substitute(runner=runner_name)
        output = InstrumentationInterface.run_instrumentation(class_string, args_string, runner_string)
        return output


class CheckPermissionsInterface(InstrumentationInterface):
    class_template = Template('com.intel.test.checkpermissions.$classname#$method')
    runner_template = Template('com.intel.test.checkpermissions/com.intel.test.checkpermissions.$runner')
    args_template = Template('-e args "$args"')

    @staticmethod
    def run_instrumentation(class_name="CheckPermissionTest", method_name="testPermission",
                            instrumentation_args=None, runner_name="GenericArgumentPassingTestRunner"):
        class_string = CheckPermissionsInterface\
            .class_template.substitute(classname=class_name, method=method_name)
        args_string = ""  # pass this to base class if there are no extra args
        if instrumentation_args is not None:
            args_string = CheckPermissionsInterface\
                .args_template.substitute(args=instrumentation_args)
        runner_string = CheckPermissionsInterface\
            .runner_template.substitute(runner=runner_name)
        output = InstrumentationInterface.run_instrumentation(class_string, args_string, runner_string)
        return output


class JavaUiAutomatorUtils(InstrumentationInterface):
    class_template = Template('com.intel.uitests.$subpackage.$classname#$method')
    args_template = Template('-e args "$args"')
    runner_template = Template('com.intel.uitests.test/com.intel.uitests.runner.$runner')


class ApiTestsGenericExtraArgs(object):
    def __init__(self, **initial_args):
        self.args = {}
        for key, value in initial_args.iteritems():
            self.args[key] = value

    def get_args_string(self, **kwargs):
        result = ""
        for key, value in self.args.iteritems():
            result += str(key) + ":" + str(value) + " "
        for key, value in kwargs.iteritems():
            self.args[key] = value
            result += str(key) + ":" + str(value) + " "
        return result[:-1]

if __name__ == "__main__":
    ApiTestsInterface\
        .run_instrumentation(instrumentation_args="createDir:EXTERNAL_SDCARD"
                             "/Android/data/com.intel.test.apitests/cache/test/ nrOfFiles:10 fileSizeKB:2")
