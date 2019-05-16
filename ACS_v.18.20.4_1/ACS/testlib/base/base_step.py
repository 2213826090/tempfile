#/usr/bin/env python

########################################################################
#
# @filename:    base_step.py
# @description: Basic test step
# @author:      alexandrux.n.branciog@intel.com
#
########################################################################

import os
import sys
from testlib.base.base_utils import TimeoutError
from testlib.utils import logger

class BlockingError(Exception):
    """Error for blocking steps"""
    pass

class FailedError(Exception):
    """Error for critical steps"""
    pass

class Resolution():
    """Holds test step resolutions"""
    PASS = "[PASSED]"
    FAIL = "[FAILED]"
    BLOCKED = "[BLOCKED]"
    TIMEOUT = "[TIMEDOUT]"
    ERROR = "[ERROR]"

class step(object):
    """
    Generic class for a test step
    Should be extended by each test step

    Members:

    blocking    -- step is blocking or not. Defaults to True
    critical    -- test step importance. Defaults to True
                   if True Test will stop at this step
    passm       -- message printed to STDOUT if step is Passed
    errorm      -- message printed to STDOUT in case of error
    step_data   -- used to pass data to next steps
                   will be returned by execute_step method
    resolution  -- one of PASS, FAILED, BLOCKED, TIMEOUT
    verbose     -- verbose mode for script issues investigation.
                   Defaults to False
    """
    blocking = False
    critical = True
    step_data = None
    resolution = None
    verbose = False
    logger = None
    with_flush = False

    def __init__(self, debug_info=True, **kwargs):
        super(step, self).__init__()
        self.passm = self.__class__.__name__ + " - [PASSED]"
        self.errorm = self.__class__.__name__ + " - [FAILED]"
        self.debug_info = debug_info
        if kwargs.has_key('blocking'):
            self.blocking = kwargs['blocking']
        if kwargs.has_key('critical'):
            self.critical = kwargs['critical']
        if kwargs.has_key('print_error'):
            self.errorm = self.errorm + " -> " + kwargs['print_error']
        if kwargs.has_key('print_pass'):
            self.passm = kwargs['print_pass']
        if kwargs.has_key('verbose'):
            self.verbose = kwargs['verbose']
        if kwargs.has_key('with_flush'):
            self.with_flush = kwargs['with_flush']
        if kwargs.has_key('logger'):
            self.logger = kwargs['logger']
        else:
            if "LOG_PATH" in os.environ:
                self.testlib_log_path = os.environ["LOG_PATH"]
            else:
                import testlib
                self.testlib_log_path = os.path.dirname(testlib.__file__)  + "/logs/"
            self.logger = logger.testlib_log(log_path = self.testlib_log_path, log_name = "testlib_default")

    def __call__(self, **kwargs):
        """will execute step"""
        return self.execute_step()

    def execute_step(self):
        """
        method to be called for each step
        check is optional
        """
        try:
            self.do()
            self.check()
        except TimeoutError, e:
            self.resolution = Resolution.TIMEOUT
            raise e
        except:
            if self.debug_info:
                self.log_error_info()
            raise
        return self.step_data

    def do(self):
        """must overwrite this method to implement an action"""
        raise NotImplementedError('Must implement "do" method for each step')

    def check(self):
        """
        it does the verification defined in check_condition
        on verification failure -->
         - if blocking it will raise a BlockingError
         - if failed it will:
            - raise a FailedError for critical steps
            - print an error message for non critical steps
        """
        check_result = self.check_condition()
        if check_result is not None:
            if check_result:
                self.resolution = Resolution.PASS
                if self.logger:
                    self.logger.info(self.passm)
                else:
                    print self.passm
            else:
                if self.blocking:
                    self.resolution = Resolution.BLOCKED
                    if self.logger:
                        self.logger.error(self.errorm)
                    raise BlockingError(self.errorm)
                else:
                    self.resolution = Resolution.FAIL
                    if self.critical:
                        if self.logger:
                            self.logger.error(self.errorm)
                        raise FailedError(self.errorm)
                    else:
                        if self.debug_info:
                            self.log_error_info()
                        if self.logger:
                            self.logger.error(self.errorm)
                        else:
                            print self.errorm
            if self.with_flush:
                sys.stdout.flush()
        else:
            self.resolution = Resolution.PASS
            self.passm = "[ Unchecked ] " + self.passm
            if self.logger:
                self.logger.info(self.passm)
            else:
                print self.passm


    def log_error_info(self):
        """
        Overwrite this method to save test artifacts in case of failed tests.
        """
        return None

    def check_condition(self):
        """
        overwrite this method to return test step specific verification
        should return True or False
        if not overwritten check will not be performed
        """
        return None

    def set_passm(self, pass_string):
        """
        Helps you customize pass message
        Example:
            step.set_passm("OK")
        """
        self.passm = self.__class__.__name__ +\
                    " {0} - [PASSED]".format(pass_string)

    def set_errorm(self, particular_string, error_string):
        """
        Helps you customize error message
        Example:
            step.set_errorm("OK", "Pressing OK button failed")
        """
        self.errorm = self.__class__.__name__ +\
                        " {0} - [FAILED] -> {1}".format(
                        particular_string, error_string)
