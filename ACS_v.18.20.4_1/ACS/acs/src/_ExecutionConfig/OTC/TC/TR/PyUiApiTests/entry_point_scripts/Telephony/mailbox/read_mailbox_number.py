from _prerequisites import *
from PyUiApi.tests.telephony.mailbox.mailbox import *

global test_outcome
test_result = SingleMethodRunner.run_single_test(MailboxTests, "test_read_mailbox_number")

if test_result.wasSuccessful():
    print "PASS"
    test_outcome = True
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
    test_outcome = False
