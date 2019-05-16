from _prerequisites import *
from PyUiApi.tests.telephony.phonebook.telephony_phonebook_tests import *

global test_outcome
test_result = SingleMethodRunner.run_single_test(TelephonyPhoneBookTests, "test_import_contacts_from_sim_flight_mode_on")

if test_result.wasSuccessful():
    print "PASS"
    test_outcome = True
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
    test_outcome = False
