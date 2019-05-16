import unittest


class SingleMethodRunner:
    @staticmethod
    def run_single_test(test_case_class, method_name):
        suite = unittest.TestSuite()
        suite.addTest(test_case_class(method_name))
        runner = unittest.TextTestRunner()
        return runner.run(suite)