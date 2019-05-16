import pkgutil

# This is add testlib path search for libraries availabe in testlib folder
# under acs_test_suites/OTC/TC/TP for interoperability between workflow
__path__ = pkgutil.extend_path(__path__, __name__)