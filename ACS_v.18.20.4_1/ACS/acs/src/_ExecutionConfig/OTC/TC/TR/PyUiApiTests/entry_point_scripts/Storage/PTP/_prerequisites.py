from os import sys, path
relevant_package_dir = "PyUiApiTests"
for p in sys.path:
    if relevant_package_dir in p:
        py_ui_api_tests_path = p[:p.index(relevant_package_dir) + len(relevant_package_dir)]
        sys.path.append(path.normpath(py_ui_api_tests_path))
        break
