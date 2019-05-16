import os
import sys
import argparse
import xml.etree.ElementTree as ET

def bypass_acs_args(options):
    wrap_options = []
    acs_options = []
    wrap_cmd_list = ["--domain",  
                    "--platform",
                    "--os", 
                    "--priority",
                    "--type",
                    "--areq",
                    "--ci", 
                    ]
    matched = False
    for opt in options:
        if opt.startswith("-"):
            matched = False
            if opt.split("=")[0] in wrap_cmd_list:
                matched = True

        if matched:
            wrap_options.append(opt)
        else:
            acs_options.append(opt)
    return wrap_options, acs_options

def read_args(options):
    parser = argparse.ArgumentParser(description="ACS Wrapper for launching ACS with dynamic campaign. ")

    parser.add_argument("--domain", type=str, dest="domain", required=True, help="Specify Test Domain")

    parser.add_argument("--platform", type=str, dest="platform", required=True, help="Specify Test on which platform")

    parser.add_argument("--os", type=str, dest="os", required=True, help="Specify the Android OS version")

    parser.add_argument("--priority", type=str, dest="priority", required=False, help="Specify the test case priorityA")

    parser.add_argument("--type", type=str, dest="type", required=False, help="Specify the test case type")

    parser.add_argument("--areq", type=str, dest="areq", required=False, help="Specify the test case related AREQ")

    parser.add_argument("--ci", type=str, dest="ci", required=False, help="Specify the ci ready status")

    args = parser.parse_args(options)
    return args

def select_test_cases(args, catalog=None):
    DEFAULT_CATALOG = "_ExecutionConfig/OTC/TC/TP/support/plangenerator/tc_catalog.xml"

    if not catalog:
        catalog = DEFAULT_CATALOG 

    tree = ET.parse(catalog)
    root = tree.getroot()

    testcases = root.findall("testcase")
    print "Total Test Cases in Catalog: " + str(len(testcases))
    
    selected_tcs= []
    for tc in testcases:
        selected_test_case = {}

        # Only handle ACS test cases
        if not tc.attrib["runner"] == "ACS":
            continue
        # Check the Doamin
        if not tc.attrib["domain"] == args.domain:
            continue
        
        # Check the Android OS
        os = tc.find("targets/target[@version='{os}']".format(os=args.os))
        if os is None:
            continue
        
        # Check the platform
        platform = os.find("variant[@name='{platform}']".format(platform=args.platform))
        if platform is None:
            continue
        # Check the Priority        
        if args.priority is not None:
            priority_list = str.lower(args.priority).split(",")
            if not str.lower(tc.attrib["priority"][:2]) in priority_list:
                continue
        # Check the Test Case Type
        if args.type is not None:
            type_list = str.lower(args.type).split(",")
            if not str.lower(tc.attrib["tctype"]) in type_list:
                continue
        # Check the CI Ready status
        if args.ci is not None:
            ci_list = str.lower(args.ci).split(",")
            if not str.lower(tc.attrib["ci_ready"]) in ci_list:
                continue
        # Check the AREQ
        if args.areq is not None:
            areq_list = str.lower(args.areq).split(",")
            areqs = tc.findall("requirements/requirement")
            matched = False
            for a in areqs:
                try:
                    if not str.lower(a.attrib["name"][:a.attrib["name"].index(" ")]) in areq_list:
                        continue
                    matched = True
                except:
                    continue
            if not matched:
                continue

        # Composite the node attribute
        selected_test_case["id"] = tc.attrib["id"]
        selected_test_case["template"] = tc.attrib["template"]

        selected_tcs.append(selected_test_case)
        print tc.attrib["domain"], os.attrib["version"], platform.attrib["name"], tc.attrib["priority"], tc.attrib["tctype"], tc.attrib["id"]
    return selected_tcs

def generate_campaign(testcases, template):
    folder_path, template_name = os.path.split(template)
    tree = ET.parse(template)
    root = tree.getroot()

    tcs = root.findall("TestCases/TestCase")
    tcs_node = root.find("TestCases")

    ph_index = 0
    for tc in tcs:
        if tc.attrib["Id"] == "placeholder":
            print "TestCase PlaceHolder index is:" + str(ph_index)
            tcs_node.remove(tc)
            break
        ph_index = ph_index + 1
    
    for sel_tc in testcases:
        sel_tc_node = ET.Element("TestCase", {"Id":sel_tc["id"]})
        tcs_node.insert(ph_index, sel_tc_node)
        ph_index = ph_index + 1

    tree.write(folder_path + "/dynamic_test_campaign.xml")


def main():
    wrap_options, acs_options = bypass_acs_args(sys.argv[1:])
    args = read_args(wrap_options)
    testcases = select_test_cases(args)
    # Assumption: all templates are same in the selected test cases. Since each domain has one template
    if len(testcases) == 0:
        print "No test cases are selected. Exit."
        exit(0)

    print "Total {0} test cases selected".format(len(testcases))
    template = testcases[0]["template"]
    print "Template is : " + template
    generate_campaign(testcases, "_ExecutionConfig/" + template)
    acs_cmd = "python ACS.py -c OTC/CAMPAIGN/SystemFunctional/M/MR_0/TP/dynamic_test_campaign {0}".format(" ".join(acs_options))
    print "Call ACS Command : " + acs_cmd
    os.system(acs_cmd)


if __name__ == "__main__":
    main()
