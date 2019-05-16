import argparse
import sys
import os
import shutil
from lxml import etree as ET


def get_arguments(argv):
    parser = argparse.ArgumentParser(prog="ACS Light Generator")
    parser.add_argument("-cf", dest="campaign_folder", help="Path to the Original Test Campaign folder")
    parser.add_argument("-csv", dest="csv_file", help="Path to the ET CSV file that contains the test names to be searched for, inside the Original Test Campaigns")
    args = parser.parse_args()
    return args, parser


def csv_to_list(csv_file):
    csv_list = []
    test_file = open(csv_file, "r")
    return [line.strip("\n") for line in test_file]


def mkdir_campaigns(path_to_campaigns):
    acs_light_campaign_directory = os.path.join("/".join((path_to_campaigns.split("/")[:-1])), "AFT_Light")
    if os.path.isdir(acs_light_campaign_directory):
        print "Cleaning location:", os.path.join("/".join((path_to_campaigns.split("/")[:-1])), "AFT_Light")
        shutil.rmtree(acs_light_campaign_directory)
    if os.path.isdir(path_to_campaigns):
        shutil.copytree(path_to_campaigns, acs_light_campaign_directory)
    else:
        print "Not a valid campaign directory"
        sys.exit()
    return acs_light_campaign_directory


def change_testcase_path(test_cases, test_case_prefix):
    test_case_partition = test_cases.partition("TC")
    path = "../" * test_case_prefix + test_case_partition[1] + test_case_partition[2]
    return path


def retrieve_subcampaign_test_list(test_case_list, test_case_path, test_case_prefix):
    xml_parser = ET.XMLParser(remove_blank_text=True)
    try:
        tree = ET.parse(os.path.join(test_case_path), xml_parser)
        tree_root = tree.getroot()
    except Exception as e:
        print "Can't open the campaign/subcampaing xml files:", e
        exit(-1)

    for child in tree_root:
        # obtain test cases from TestCase tag and from SubCampaign tag
        for test_cases in child.findall("*"):
            if test_cases.tag == "TestCase":
                # if TC path comes from subcampaign the TC path should be recalculated
                test_case_list.append(change_testcase_path(test_cases.get("Id"), test_case_prefix))
            if test_cases.tag == "SubCampaign":
                # calculate subcampaign file path
                count_folders_untill_root = test_cases.get("Id").split("/").count("..")
                subcampaign_relative_to_campaign = "/".join(test_cases.get("Id").split("/")[count_folders_untill_root:])
                if subcampaign_relative_to_campaign.endswith("/"):
                    subcampaign_relative_to_campaign = subcampaign_relative_to_campaign[:-1]
                root_execution = "/".join(test_case_path.split("/")[:-(count_folders_untill_root + 1)])
                subcampaign_path = os.path.join(root_execution, subcampaign_relative_to_campaign + ".xml")
                retrieve_subcampaign_test_list(test_case_list,
                                               os.path.join(root_execution, subcampaign_path),
                                               test_case_prefix)

    return test_case_list


def main(argv):
    tests_in_csv = 0
    tree = None
    test_cases = []

    # get arguments from command line
    args, parser = get_arguments(argv)
    if not args.campaign_folder or not args.csv_file:
        parser.print_help()
        sys.exit()
    print "-" * 100
    print "Using the following campaign folder:", args.campaign_folder
    print "Using the following csv file:", args.csv_file
    print "-" * 100
    csv_list = csv_to_list(args.csv_file)
    csv_list.sort()
    tests_in_csv = len(csv_list)
    xml_parser = ET.XMLParser(remove_blank_text=True)

    acs_light_campaign_directory = mkdir_campaigns(args.campaign_folder)
    for root, dirs, files in os.walk(acs_light_campaign_directory):
        for filename in files:
            if filename.endswith('.xml'):
                test_cases_list = []
                campaign_path = os.path.join(root, filename)

                # campaing path relative to execution folder is reflected in xml's tests case path
                campaign_path_prefix = campaign_path.partition("CAMPAIGN")[2].count("/")
                retrieve_subcampaign_test_list(test_cases_list, campaign_path, campaign_path_prefix)

                tree = ET.parse(os.path.join(root, filename), xml_parser)
                tree_root = tree.getroot()

                # clear campaign nodes and repopulate with matching nodes
                test_cases_content = tree_root.findall('TestCases')[0]
                test_cases_content.clear()
                children = []
                for element in test_cases_list:
                    test_name = element.split("/")[-1]
                    if test_name in csv_list or test_name.lower().startswith("prereq") or test_name.lower().startswith(
                            "postreq") or test_name.lower().startswith("init"):
                        new_node = ET.Element("TestCase", Id=element)
                        try:
                            csv_list.remove(test_name)
                        except:
                            # these are the prereq tests
                            print 'Found prerequisite: ', element.split("/")[-1]
                            pass
                    else:
                        new_node = ET.Comment(ET.tostring(ET.Element("TestCase", Id=element)))
                    test_cases_content.append(new_node)
                test_cases.extend(children)
                tree.write(os.path.join(acs_light_campaign_directory, filename), pretty_print=True)
    print "-" * 100
    for remaining_test in csv_list:
        print "This ET CSV test was not found in the Targeted Campaing Folder:", remaining_test
    print "-" * 100
    print "ACS AFT Light Campaigns successfully generated in:", acs_light_campaign_directory
    print "Number of tests in input ET csv: {} \nNumber of tests that were not found in Targeted Campaigns: {}".format(tests_in_csv, len(csv_list))
    return


if __name__ == "__main__":
    main(sys.argv)
