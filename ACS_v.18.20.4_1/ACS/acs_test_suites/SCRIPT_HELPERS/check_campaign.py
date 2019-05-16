import fnmatch
import itertools
import optparse
import os
import sys
from xml.etree import ElementTree

class CommentedTreeBuilder (ElementTree.XMLTreeBuilder):
    def __init__ (self, html = 0, target = None):
        ElementTree.XMLTreeBuilder.__init__(self, html, target)
        self._parser.CommentHandler = self.handle_comment

    def handle_comment (self, data):
        self._target.start(ElementTree.Comment, {})
        self._target.data(data)
        self._target.end(ElementTree.Comment)

def find_file(root_dir, filename):
    """
    Get the location of a file from a root directory

    :param root_dir: where to start the seach
    :type  root_dir: str.

    :param filename: file to locate
    :type  filename: str.

    :return: The full path to the filename/dirname we are looking for
    :rtype: String

    """
    result = []
    for root, _, filenames in os.walk(root_dir):
        for file_name in fnmatch.filter(filenames, filename):
            result.append(os.path.join(root, file_name))

    return result

def check_campaign(campaign_path, auto_correct):
    correct = False
    print "Parsing %s..." % campaign_path,

    try:
        with open(campaign_path, "r") as f:
            tree = ElementTree.parse(f, parser = CommentedTreeBuilder())
    except:
        print "ERROR: Parsing error, please check the campaign !"
        return False

    campaign_folder = os.path.dirname(campaign_path)
    try:
        parent = tree.iter("TestCases").next()
    except:
        print "WARNING: Only campaigns should be located in CAMPAIGNS folder !"
        return True

    files_error = []
    tc_to_be_removed = []

    for node in itertools.chain(tree.iter("TestCase"), tree.iter("TC")):
        if node.tag == "TestCase":
            tc_file = node.attrib["Id"] + ".xml"
        else:
            tc_file = node.attrib["name"] + ".xml"

        tc_path = os.path.join(campaign_folder, tc_file)
        tc_path = tc_path.replace("\\", os.sep)
        tc_path = tc_path.replace("/", os.sep)
        tc_path = os.path.abspath(tc_path)

        if not os.path.isfile(tc_path):
            file_loc = find_file(os.path.abspath(os.path.join(campaign_folder, "..")),
                                 os.path.split(tc_file)[1])
            if file_loc:
                files_error.append("\tERROR: %s does not exist, files available:" % tc_path)
                for f in file_loc:
                    files_error.append("\t%s" % f)

                if auto_correct:
                    if len(file_loc) == 1:
                        rel_path = os.path.relpath(file_loc[0],
                                                   campaign_folder)
                        rel_path = os.path.splitext(rel_path)[0]
                        if node.tag == "TestCase":
                            node.attrib["Id"] = rel_path
                        else:
                            node.attrib["name"] = rel_path
                    else:
                        print "Cannot correct => two or more TC possibilities!"
            else:
                if auto_correct:
                    # Remove it
                    tc_to_be_removed.append(node)
                    files_error.append("\tERROR:\t%s does not exist => REMOVED !" % tc_path)
                else:
                    files_error.append("\tERROR:\t%s does not exist" % tc_path)


    if files_error:
        print ""
        for f in files_error:
            print f

        if auto_correct:
            for tc in tc_to_be_removed:
                parent.remove(tc)
            tree.write(campaign_path)
    else:
        correct = True
        print "OK !"

    return correct

def retrieve_campaign_list(folder):
    campaigns = []
    for dir_name, sub_dirs, files in os.walk(folder):
        # if os.path.basename(dir_name) == "CAMPAIGN":
        if "CAMPAIGN" in dir_name:
            files = [os.path.join(dir_name, x) for x in files if os.path.splitext(x)[1] == ".xml"]
            campaigns.extend(files)

    return campaigns

def main(auto_correct, folder):
    if folder:
        os.path.abspath(os.path.join(os.getcwd(), folder))
    else:
        folder = os.getcwd()
    campaigns = retrieve_campaign_list(folder)
    error = 0
    for campaign in campaigns:
        if not check_campaign(campaign, auto_correct):
            error += 1

    print "\nFiles:\t%d" % len(campaigns)
    print "Errors:\t%d" % error

def conf_parse():
    parser = optparse.OptionParser()
    parser.add_option("-c", action = "store_true", default = False,
                      help = "Try to fix campaign files.")
    parser.add_option("-f", action = "store", default = None,
                      metavar = "FOLDER",
                      help = "Start the process on folder FOLDER")
    return parser.parse_args()[0]

if __name__ == "__main__":
    arg = conf_parse()
    sys.exit(main(arg.c, arg.f))
