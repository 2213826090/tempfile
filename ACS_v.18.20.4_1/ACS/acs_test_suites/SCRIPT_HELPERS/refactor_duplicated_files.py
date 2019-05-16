#!/usr/bin/env python

"""

Usage : python refactor_duplicated_files.py [options] <folder_to_analyze>

Command line options:
  h - show help on usage
  a - list all XML files, with their properties (type, md5, etc.)
  r - change TC's and SubCampaign's paths in campaign files (convert them to relative path)
  o - rename all duplicated TC and campaign files (adding suffix __X, where X is incremented for each instance of the file).
      modify campaigns content, to reflect those changes in filenames.
  d - remove all duplicated TC and campaign files, based on 'duplicated_filter.yaml'.
      modify campaigns content, to reflect those changes in filenames.
  t - remove test cases, based on 'tc_list.yaml'.
      modify campaigns content, to reflect those changes.
  u - modify campaigns, removing test cases that does not exist.

optional flags:
  m - use this option combined with -d, -r, -o, or -u.
      This will apply modifications on files, otherwise only csv will be generated

If no folder is specified, current directory is used.
"""

import getopt
import os
import os.path
import sys
import shutil
import hashlib
import yaml
from tempfile import mkstemp
from lxml import etree


PARSING_WILLFUL_ERRORS = ["SI/BBot_EAT_PACT/TC/FWK/NOVAR/CAMPAIGN_CFG/scripts/CORRUPTED_TC.xml",
                         "SI/BBot_EAT_PACT/TC/FWK/NOVAR/CAMPAIGN_CFG/scripts/EMPTY_TC.xml",
                         "SI/BBot_EAT_PACT/TC/FWK/NOVAR/CAMPAIGN_CFG/scripts/SI_ACS_NOVAR_FWK_CAMPAIGN_CFG_019_Campaign.xml",
                         "SI/BBot_EAT_PACT/TC/FWK/NOVAR/MULTI_CAMPAIGN/scripts/SI_ACS_NOVAR_FWK_MULTI_CAMPAIGN_021_Campaign.xml",
                         "SI/BBot_EAT_PACT/TC/FWK/NOVAR/TESTSTEP/scripts/badnodesecondaryteststeptatalog.xml",
                         "SI/BBot_EAT_PACT/TC/FWK/NOVAR/TESTSTEP/scripts/badnodesequence.xml",
                         "SI/BBot_EAT_PACT/TC/FWK/NOVAR/TESTSTEP/scripts/badnodeteststepset.xml",
                         "SI/BBot_EAT_PACT/TC/FWK/NOVAR/TESTSTEP/scripts/emptyfile.xml",
                         "SI/BBot_EAT_PACT/TC/FWK/NOVAR/TESTSTEP/scripts/malformedfile.xml"]

XML_FILE_TYPES = {'Campaign': 'Campaign',
                  'CampaignDescription': 'CampaignDescription',
                  'Campaigns': 'CampaignDescription',
                  'TestCases': 'CampaignDescription',
                  'TestCase': 'TestCase',
                  'UI_SEQ': 'UiSequence',
                  'operationsets': 'UiSequence',
                  'TestSteps': 'TestSteps',
                  'Include': 'TestSteps',
                  'BenchConfig': 'BenchConfig',
                  'Sequence': 'EquipmentConfig',
                  'PowerProject': 'EquipmentConfig',
                  'SLAVE_CONF': 'SlaveConfig'}


class HelpException(Exception):
    pass


def removeLineInTxtFile(filePath, pattern):
    def rmLine(aline, pattern, new_file, subst):
        # copy line only if it does not match pattern
        if aline.find(pattern) == -1:
            new_file.write(aline)

    _changeTxtInFile(filePath, rmLine, pattern, None)


def replaceTxtInFile(filePath, pattern, subst):
    def changeLine(aline, pattern, new_file, subst):
        new_file.write(aline.replace(pattern, subst))

    _changeTxtInFile(filePath, changeLine, pattern, subst)


def _changeTxtInFile(filePath, action, pattern, subst):
    #Create temp file
    fh, absPath = mkstemp()
    new_file = open(absPath, 'wb')
    old_file = open(filePath, 'r')
    for aline in old_file:
        action(aline, pattern, new_file, subst)

    #close temp file
    new_file.close()
    os.close(fh)
    old_file.close()
    #Remove original file
    os.remove(filePath)
    #Move new file
    shutil.move(absPath, filePath)


def getXmlFileType(rootNode):
    filetype = XML_FILE_TYPES.get(rootNode.tag)
    if not filetype:
        filetype = "Unknown"
    return filetype


def shouldBeIgnored(path, ignoreList):
    relPath = path[len(ROOT_DIR)+1:].replace('\\', '/')  # +1 is for the last '/'
    return relPath in ignoreList


def listAllXmlFiles(allfiles, dirname, files):
    for f in files:
        path = os.path.join(dirname, f)
        if os.path.isfile(path) and f.lower().endswith('.xml'):

            # parse xml file to determine its type
            try:
                parsedFile = etree.parse(path)
                filetype = getXmlFileType(parsedFile.getroot())
            except Exception as ex:
                if shouldBeIgnored(path, PARSING_WILLFUL_ERRORS):
                    filetype = "WillfulCorrupt"
                else:
                    sys.stderr.write("Error while parsing %s : %s\n" % (path, ex.message))
                    filetype = "Corrupted"

            # compute checksum for TC and campaigns files
            md5Sum = ""
            if filetype in ['Campaign', 'TestCase', 'CampaignDescription']:
                md5 = hashlib.md5()
                md5.update(open(path, "rb").read())
                md5Sum = md5.hexdigest()

            # create key from filename
            filekey = os.path.splitext(f.lower())[0]

            # store file data
            if filekey in allfiles:
                if filetype in allfiles[filekey]:
                    allfiles[filekey][filetype].append((f, dirname, "", md5Sum))
                else:
                    allfiles[filekey][filetype] = [(f, dirname, "", md5Sum)]
            else:
                allfiles[filekey] = {filetype: [(f, dirname, "", md5Sum)]}


def addNewFilenameToTuple(files, newName=None):
    for i, f in enumerate(files):
        filename = f[0]
        folder = f[1]
        newfilename = newName if newName else "%s__%d.xml" % (f[0][:-4], i+1)
        md5 = f[3]
        files[i] = (filename, folder, newfilename, md5)


def computeDupNewFilename(duplicates):
    for filekey, filedata in duplicates.iteritems():
        for filetype, files in filedata.iteritems():
            # change only TC and campaigns filename
            if filetype == 'TestCase':
                addNewFilenameToTuple(files)

            elif filetype == 'Campaign':
                if 'TestCase' in filedata:
                    if len(filedata) == 2:
                        # only campaign and TC with same name => do nothing : TC will be renamed
                        pass
                    else:
                        # TC, Campaign, and other filetype with same name ... What do we do ?
                        sys.stderr.write("Error while generating new filename for %s: multiple types not handled\n" % filekey)
                else:
                    print "Warning: campaign %s will be renamed " % filekey
                    addNewFilenameToTuple(files)


def filterOnCampaigns(allFiles):
    allCampaigns = {}

    # loop over allFiles, keeping only campaigns
    for filekey, filedata in allFiles.iteritems():
        campaigns = filedata.get('Campaign')
        campaignDescs = filedata.get('CampaignDescription')

        # create entry in allCampaign if it does not already exist
        if (campaigns or campaignDescs) and not allCampaigns.get(filekey):
            allCampaigns[filekey] = {}

        # add campaign and campaignDescription if any
        if campaigns:
            allCampaigns[filekey]['Campaign'] = campaigns
        if campaignDescs:
            allCampaigns[filekey]['CampaignDescription'] = campaignDescs

    return allCampaigns


def filterFromFileList(allFiles, yamlFile):
    filtered = {}

    with open(yamlFile) as f:
        fullPathList = yaml.load(f)

        # create filter dict with the right format
        filters = {}
        for fullPath in fullPathList:
            filename = os.path.basename(fullPath)
            folder = os.path.dirname(fullPath)
            filekey = os.path.splitext(filename.lower())[0]
            if not filters.get(filekey):
                filters[filekey] = []
            filters[filekey].append(folder.lower())

        # loop over allFiles, keeping only files declared in filters
        for filekey, filedata in allFiles.iteritems():
            if filekey in filters.keys():
                for filetype, files in filedata.iteritems():
                    for fil in files:
                        folder = fil[1]
                        if folder.lower() in filters[filekey]:
                            # we have to keep this file
                            if not filtered.get(filekey):
                                filtered[filekey] = {}
                            if not filtered[filekey].get(filetype):
                                filtered[filekey][filetype] = []
                            filename = fil[0]
                            newFileName = "toRemove"
                            md5 = fil[2]
                            filtered[filekey][filetype].append((filename, folder, newFileName, md5))

    return filtered


def filterDuplicatesFromYaml(duplicates, yamlFile):
    filteredDuplicates = {}

    with open(yamlFile) as f:
        filters = yaml.load(f)

        for filekey, filedata in duplicates.iteritems():

            if filekey in filters.keys():
                filteredDuplicates[filekey] = filedata
                newFileName = filters[filekey]

                for filetype, files in filedata.iteritems():
                    # change only TC filename
                    if filetype == 'TestCase':
                        addNewFilenameToTuple(files, newFileName)

                    else:
                        # TC, Campaign, and other filetype with same name ... What do we do ?
                        sys.stderr.write(
                            "Error while generating new filename for %s: multiple types not handled\n" % filekey)

    return filteredDuplicates


def exportDuplicatesYaml(duplicates):
    exportpath = os.path.join(os.path.dirname(__file__), "duplicates.yaml")
    with open(exportpath, "w") as f:
        f.write(yaml.dump(duplicates, default_flow_style=True))

    print "Exported to %s" % exportpath
    print 'Number of duplicated files : %d' % len(duplicates)


def exportDuplicatesCsv(duplicates, filename):
    exportpath = os.path.join(os.path.dirname(__file__), filename)
    i = 0
    with open(exportpath, "w") as f:
        f.write("Filename;Md5;Type;New filename;Folder;\n")

        for filekey, filedata in duplicates.iteritems():
            for filetype, files in filedata.iteritems():
                for filename, folder, newfilename, md5 in files:
                    i += 1
                    curPath = os.path.normpath(folder)
                    f.write("%s;%s;%s;%s;%s;\n" % (filename, md5, filetype, newfilename, curPath))

    print "Exported to %s" % exportpath
    print '%d duplicated files, in %d groups' % (i, len(duplicates))


def doRenameDuplicates(duplicates):
    if DO_MODIFY_FILES:
        renameCount = 0
        for filekey, filedata in duplicates.iteritems():
            for filetype, files in filedata.iteritems():
                for filename, folder, newfilename, md5 in files:
                    if newfilename:
                        path = os.path.normpath(folder)
                        oldname = os.path.join(path, filename)
                        newname = os.path.join(path, newfilename)
                        os.rename(oldname, newname)
                        renameCount += 1
        print "Renamed %d duplicates" % renameCount


def doRemoveDuplicates(duplicates):
    if DO_MODIFY_FILES:
        removeCount = 0
        for filekey, filedata in duplicates.iteritems():
            for filetype, files in filedata.iteritems():
                for filename, folder, newfilename, md5 in files:
                    if newfilename:
                        # remove only files that are different from their new name
                        if os.path.join(folder, filename) != newfilename:
                            path = os.path.normpath(os.path.join(folder, filename))
                            os.unlink(path)
                            removeCount += 1
        print "Removed %d files" % removeCount


def computeCampaignTcsRemoval(parsedFile, nodeName, attrName, nodeType, execConfigDir, campaignDir, filesToRemove):
    refacto = {}

    # get all requested nodes
    nodes = parsedFile.getroot().xpath('//%s' % nodeName)
    for node in nodes:
        # get path from xml
        path = node.get(attrName, "")

        # search filename in filesToRemove dict
        origFolder = os.path.dirname(path)
        origName = os.path.basename(path)
        ftr = filesToRemove.get(origName.lower())
        if ftr:
            # check that there are filesToRemove for this kind of node (depending node type and file type)
            files = ftr.get(nodeType)
            if files:
                # check that abs paths match
                absPathFromCampaign = os.path.abspath(os.path.join(campaignDir, "%s.xml" % path))
                for filename, folder, newfilename, md5 in files:
                    absPathFromFtr = os.path.abspath(os.path.join(folder, filename))
                    # if path in campaign matches a path in ftr => remove TC
                    if absPathFromCampaign == absPathFromFtr:
                        refacto[path] = "toRemove"
                        break

    return refacto, set()


def computeCampaignDupsRemoval_suffix(parsedFile, nodeName, attrName, nodeType, execConfigDir, campaignDir, duplicates):
    refacto = {}

    # get all requested nodes
    nodes = parsedFile.getroot().xpath('//%s' % nodeName)
    for node in nodes:
        # get path from xml
        path = node.get(attrName, "")

        # search filename in duplicates dict
        origFolder = os.path.dirname(path)
        origName = os.path.basename(path)
        dup = duplicates.get(origName.lower())
        if dup:
            # check that there are duplicates for this kind of node (depending node type and file type)
            files = dup.get(nodeType)
            if files:
                # check that abs paths match
                absPathFromCampaign = os.path.abspath(os.path.join(campaignDir, "%s.xml" % path))
                for filename, folder, newfilename, md5 in files:
                    absPathFromDup = os.path.abspath(os.path.join(folder, filename))
                    # if path in campaign matches a path in dup => replace filename in campaign path
                    if absPathFromCampaign == absPathFromDup:
                        # get new filename without extension
                        newName = os.path.basename(os.path.splitext(newfilename)[0])
                        refacto[path] = os.path.join(origFolder, newName).replace('\\', '/')
                        break

    return refacto, set()


def computeCampaignDupsRemoval_yaml(parsedFile, nodeName, attrName, nodeType, execConfigDir, campaignDir, duplicates):
    refacto = {}

    # get all requested nodes
    nodes = parsedFile.getroot().xpath('//%s' % nodeName)
    for node in nodes:
        # get path from xml
        path = node.get(attrName, "")

        # search filename in duplicates dict
        origFolder = os.path.dirname(path)
        origName = os.path.basename(path)
        dup = duplicates.get(origName.lower())
        if dup:
            # check that there are duplicates for this kind of node (depending node type and file type)
            files = dup.get(nodeType)
            if files:
                # check that abs paths match
                absPathFromCampaign = os.path.abspath(os.path.join(campaignDir, "%s.xml" % path))
                for filename, folder, newfilename, md5 in files:
                    absPathFromDup = os.path.abspath(os.path.join(folder, filename))
                    # if path in campaign matches a path in dup => replace filename in campaign path
                    if absPathFromCampaign == absPathFromDup:
                        # no need to change if new path is the same as old one
                        if os.path.abspath(newfilename) != absPathFromCampaign:
                            # get new filename without extension
                            newName = os.path.splitext(os.path.relpath(newfilename, execConfigDir))[0]
                            refacto[path] = newName.replace('\\', '/')
                        break

    return refacto, set()


def computeCampaignRelativePaths(parsedFile, nodeName, attrName, nodeType, execConfigDir, campaignDir, filesSubset):
    refacto = {}
    errors = set()

    # get all requested nodes
    nodes = parsedFile.getroot().xpath('//%s' % nodeName)
    for node in nodes:
        # get path from xml
        path = node.get(attrName, "")
        if not path:
            sys.stderr.write("Unable to analyze %s: errors in XML file\n" % parsedFile.docinfo.URL)
        elif not path.startswith('.'):
            absPath = os.path.abspath(os.path.join(execConfigDir, path))
            if not os.path.isfile(absPath + '.xml'):
                # check if path is implicitly relative (eg: ./ is implicit)
                absPath = os.path.abspath(os.path.join(campaignDir, path))
                if os.path.isfile(absPath + '.xml'):
                    relPath = "./%s" % path
                    refacto[path] = relPath.replace('\\', '/')
                else:
                    # file cannot be found => add it to error list
                    errors.add(path)

    return refacto, errors


def computeCampaignInexistingTcs(parsedFile, nodeName, attrName, nodeType, execConfigDir, campaignDir, filesSubset):
    refacto = {}
    errors = set()

    # get all requested nodes
    nodes = parsedFile.getroot().xpath('//%s' % nodeName)
    for node in nodes:
        # get path from xml
        path = node.get(attrName, "")
        if not path:
            sys.stderr.write("Unable to analyze %s: errors in XML file\n" % parsedFile.docinfo.URL)
        else:
            if path.startswith('.'):
                absPath = os.path.abspath(os.path.join(campaignDir, path))
            else:
                absPath = os.path.abspath(os.path.join(execConfigDir, path))

            if not absPath.lower().endswith('.xml'):
                absPath += '.xml'

            if not os.path.isfile(absPath):
                refacto[path] = "toRemove"

    return refacto, errors


def computePathsForAllCampaigns(allFiles, pathOperationFunction, filesSubset=None, execConfigDir=""):

    # dict containing campaign data needed to refactor xml content
    # key is campaign abs path, entry is a tuple containing 1 dict, 1 set, and the filetype :
    # dict is associating TC current path with TC new path,
    # set contains TC file that does not exist,
    campaignsRefacto = {}

    # iterate over all files
    for filekey, filedata in allFiles.iteritems():

        # depending file type, analyze different nodes
        if filedata.get('Campaign') is not None:
            # campaign
            filetype = 'Campaign'
            nodesList = [('TestCase', 'Id', 'TestCase'), ('SubCampaign', 'Id', 'Campaign')]
        elif filedata.get('CampaignDescription') is not None:
            # campaign description
            # hopefully, a filename can't be the same for CampaignDescription and Campaign
            filetype = 'CampaignDescription'
            nodesList = [('TC', 'name', 'TestCase')]
        else:
            # ignore files that are not campaigns
            continue

        files = filedata[filetype]
        for filename, folder, newfilename, md5 in files:
            path = os.path.join(folder, filename)
            campaignDir = os.path.dirname(path)

            # create entry for this path in campaign refacto dict
            campaignsRefacto[path] = ({}, set(), filetype)

            # parse xml file
            parsedFile = etree.parse(path)

            # iterate over the different kind of nodes containing a path
            for nodeName, attrName, nodeType in nodesList:
                # execute paths operation for the current file
                refacto, errors = pathOperationFunction(parsedFile, nodeName, attrName, nodeType, execConfigDir, campaignDir, filesSubset)
                campaignsRefacto[path][0].update(refacto)
                campaignsRefacto[path][1].update(errors)

    return campaignsRefacto


def exportCampaignRefactoCsv(campaignRefacto, filename):
    exportpath = os.path.join(os.path.dirname(__file__), filename)
    i = 0
    e = 0
    with open(exportpath, "w") as f:
        f.write("Filetype;Campaign;Current path;New path;Errors;\n")

        for campaign, tcs in campaignRefacto.iteritems():
            for oldTc, newTc in tcs[0].iteritems():
                i += 1
                f.write("%s;%s;%s;%s;;\n" % (tcs[2], campaign, oldTc, newTc))
            for err in tcs[1]:
                e += 1
                f.write("%s;%s;;;%s;\n" % (tcs[2], campaign, err))

    print "Exported to %s" % exportpath
    print 'Number of analyzed campaigns: %d   Changes: %d    Errors: %d' % (len(campaignRefacto), i, e)


def doRefactoOnCampaignFiles(campaignRefacto):
    if DO_MODIFY_FILES:
        i = 0
        c = 0
        camp = None
        for campaign, tcs in campaignRefacto.iteritems():
            filetype = tcs[2]
            if filetype == 'Campaign':
                prefix = 'Id="%s'
            elif filetype == 'CampaignDescription':
                prefix = 'name="%s'

            for oldTc, newTc in tcs[0].iteritems():
                if oldTc and newTc:
                    if camp != campaign:
                        c += 1
                    i += 1
                    pattern = prefix % oldTc
                    if newTc == "toRemove":
                        # if file is tagged as "to be removed", remove the line in campaign.
                        removeLineInTxtFile(campaign, pattern)
                    else:
                        # replace tc path in campaign with new path
                        subst = prefix % newTc
                        replaceTxtInFile(campaign, pattern, subst)
            camp = campaign
        print "Refactored %d lines in %d campaigns" % (i, c)


def exportAllCsv(allfiles, filename):
    exportpath = os.path.join(os.path.dirname(__file__), filename)
    i = 0
    with open(exportpath, "w") as f:
        f.write("Filetype;Filekey;Filename;Folder;Md5;\n")

        for filekey, filedata in allfiles.iteritems():
            for filetype, files in filedata.iteritems():
                for filename, folder, newfilename, md5 in files:
                    i += 1
                    f.write("%s;%s;%s;%s;%s;\n" % (filetype, filekey, filename, folder, md5))

    print "Exported to %s" % exportpath
    print 'Number of files : %d' % i


def handleExportAll(allFiles):
    exportAllCsv(allFiles, 'xml_files_list.csv')


def handleRelativePath(allFiles):
    # generate dict containing all campaigns/campaignDescription that needs to be reworked
    campaignRefacto = computePathsForAllCampaigns(allFiles, computeCampaignRelativePaths, None, ROOT_DIR)

    # export a csv file containing all changes
    exportCampaignRefactoCsv(campaignRefacto, "relative_path_refacto.csv")

    # modify campaign files
    doRefactoOnCampaignFiles(campaignRefacto)


def handleDuplicates(allFiles, operationType):
    # filter file list, removing files that exists in only on folder
    duplicates = dict(x for x in allFiles.iteritems() if len(x[1]) > 1 or len(x[1].itervalues().next()) > 1)

    # generate new names for duplicated files
    computeDupNewFilename(duplicates)

    # export a csv file containing all refacto data
    exportDuplicatesCsv(duplicates, "duplicated_files.csv")

    if operationType == 'yaml':
        # load a list of double to rework
        duplicates = filterDuplicatesFromYaml(duplicates, 'duplicates_filter.yaml')
        operation = computeCampaignDupsRemoval_yaml
        execConfigDir = ROOT_DIR
    else:
        operation = computeCampaignDupsRemoval_suffix
        execConfigDir = ""

    # generate dict containing all campaigns/campaignDescription that needs to be reworked
    campaignRefacto = computePathsForAllCampaigns(allFiles, operation, duplicates, execConfigDir)

    # export a csv file containing all changes
    exportCampaignRefactoCsv(campaignRefacto, "duplicates_removal.csv")

    # remove unused TC
    doRenameDuplicates(duplicates)

    # modify campaign files
    doRefactoOnCampaignFiles(campaignRefacto)


def handleDuplicatesSuffix(allFiles):
    handleDuplicates(allFiles, 'suffix')


def handleDuplicatesYaml(allFiles):
    handleDuplicates(allFiles, 'yaml')


def handleTcRemoval(allFiles):
    # load a list of TC to remove
    tcsToRemove = filterFromFileList(allFiles, 'tc_list.yaml')

    # generate dict containing all campaigns/campaignDescription that needs to be reworked
    campaignRefacto = computePathsForAllCampaigns(allFiles, computeCampaignTcsRemoval, tcsToRemove)

    # export a csv file containing all changes
    exportCampaignRefactoCsv(campaignRefacto, "tcs_removal.csv")

    # remove unused TC
    doRemoveDuplicates(tcsToRemove)

    # modify campaign files
    doRefactoOnCampaignFiles(campaignRefacto)


def handleInexistingTcs(allFiles):
    # get all campaigns/campaignDescription files
    campaigns = filterOnCampaigns(allFiles)

    exportDuplicatesCsv(campaigns, "campaigns_files.csv")

    # generate dict containing all campaigns/campaignDescription that needs to be reworked
    campaignRefacto = computePathsForAllCampaigns(allFiles, computeCampaignInexistingTcs, None, ROOT_DIR)

    # export a csv file containing all changes
    exportCampaignRefactoCsv(campaignRefacto, "inexisting_tcs.csv")

    # modify campaign files
    doRefactoOnCampaignFiles(campaignRefacto)


def main(argv):
    global DO_MODIFY_FILES
    DO_MODIFY_FILES = False

    # parse command line args
    optlist, args = getopt.getopt(argv, "hadrotum")
    if not optlist:
        raise HelpException()

    for o, a in optlist:
        if o == "-h":
            raise HelpException()
        elif o == "-a":
            handleFunction = handleExportAll
        elif o == "-o":
            handleFunction = handleDuplicatesSuffix
        elif o == "-d":
            handleFunction = handleDuplicatesYaml
        elif o == "-r":
            handleFunction = handleRelativePath
        elif o == "-t":
            handleFunction = handleTcRemoval
        elif o == "-u":
            handleFunction = handleInexistingTcs
        elif o == "-m":
            DO_MODIFY_FILES = True

    # get root path from command line
    paths = ["."]
    if args:
        paths = args
    global ROOT_DIR
    ROOT_DIR = os.path.abspath(paths[0])

    # create dict associating filename with a list of folder containing it
    allFiles = {}
    os.path.walk(ROOT_DIR, listAllXmlFiles, allFiles)

    handleFunction(allFiles)


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except getopt.GetoptError, e:
        print >> sys.stderr, e
        print >> sys.stderr, "Try '%s -h' for help." % sys.argv[0]
        raise SystemExit(2)
    except HelpException, e:
        print __doc__
