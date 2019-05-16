# coding=utf-8
import os
import fnmatch
import sys
import shutil
import requests
import lxml.etree as etree

ATF_PATH = os.path.dirname(os.path.abspath(__file__))
ACS_TEST_OUTPUT = os.path.join(ATF_PATH, "ACS_OUPUT")
ARTI_INST_ANDROID_PATH = "INSTRUMENTATION/ANDROID"
# TODO: handle new ANDROID_INST parameters & count & fail_retry ATF param


def locate(pattern, root=os.curdir):
    for path, dirs, files in os.walk(os.path.abspath(root)):
        for filename in fnmatch.filter(files, pattern):
            yield os.path.join(path, filename)


def parse_ini(f):
    data = {}
    with open(f, "r") as init_content:
        for line in init_content.readlines():
            if line.strip() and "=" in line:
                key, value = line.split("=", 1)
                if "test_runner" in key and "InstrumentationTests_Runner.py" not in value:
                    return {}
                elif "test_script" in key:
                    if ".py" in value:
                        return {}
                    data["test_script"] = value.strip()
                elif "am_extra" in key:
                    data["am_extra"] = value.strip()
                elif "timeout" in key:
                    data["timeout"] = value.strip()
                elif "apks" in key:
                    apks = value.strip().split(":")
                    new_apks = []
                    for apk in apks:
                        new_apks.append(apk)
                    data["apks"] = ";".join(new_apks)
                elif "media_file" in key:
                    data["media_file"] = value.strip().replace(":", ";")
                elif "adb_destination" in key:
                    data["adb_destination"] = value.strip().replace(":", ";")
                elif "set_prop" in key:
                    data["set_prop"] = value.strip().replace(":", ";")
                elif "retrieve_artefacts" in key:
                    data["retrieve_artefacts"] = value.strip().replace(":", ";")
                elif "count" in key:
                    data["count"] = value.strip()
                elif "fail_retry" in key:
                    data["fail_retry"] = value.strip()
                elif "PRE_REBOOT" in key:
                    data["PRE_REBOOT"] = "False" if "0" in value else "True"
                elif "POST_REBOOT" == key.strip():
                    data["POST_REBOOT"] = "False" if "0" in value else "True"
                elif "POST_REBOOT_NOK" in key:
                    data["POST_REBOOT_NOK"] = "False" if "0" in value else "True"
                elif "TAGS" in key:
                    data["TAGS"] = value.strip()
    return data


def get_atf_data():
    atf_data = {}
    for f in locate("*.ini", ATF_PATH):
        if "test_config" in f:
            data = parse_ini(f)
            if data:
                atf_data[f] = data
    return atf_data


def gen_acs_data(atf_data):
    atf_scripts_folder = os.path.join(ATF_PATH, 'test_config', "")
    acs_campaign = etree.parse(os.path.join(ATF_PATH, "acs_camp.xml"))
    acs_test_cases = acs_campaign.xpath("//TestCases")[0]
    acs_tc_generated = 0
    atf_tc_skipped = 0
    campaign_per_tag = {}

    def check_media_is_on_artifactory(artifact,
                                      artifactory_uri="https://mcg-depot.intel.com/artifactory",
                                      repo_name="acs_test_artifacts"):
        is_present = False
        full_uri = artifactory_uri + "/" + repo_name + "/" + artifact
        proxy = {'http': "", 'https': ""}
        response = None
        try:
            response = requests.get(full_uri, proxies=proxy, timeout=5, stream=True, verify=False)
        except requests.exceptions.RequestException as e:
            error_msg = "%s: An error occur during connection with url : (" + str(e) + ")"
            print (error_msg)
        if response is not None:
            is_present = response.status_code == 200

        return is_present

    def hack_acs_test_artifacts(artifact, debug=True):
        """
        Nasted method to compute acs_test_artifacts path...
        """
        return_artifactory_sub_path = ""
        splited_uri = artifact.split('/')
        media_type = ""
        if len(splited_uri) >= 2:
            if splited_uri[0].lower() in ["audio", "video"]:
                media_type = splited_uri[0]
            file_name = splited_uri[-1]
        else:
            if debug:
                print "hack_acs_test_artifacts: i don't know how to compute test_artifacts uri : %s" % (artifact)
            return ""

        extension = file_name.split('.')[-1]
        type = file_name.split('_')[0]
        # is audio ?
        if media_type.lower() == "audio":
            if extension.lower() in ["amr", "aac", "ogg", "flac", "midi", "pcm", "wma"] or \
                extension.lower() == "mp3" and type.lower() != "aac" or \
                extension.lower() == "wav" and type.lower() != "pcm":
                return_artifactory_sub_path = "%s/%s/%s" % (media_type.upper(), extension.upper(), file_name)
            elif extension.lower() == "mid":
                return_artifactory_sub_path = "AUDIO/MIDI/%s" % (file_name)
            else:
                return_artifactory_sub_path = "%s/%s/%s" % (media_type.upper(), type.upper(), file_name)
        elif media_type.lower() == "video":
            if extension.lower() in ["3gp"]:
                return_artifactory_sub_path = "VIDEO/%s/%s" % (extension.upper(), file_name)
            elif extension.lower() in ["wmv", "mkv", "webm"]:
                return_artifactory_sub_path = "VIDEO/%s/%s" % (type.upper(), file_name)
            elif extension.lower() in ["mp4"]:
                if type.lower() in ["h264", "h263", "mpeg4", "vc1", "vc2"]:
                    return_artifactory_sub_path = "VIDEO/%s/%s" % (type.upper(), file_name)
                else:
                    for vid_type in ["h264", "h263", "mpeg4", "vc1", "vc2"]:
                        if vid_type in file_name:
                            return_artifactory_sub_path = "VIDEO/%s/%s" % (vid_type.upper(), file_name)
                            break
                    else:
                        if debug:
                            print "hack_acs_test_artifacts: i don't know how to compute test_artifacts uri : %s" % (
                                artifact)
            else:
                if debug:
                    print "hack_acs_test_artifacts: i don't know how to compute test_artifacts uri : %s" % (artifact)
        else:
            if debug:
                print "hack_acs_test_artifacts: i don't know how to compute test_artifacts uri : %s" % (artifact)
        return return_artifactory_sub_path

    error_acs_test_artifacts = {}

    for key, value in atf_data.items():
        contor = True
        if "TAGS" in value and 'abt' not in value["TAGS"].lower():
            continue
        try:
            acs_tc_template = etree.parse(os.path.join(ATF_PATH, "acs_tc_temp.xml"))
            atf_script_path = key.replace(atf_scripts_folder, "")
            path, file_name = os.path.split(atf_script_path)
            path = os.path.join("TC", path)
            dest_path = os.path.join(ACS_TEST_OUTPUT, path)
            # Update TC
            acs_tc_template.xpath('//Name[. = "TIMEOUT"]/../Value')[0].text = value["timeout"]
            acs_tc_template.xpath('//Name[. = "PROCESS_NAME"]/../Value')[0].text = value["test_script"]
            if "am_extra" in value:
                acs_tc_template.xpath('//Name[. = "AM_EXTRA"]/../Value')[0].text = value["am_extra"]
            if "apks" in value:
                apk_artifacts = [x.strip() for x in value["apks"].split(";")]
                # init_test_step_node = acs_tc_template.xpath('/TestCase/TestSteps/Initialize')[0]
                artifact_id = 0
                new_apk_path = []
                for artifact in apk_artifacts:
                    arti_path = "{0}/{1}".format(ARTI_INST_ANDROID_PATH, artifact)
                    if not check_media_is_on_artifactory(arti_path):
                        contor = False
                        if artifact not in error_acs_test_artifacts:
                            error_acs_test_artifacts[artifact] = [atf_script_path]
                        elif atf_script_path not in error_acs_test_artifacts[artifact]:
                            error_acs_test_artifacts[artifact].append(atf_script_path)
                        atf_tc_skipped += 1
                        break

                    # local_dest = "APK_PATH_IN_CACHE{0}".format(artifact_id)
                    # get_artifact_elm = etree.Element("TestStep",
                    #                                  attrib=dict(Id="GET_ARTIFACT",
                    #                                              ARTIFACT=arti_path,
                    #                                              ARTIFACT_SOURCE="FROM_BENCH:ARTIFACT_MANAGER:URI",
                    #                                              TRANSFER_TIMEOUT="DEFAULT",
                    #                                              STORED_FILE_PATH=local_dest,
                    #                                              EQT="DEFAULT"))
                    # install_file_elm = etree.Element("TestStep",
                    #                                  attrib=dict(Id="INSTALL_APP",
                    #                                              DEVICE="PHONE1",
                    #                                              FILE_PATH="FROM_CTX:{0}".format(local_dest),
                    #                                              BACKUP="False",
                    #                                              BACKUP_FILE_PATH="tmp",
                    #                                              TIMEOUT="DEFAULT"))
                    #
                    # init_test_step_node.insert(0, install_file_elm)
                    # init_test_step_node.insert(0, get_artifact_elm)
                    artifact_id += 1
                    new_apk_path.append(arti_path)
                acs_tc_template.xpath('//Name[. = "APKS"]/../Value')[0].text = ";".join(new_apk_path)

            if "media_file" in value:
                media_artifacts = [x.strip() for x in value["media_file"].split(";")]
                if value.get("adb_destination"):
                    test_files_dest = [x.strip() for x in value["adb_destination"].split(";")]
                    artifact_data = [(el, test_files_dest[idx] if (idx + 1) <= len(test_files_dest)
                                      else test_files_dest[-1]) for idx, el in enumerate(media_artifacts)]
                else:
                    artifact_data = [(x, "/mnt/sdcard/") for x in media_artifacts]

                # init_test_step_node = acs_tc_template.xpath('/TestCase/TestSteps/Initialize')[0]
                artifact_id = 0
                new_media_path = []
                media_dest = []
                for artifact, device_destination in artifact_data:
                    save_artifact = artifact
                    artifact = hack_acs_test_artifacts(artifact, debug=False)
                    if not artifact:
                        contor = False
                        if save_artifact not in error_acs_test_artifacts:
                            error_acs_test_artifacts[save_artifact] = [atf_script_path]
                        elif atf_script_path not in error_acs_test_artifacts[save_artifact]:
                            error_acs_test_artifacts[save_artifact].append(atf_script_path)
                        atf_tc_skipped += 1
                        break
                    else:
                        if not check_media_is_on_artifactory(artifact):
                            contor = False
                            if save_artifact not in error_acs_test_artifacts:
                                error_acs_test_artifacts[save_artifact] = [atf_script_path]
                            elif atf_script_path not in error_acs_test_artifacts[save_artifact]:
                                error_acs_test_artifacts[save_artifact].append(atf_script_path)
                            atf_tc_skipped += 1
                            break
                    # local_dest = "FILE_PATH_IN_CACHE{0}".format(artifact_id)
                    # device_dest = "FILE_PATH_ON_DEVICE{0}".format(artifact_id)
                    # get_artifact_elm = etree.Element("TestStep",
                    #                                  attrib=dict(Id="GET_ARTIFACT",
                    #                                              ARTIFACT=artifact,
                    #                                              ARTIFACT_SOURCE="FROM_BENCH:ARTIFACT_MANAGER:URI",
                    #                                              TRANSFER_TIMEOUT="DEFAULT",
                    #                                              STORED_FILE_PATH=local_dest,
                    #                                              EQT="DEFAULT"))
                    # push_file_elm = etree.Element("TestStep",
                    #                               attrib=dict(Id="INSTALL_FILE",
                    #                                           DEVICE="PHONE1",
                    #                                           FILE_PATH="FROM_CTX:{0}".format(local_dest),
                    #                                           TYPE="media",
                    #                                           DESTINATION=device_destination,
                    #                                           TIMEOUT="DEFAULT",
                    #                                           DESTINATION_STORED_PATH=device_dest))
                    #
                    # init_test_step_node.insert(0, push_file_elm)
                    # init_test_step_node.insert(0, get_artifact_elm)
                    artifact_id += 1
                    new_media_path.append(artifact)
                    media_dest.append(device_destination)
                acs_tc_template.xpath('//Name[. = "TEST_FILES"]/../Value')[0].text = ";".join(new_media_path)
                acs_tc_template.xpath('//Name[. = "TEST_FILES_DEST"]/../Value')[0].text = ";".join(media_dest)
            if not contor:
                continue
            # Create folder if needed
            if not os.path.isdir(dest_path):
                os.makedirs(dest_path)
            if "set_prop" in value:
                acs_tc_template.xpath('//Name[. = "SET_PROPS"]/../Value')[0].text = value["set_prop"]
            if "retrieve_artefacts" in value:
                acs_tc_template.xpath('//Name[. = "RETRIEVE_ARTIFACT"]/../Value')[0].text = value["retrieve_artefacts"]
            if 'PRE_REBOOT' in value:
                acs_tc_template.xpath('//Name[. = "PRE_REBOOT"]/../Value')[0].text = value["PRE_REBOOT"]
            if "POST_REBOOT" in value:
                acs_tc_template.xpath('//Name[. = "POST_REBOOT"]/../Value')[0].text = value["POST_REBOOT"]
            if "POST_REBOOT_NOK" in value:
                acs_tc_template.xpath('//Name[. = "POST_REBOOT_NOK"]/../Value')[0].text = value["POST_REBOOT_NOK"]
            if "TAGS" in value:
                acs_tc_template.xpath('//Name[. = "TAGS"]/../Value')[0].text = value["TAGS"].strip('"')
                for tag in value["TAGS"].split(","):
                    tag = tag.strip("/").strip("\"")
                    if tag in campaign_per_tag:
                        campaign_per_tag[tag].append(os.path.join("..", path, format(os.path.splitext(file_name)[0])))
                    else:
                        campaign_per_tag[tag] = [os.path.join("..", path, format(os.path.splitext(file_name)[0]))]
            if "count" in value:
                acs_tc_template.xpath('//b2bIteration')[0].text = value["count"]
            if "fail_retry" in value:
                acs_tc_template.xpath('//TcMaxAttempt')[0].text = value["fail_retry"]

            with open(os.path.join(dest_path, "{0}.xml".format(os.path.splitext(file_name)[0])), "w") as f:
                f.write(etree.tostring(acs_tc_template, pretty_print=True, xml_declaration=True))
                print("{0} generated.".format(f.name))

            # Update campaign
            tc_elem = etree.SubElement(acs_test_cases, "TestCase")
            tc_elem.attrib["Id"] = os.path.join("..", path, format(os.path.splitext(file_name)[0]))
            acs_tc_generated += 1

        except Exception as ex:
            print("Cannot convert {0} => {1}".format(key, ex.message))

    if not os.path.isdir(os.path.join(ACS_TEST_OUTPUT, "CAMPAIGN")):
        os.makedirs(os.path.join(ACS_TEST_OUTPUT, "CAMPAIGN"))

    # Write campaign ALL
    with open(os.path.join(ACS_TEST_OUTPUT, "CAMPAIGN", "ABT_ALL.xml"), "w") as f:
        f.write(etree.tostring(acs_campaign, pretty_print=True, xml_declaration=True))
    # Create one campaign per tag
    for k, v in campaign_per_tag.items():
        acs_campaign = etree.parse(os.path.join(ATF_PATH, "acs_camp.xml"))
        acs_test_cases = acs_campaign.xpath("//TestCases")[0]
        for tc in v:
            tc_elem = etree.SubElement(acs_test_cases, "TestCase")
            tc_elem.attrib["Id"] = tc
        with open(os.path.join(ACS_TEST_OUTPUT, "CAMPAIGN", "ABT_{0}.xml".format(k)), "w") as f:
            f.write(etree.tostring(acs_campaign, pretty_print=True, xml_declaration=True))
    for artifact_error, atf_files in error_acs_test_artifacts.items():
        print ("\n{0} need to be uploaded on artifactory server !".format(artifact_error))
        print ("ATF tests not converted:")
        for atf_file in atf_files:
            print("\t{0}".format(atf_file))

    print("Generated {0} tests.".format(acs_tc_generated))
    print("Skipped {0} tests.".format(atf_tc_skipped))


def main():
    if os.path.isdir(ACS_TEST_OUTPUT):
        shutil.rmtree(ACS_TEST_OUTPUT)
    atf_data = get_atf_data()
    gen_acs_data(atf_data)
    return 0

if __name__ == '__main__':
    sys.exit(main())
