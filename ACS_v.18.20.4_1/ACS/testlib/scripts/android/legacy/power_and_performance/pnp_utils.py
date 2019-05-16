#!/usr/bin/env python

########################################################################
#
# @filename:    pnp_utils.py
# @description: PnP helper functions.
# @author:      emilianx.c.ioana@intel.com
#
########################################################################

from testlib.scripts.android.ui import ui_steps

import ast
import time
import os
import MySQLdb
import xml.etree.ElementTree as ET

def get_target_temps(platform):
    ########################################################################
    # Returns an array with target temps for all platform cores. Currently
    # only t100 is implemented.
    ########################################################################
    if platform == "t100":
        return [41,41,41,41]

def get_current_temps(platform, adb_connection):
    ########################################################################
    # Returns an array with current temps for all platform cores. Currently
    # only t100 is implemented. Values read from sysfs.
    ########################################################################
    result = []
    for i in [2,3,4,5]:
        if platform == "t100":
            myRawTemp = adb_connection.parse_file("/sys/devices/platform/coretemp.0/temp{0}_input".format(i)).strip().split("\r\n")
            result.append(int(myRawTemp[0])/1000)
    return result
def get_antutu_possible_tests():
    ########################################################################
    # Returns a dictionary containing all identifiers for antutu tests.
    # TO DO: fetch values from some other place (instead of having them
    # hard-coded here.
    ########################################################################
    return {'UX':['Multitask:','Dalvik:'], \
        'CPU':['CPU integer:','CPU float-point:'], \
        'GPU':['2D graphics:','3D graphics:'], \
        'IO':['Storage I/O:','Database I/O:'], \
        'RAM':['RAM Operation:','RAM Speed:']}
def get_antutu_sleep(platform):
    ########################################################################
    # Returns a dictionary containing all sleep intervals for antutu tests
    # TO DO: fetch values from some other place (instead of having them
    # hard-coded here.
    ########################################################################
    if platform == "t100":
        return {'UX':180, \
            'CPU':160, \
            'GPU':200, \
            'IO':120, \
            'RAM':130, \
            'all': 300}
def get_antutu_result(test_case,possible_tests):
    ########################################################################
    # Fetch Antutu results using the GUI values
    ########################################################################
    result={}
    ui_handler = ui_steps.click_button(view_to_find = {"textContains":"Test"}).uidevice
    ui_steps.click_button(view_to_find = {"textContains":"Test"})()
    ui_steps.click_button(view_to_find = {"textContains":"Details"})()
    ########################################################################
    # If it's a generic AnTuTu test, get all values in the list.
    # Otherwise, only grab the current tests' values
    ########################################################################
    if test_case == "all":
        for key in possible_tests:
            for measurement in possible_tests[key]:
                ui_handler(scrollable=True).scroll.to(text=measurement)
                ############################################################
                # Gpu results have a slightly different format.
                # There's a resolution box which is contained in the same
                # text field as the score. As a result I split by '] ' and
                # take the last value in this array; for normal fields, this
                # is the only value, for GPU fields it's the second
                ############################################################
                currentValue=int(ui_handler(textContains=measurement).\
                right(textMatches="[0-9\[x\]\s]+").info[u'text'].split("] ")[-1])
                result.update({str(measurement):currentValue})
        result={'all':sum(result.itervalues())}
    else:
        for measurement in possible_tests[test_case]:
            ui_handler(scrollable=True).scroll.to(text=measurement)
            ################################################################
            #same deal as above
            ################################################################
            currentValue=int(ui_handler(textContains=measurement).\
            right(textMatches="[0-9\[x\]\s]+").\
            info[u'text'].split("] ")[-1])
            result.update({str(measurement):currentValue})
    return result

def get_threedmark_sleep(platform):
    ########################################################################
    # Returns sleep interval for 3D Mark
    ########################################################################
    if platform == "t100":
        return 500

def get_threedmark_result(adb_connection,test_case):
    ########################################################################
    # Fetch 3D Mark results using the logcat data.
    ########################################################################
    if test_case=="Ice Storm":
        toreplace="true"
    else:
        toreplace="false"
    result = ast.literal_eval(adb_connection.parse_logcat(grep_for="result dialog object").\
        split("current\":")[1].split(",\"best")[0].replace(toreplace,"\"true\""))
    unwanted = ["date", "benchmark_test", "os", "benchmark_version", "default_settings"]
    for unw in unwanted:
        result.pop(unw)
    return result

def get_quadrant_sleep(platform):
    ########################################################################
    # Returns sleep interval for Quadrant
    ########################################################################
    if platform == "t100":
        return {'all':80}

def get_quadrant_result(adb_connection):
    ########################################################################
    # Fetch Quadrant results using the logcat data.
    ########################################################################
    result={}
    results = adb_connection.parse_logcat(grep_for="agg").split("\r\n")
    for r in results:
        myBuffer = r.split(" ")
        measurement = myBuffer[-5]
        currentValue = myBuffer[-1]
        result.update({str(measurement):currentValue})
    return result

def get_glbenchmark_sleep(platform):
    ########################################################################
    # Returns sleep interval for GL Benchmark
    ########################################################################
    if platform == "t100":
        return {'all':420}

def get_glbenchmark_result(adb_connection):
    ########################################################################
    # Fetch GL Benchmark results using the xml result file.
    ########################################################################
    result={}
    adb_connection.get_file("/mnt/shell/emulated/0/Android/data/com.glbenchmark.glbenchmark27/cache/last_results_2.7.0.xml", "/tmp/gl.xml")
    myXml = ET.parse('/tmp/gl.xml')
    allResults = myXml.getroot()
    for child in allResults.findall('test_result'):
        measurement = child.find('title').text.split(" ")[-2] + \
        child.find('type').text.split(" ")[-1]
        currentValue = child.find('fps').text.split(" ")[0]
        result.update({str(measurement):currentValue})
    return result

def wait_for_cooldown(adb_connection, platform):
    ########################################################################
    # Sleeps until target temps are reached. Always use this with pnp tests.
    ########################################################################

    temperatures = get_target_temps(platform)
    current_temperatures = get_current_temps(platform, adb_connection)

    while current_temperatures > temperatures:
        print "waiting for cooldown" + str(current_temperatures)
        time.sleep(2)
        temperatures = get_target_temps(platform)
        current_temperatures = get_current_temps(platform, adb_connection)
    return True

#########################################################################
# MYSQL SPECIFIC UTILS BELOW
# TO DO: Proper descriptions
#########################################################################
def get_mysql_info(ini_location):
    reportini=ini_location+"/report.ini"
    props={}
    f = open(reportini,"r")
    for line in f:
        key,value = line.split("=",1)
        props[key] = value.strip()
    return props

def run_query(query,connection_location = "/sp/pnp_automation/testlib/scripts/power_and_performance"):
    props = get_mysql_info(connection_location)
    connection=MySQLdb.connect(
        host=props["host"],
        user=props["mysql_user"],
        passwd=props["mysql_password"],
        db=props["mysql_db"])
    cursor=connection.cursor()
    value = False
    try:
        cursor.execute(query)
        connection.commit()
        value = cursor.fetchall()
    except:
        connection.rollback()
    return value

#########################################################################
# RUNNER SPECIFIC UTILS BELOW
#########################################################################

def check_current_version(adb_connection):
    line = adb_connection.parse_file("/system/build.prop", grep_for = "ro.build.version.incremental=")
    return line.strip().split("=")[1]

def decide_next_test(adb_connection):
    ############################################################################
    # Decide next benchmark and test case to be performed on DUT. Decision is
    # based on available test history for current version.
    # Note to self:
    #       - add some randomness in selecting from available test cases
    ############################################################################
    value=[]
    removableKpis = [0,37,38,39,40,41,42,43,48,49,4,7,10,11,13,20,30,31,26,27,28,29]
    validKpis=range(53)

    ############################################################################
    # Check if a new version is available for flashing
    ############################################################################
    version = check_current_version(adb_connection)
    query = "SELECT build_id,build_path FROM `sp_builds` where build_id > '{0}' \
            order by build_id asc limit 1".format(version)
    queryresult = run_query(query)
    if len(queryresult) > 0:
        nextNewestVersion = queryresult[0]
        print "next version increment is: ", str(nextNewestVersion[0])
    else:
        nextNewestVersion = version
        print "No newer version available."

    ############################################################################
    # Count current number of distinct KPIs (all == 33, hence the comparison)
    ############################################################################
    value =  run_query("select count(distinct kpi) from `measurements_test` \
                            where build_version='{0}'".format(version))
    ############################################################################
    # If not all have been performed, remove the ones which have from the
    # pool of possible tests
    ############################################################################
    if value[0][0] < 29:
        value =  run_query("select distinct kpi from `measurements_test` \
                                where build_version='{0}'".format(version))
        for v in removableKpis:
            validKpis.remove(v)
        for v in value:
            validKpis.remove(v[0])
        testKpi=validKpis[0]
        print "test kpi will be:", str(testKpi)
    ############################################################################
    # If all have been performed, get each KPI and its corresponding number of
    # samples. Least ran tests will then be prioritized.
    ############################################################################
    else:
        value = run_query("select count(*),kpi from `measurements_test` \
                                where build_version='{0}' group by kpi \
                                order by count(*) asc limit 1"\
                                .format(version))
        testKpi = value[0][1]
        currentSamples = value[0][0]

        print "test kpi will be:", str(testKpi)
        print "current samples is:", str(currentSamples)

    ############################################################################
    # If there's a newer version and at least 20 iterations for each KPI have
    # been performed, flash the new version.
    # For consistency with other tests, flashing a device is similar to running
    # a test for which the benchmark is "flashing" and the test case is
    # the actual version path.
    ############################################################################
    if nextNewestVersion != version and currentSamples > 20:
        benchmark = "Flash"
        test_case = nextNewestVersion[1]
        print "New version available. Dumping current test for flashing"
    else:
        ########################################################################
        # get corresponding value from KPI table
        ########################################################################
        query = "select * from `KPI` where kpiId='{0}'".format(testKpi)
        value = run_query(query)[0]

        ########################################################################
        # set runner params
        ########################################################################
        benchmark = {"Antutu" : "Antutu", \
        "Ice Storm" : "3D Mark", \
        "Ice Storm Extreme" : "3D Mark", \
        "GL Benchmark" : "GL Benchmark", \
        "Quadrant" : "Quadrant"}[value[1]]


        if benchmark == "Antutu":
            test_case = {"2D" : "GPU", \
            "3D" : "GPU", \
            "CPU" : "CPU", \
            "CPU Float" : "CPU", \
            "RAM" : "RAM", \
            "IO Storage" : "IO", \
            "DB" : "IO", \
            "Dalvik" : "UX", \
            "Multitask" : "UX", \
            "Ram Speed" : "RAM", \
            "Total Score" : "all"}[value[2]]
        elif benchmark == "Quadrant" or benchmark == "GL Benchmark":
            test_case = "all"
        else:
            test_case = value[1]
    return {"benchmark": benchmark,
            "test_case":test_case}

def set_next_test(runnerini, machine_ip, runner_props):
    f = open(runnerini,"w")
    benchmark_line="benchmark={0}\n".format(runner_props["benchmark"])
    serial_line="serial={0}:5555\n".format(machine_ip)
    print serial_line
    testcase_line="test={0}\n".format(runner_props["test_case"])
    f.write(benchmark_line)
    f.write(serial_line)
    f.write(testcase_line)
    f.close()

def set_runner_props(runnerini):
    runner_props = {}
    f = open(runnerini,"r")
    for line in f:
        key,value = line.split("=",1)
        runner_props[key] = value.strip()
    return runner_props
def kpi_id_of(benchmark, identifier):
    query = "select kpiId from `KPI` where name='{0}' and identifier='{1}'".format(benchmark, identifier)
    return str(run_query(query)[0][0])

def get_report_details (adb_connection, benchmark, identifier, result, platform, ip):
    build_specific = {}
    if platform == "t100":
        build_specific["platform"] = "15"
    build_specific["score"] = str(result)
    parameters={"build_date": "ro.build.date=", "build_version":"ro.build.version.incremental=", "os_build":"ro.build.version.release"}
    for param,value in parameters.iteritems():
        line = adb_connection.parse_file("/system/build.prop", grep_for = value)
        currentValue = line.strip().split("=")[1]
        build_specific.update({param:currentValue})
    ############################################################################
    # change timezone in order to easily parse build time from build.prop
    ############################################################################
    os.environ['TZ'] = 'US/Pacific'
    build_specific["build_date"]=time.strftime("%Y-%m-%d %H:%M:%S", \
        time.strptime(str(build_specific["build_date"]).strip(),"%a %b %d %H:%M:%S %Z %Y"))
    build_specific["test_date"]=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    build_specific["kpi_id"] = kpi_id_of(benchmark, identifier)
    build_specific["machine"] = ip
    return build_specific

def create_push_result_query(table, build_specific):
    query = 'INSERT into `{0}` (`kpi`,'.format(table) + \
                                '`platform`,' + \
                                '`build_date`,' + \
                                '`build_version`,' + \
                                '`test_date`,' + \
                                '`score`,' + \
                                '`os_build`,' + \
                                '`machine`)'
    query += " VALUES ('"+\
                build_specific["kpi_id"] + "', '" + \
                build_specific["platform"] + "', '" + \
                build_specific["build_date"] + "', '" + \
                build_specific["build_version"] + "', '" + \
                build_specific["test_date"] + "', '" + \
                build_specific["score"] + "', '" + \
                build_specific["os_build"] + "', '" + \
                build_specific["machine"] + "')"

    return query
