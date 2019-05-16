#!/usr/bin/env python
#-*- encoding: utf-8 -*-

import os
import time
import commands


class APConfImpl(object):
    def __init__(self):
        self.cur_path = os.path.split(os.path.realpath(__file__))[0]
        self.cur_list = self.cur_path.split(os.sep)
        self.conf_list = self.cur_list[:-3] + ["testplan", "wifi", "apconf"]
        self.__CONF_PATH__ = (os.sep).join(self.conf_list)
        print self.__CONF_PATH__

        self.__STORE_PATH__ = "/tmp"
        self.__CLIENT_PATH__ = "/tmp/ap_conf"
        self.__SERVER_PATH__ = "wifi"
        self.__LOG_PATH__ = "/tmp/ap_change.log"
        self.__AGENT_SC__ = "host_bt_auto_accept_agent"
        self.__SEND_SC__ = "host_bt_send"
        self.__MAC_SC__ = "test-device"
        self.__MAC_ADD__ = self.get_mac()
        self.__TIMEOUT__ = 120
        self.__OBEX_PATH__ = "/user/lib/obexd"
        self.__OBEX_CLIENT__ = "obex-client"
        self.__CONFS__ = ["ap_ready.conf", "ap_process.conf", "ap_fail.conf"]

    def run_command(self, input_cmd):
        r_code, r_mes = commands.getstatusoutput(input_cmd)
        return r_code, r_mes

    def get_mac(self):
        """Get the Mac Address"""
        script_path = self.__CLIENT_PATH__ + os.sep + self.__MAC_SC__
        cmd = "python %s list" % script_path
        code, mes = commands.getstatusoutput(cmd)
        mes_list = mes.split()
        return mes_list[0]

    def read_log(self):
        if not os.path.exists(self.__LOG_PATH__):
            return ""
        f = open(self.__LOG_PATH__)
        data = f.read()
        f.close()
        return data

    def read_file(self, file_path):
        f = open(file_path)
        lines = f.readlines()
        f.close()
        return lines

    def write_log(self, confName):
        f = open(self.__LOG_PATH__, "w")
        f.write(confName)
        f.close()

    def kill_client(self):
        """Check the client is right"""
        cmd = "ps -ef | grep obex"
        kill_cmd = "kill -9 %s"

        code, mes = commands.getstatusoutput(cmd)
        lines = mes.split("\n")

        result = None
        for line in lines:
            if line.find("obex-client") != -1:
                result = line

        if result:
            print "[INFO]: Kill obex client"
            cur_id = result.split()[1]
            os.system(kill_cmd % cur_id)

        self.start_client()

    def start_client(self):
        """Check the client is right"""
        obex_path = self.__OBEX_PATH__ + os.sep + self.__OBEX_CLIENT__
        cmdopen = "python %s &" % obex_path

        cmdcheck = "ps -ef | grep obex"
        code, mes = commands.getstatusoutput(cmdcheck)
        lines = mes.split("\n")
        for line in lines:
            if line.find("obex-client") != -1:
                print "[INFO]: Obex-client has already launched"
            else:
                code, mes = commands.getstatusoutput(cmdopen)

    def kill_agent(self):
        """Kill the agent"""
        cmd = "ps -ef | grep hci0"
        kill_cmd = "kill -9 %s"
        for i in range(3):
            code, mes = commands.getstatusoutput(cmd)
            lines = mes.split("\n")

            result = None
            for line in lines:
                if line.find(self.__AGENT_SC__) != -1:
                    result = line
            print line
            if result:
                cur_id = result.split()[1]
                par_id = result.split()[2]
                if par_id != "1":
                    print "[INFO]: Kill auto agent process"
                    os.system(kill_cmd % cur_id)
                else:
                    return True
            else:
                return True
        return False

    def start_agent(self):
        """Start the accept agent"""
        script_path = self.__CLIENT_PATH__ + os.sep + self.__AGENT_SC__
        check_cmd = "ps -ef | grep %s" % self.__AGENT_SC__
        start_cmd = "python %s hci0 &" % script_path
        for i in range(3):
            code, mes = commands.getstatusoutput(check_cmd)
            if mes.find("%s hci0" % self.__AGENT_SC__) != -1:
                return True
            print "[INFO]: Restart accept agent"
            os.system(start_cmd)
            time.sleep(3)
        return False

    def get_send_conf(self, confName, mes):
        """Create a send conf"""
        old_conf = self.__CONF_PATH__ + os.sep + confName
        lines = self.read_file(old_conf)

        new_conf = self.__STORE_PATH__ + os.sep + confName
        f = open(new_conf, "w")
        f.write("# %s\n" % mes)
        for line in lines:
            f.write(line)
        f.close()
        if os.path.exists(new_conf):
            return True
        return False

    def send_conf(self, confName):
        """Send conf to device"""
        print "[INFO]: Send conf file to server"
        script_path = self.__CLIENT_PATH__ + os.sep + self.__SEND_SC__
        conf_path = self.__STORE_PATH__ + os.sep + confName
        info = (script_path, self.__MAC_ADD__, self.__SERVER_PATH__, conf_path)
        send_cmd = "python %s -d %s -c %s -p %s" % info
        check_info = (script_path, self.__MAC_ADD__, self.__SERVER_PATH__)
        check_cmd = "python %s -d %s -c %s -l" % check_info

        s_code, s_mes = self.run_command(send_cmd)
        print "[INFO]: Send return code:", s_code
        if s_code != 0:
            return False

        c_code, c_mes = self.run_command(check_cmd)
        conf_list = c_mes.split("\n")
        print "[INFO]: Sent file on server:", conf_list
        if confName in conf_list or "ap_process.conf" in conf_list:
            return True
        return False

    def check_finish(self, confName, timeout=60):
        """Check finish to change ap"""
        print "[INFO]: Check finish to change ap"
        script_path = self.__CLIENT_PATH__ + os.sep + self.__SEND_SC__
        check_info = (script_path, self.__MAC_ADD__, self.__SERVER_PATH__)
        check_cmd = "python %s -d %s -c %s -l" % check_info

        for i in range(timeout):
            c_code, c_mes = self.run_command(check_cmd)
            conf_list = c_mes.split("\n")
            if len(conf_list) == 0:
                print "[INFO]: No file in server"
                return False
            elif len(conf_list) == 1:
                value = conf_list[0]
                if value == "ap_fail.conf":
                    print "[INFO]: Fail to set ap"
                    return False
                elif value == "ap_ready.conf":
                    print "[INFO]: Success to set ap"
                    return True
            time.sleep(1)
        print "[INFO]: Timeout, fail to set ap"
        return False

    def ap_conf(self, confName, otherConf="ALONE"):
        conf_path = self.__STORE_PATH__ + os.sep + confName
        conf_list = os.listdir(self.__CONF_PATH__)
        self.start_client()
        if confName not in conf_list:
            raise Exception("%s does not exists" % (self.__CONF_PATH__ + os.sep + confName))

        pre_conf = self.read_log()
        if pre_conf == confName:
            print "[INFO]: AP has been set before"
        else:
            print "[INFO]: Ready to set ap"
            # Check the file is ready
            file_flag = self.get_send_conf(confName, otherConf)
            if not file_flag:
                raise Exception("Fail to create send conf")

            kill_flag = self.kill_agent()
            if not kill_flag:
                raise Exception("Fail to kill auto agent process")

            # Check the monitor is ready
            start_flag = self.start_agent()
            if not start_flag:
                raise Exception("Fail to start accept agent")

            # Ready to send file and check it finish
            for i in range(3):
                finish_flag = False
                send_flag = self.send_conf(confName)
                if not send_flag:
                    print "[INFO]: Send fail, retry"
                    self.kill_client()
                    continue

                finish_flag = self.check_finish(confName, self.__TIMEOUT__)

                if finish_flag:
                    self.write_log(confName)
                    os.remove(conf_path)
                    return True

            if not finish_flag:
                raise Exception("Fail to set the ap")


if __name__ == "__main__":
    #os.system("whoami")
    #APConfImpl().ap_conf("default_ap.conf", "ALONE")
    #print APConfImpl().run_command("ls")
    APConfImpl()
