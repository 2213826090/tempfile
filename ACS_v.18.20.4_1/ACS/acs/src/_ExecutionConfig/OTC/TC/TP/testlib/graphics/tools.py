# coding: UTF-8
import ConfigParser
import sys
import os
from testlib.util.common import g_common_obj

class ConfigHandle():

    def read_configuration(self, title, key, path, filename="configuration.conf"):
        # this function read configuration from configuration.conf file and return the value
        cf = ConfigParser.ConfigParser()
        file_dir = path + "/" + filename
        print file_dir
        cf.read(file_dir)
        str_val = cf.get(title, key)
        return str_val

    def set_configuration(self, path, title, key, value, filename="configuration.conf"):
        # write new configuration value to the appointed configuration file
        cf = ConfigParser.ConfigParser()
        file_dir = path + "/" + filename
        cf.read(file_dir)
        
        try:
            cf.set(title, key, value)
            cf.write(open(file_dir, "w"))
        except Exception as d:
            print "can't write configuration information in " + path + " " + filename

    def write_wifi_configuration(self, ap_name, password):
        # write configuration to wifiap.conf
        base_path = os.path.abspath(__file__)
        base_path = os.path.join(base_path, "..")
        base_path = os.path.normpath(base_path)
        print base_path
        self.set_configuration(base_path, "wifiap", "ap_ssid", ap_name, "wifiap.conf")
        self.set_configuration(base_path, "wifiap", "ap_passwd", password, "wifiap.conf")

    def write_google_account_configuration(self, username, password):
        # write google account configuration to configuration.conf
        base_path = os.path.abspath(__file__)
        base_path = os.path.join(base_path, "..")
        base_path = os.path.normpath(base_path)
        print base_path
        self.set_configuration(base_path, "google_account", "user_name", username, "configuration.conf")
        self.set_configuration(base_path, "google_account", "password", password, "configuration.conf")
        
    def write_email_configuration(self, receiver_addr):
        # write hangout configuration to configuration.conf
        base_path = os.path.abspath(__file__)
        base_path = os.path.join(base_path, "..")
        base_path = os.path.normpath(base_path)
        path = base_path[0:-13] + "mtbf/" + "email/" + "gmail/" + "conf" 
        self.set_configuration(path, "Email", "Receiver_Email_Addr", receiver_addr)
   
    def read_wifi_configuration(self):
        # read information from wifi configuraion information from configuration.conf
        # and return ssid and password
        ssid = self.read_configuration("wifisetting", "ssid", "/etc/oat/", "sys.conf")
        password = self.read_configuration("wifisetting", "passwd", "/etc/oat/", "sys.conf")
        return ssid, password

    def read_google_account_configuration(self):
        # read information from configuration.conf and return username and password
        base_path = os.path.abspath(__file__)
        base_path = os.path.join(base_path, "..")
        base_path = os.path.normpath(base_path)
        username = self.read_configuration("google_account", "user_name", base_path, "configuration.conf")
        password = self.read_configuration("google_account", "password", base_path, "configuration.conf")
        return username, password

    def read_content_url(self):
        # read download mtbf-content.zip's url
        base_path = os.path.abspath(__file__)
        base_path = os.path.join(base_path, "..")
        base_path = os.path.normpath(base_path)
        url = self.read_configuration("content_url", "url", base_path)
        return url

    def write_content_url(self, url):
        # write download mtbf-content.zip url
        base_path = os.path.abspath(__file__)
        base_path = os.path.join(base_path, "..")
        base_path = os.path.normpath(base_path)
        self.set_configuration("content_url", "url", url, base_path)

    def check_apps(self, package_name):
        result = g_common_obj.adb_cmd_common("shell pm list package | grep %s" % (package_name))
        ret = result.split("\n")
        for _ret in ret:
            if _ret == "package:%s" % package_name:
                print "%s app has been installed" % package_name
                return True
            else:
                return False

class GetInput():

    def __init__(self):
        self.wifi_apname = None
        self.wifi_password = None
        self.url_timeout = None
        self.google_account_username = None
        self.google_account_password = None
        self.url = None

    def get_input(self):
        ch = ConfigHandle()
        (apname, password) = ch.read_wifi_configuration()
        url = ch.read_content_url()
        print "\n\n"
        print "####################the default configuration are as below ##############################\n"
        print "ap_name" + "                  " + apname
        print "password" + "                  " + password
        print "download content url          " + url
        print "\n"
        print "Type y to verify, or Type n to reset"
        input_result = raw_input()
        if input_result == "n":
            while self.wifi_apname == None:
                print "\nplease input wifi apname"
                self.wifi_apname = raw_input()

            while self.wifi_password == None:
                print "\nplease input wifi password"
                sys.stdin.flush()
                self.wifi_password = raw_input()
            while self.url == None:
                print "\nplease input download mtbf-content.zip url"
                self.url = raw_input()

            self.check_information()

    def check_information(self):
        print "\n####################please check the information input###########################\n"
        print "wifi_apname: " + self.wifi_apname
        print "wifi_password: " + self.wifi_password
        print "download content url: " + self.url
        print "\n###################################################################################\n"
        print "Type y to verify, or Type n to reset"
        result = raw_input()
        if result == "n":
            self.wifi_apname = None
            self.wifi_password = None
            self.url = None
            self.get_input()
        if result == "y":
            config_handle = ConfigHandle()
            config_handle.write_wifi_configuration(self.wifi_apname, self.wifi_password)

    def check_google_account(self):
        ch = ConfigHandle()
        (self.google_account_username, self.google_account_password) = ch.read_google_account_configuration()
        print "\n####################please check google account information #######################\n"
        print "username: " + self.google_account_username
        print "password: " + self.google_account_password
        print "\n###################################################################################\n"
        print "Type y to verify, or Type n to reset"
        result = raw_input()
        if result == "n":
            self.google_account_username = None
            self.google_account_password = None
            while self.google_account_username == None:
                print "\nplease input google account"
                self.google_account_username = raw_input()
            while self.google_account_password == None:
                print "\nplease input password"
                self.google_account_password = raw_input()
            config_handle = ConfigHandle()
            config_handle.write_google_account_configuration(self.google_account_username, self.google_account_password)

