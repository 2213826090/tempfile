#!/usr/bin/env python

import telnetlib
from testlib.utils.connections.connection import *

class Telnet(Connection):
    pids = []
    host = ""
    prev_path = curr_path = ""

    def __init__(self, host, user, password):
        super(Telnet, self).__init__()
        self.host = host
        self.user = user
        self.password = password
        self.prompt = self.user + '@'
        self.prev_path = self.curr_path = "/home/{0}".format(user)
        self.telnet_open = False

    def open_connection(self):
        if not self.telnet_open:
            self.telnet = telnetlib.Telnet(self.host)

            # send the login user
            self.telnet.read_until('login: ')
            self.telnet.write(self.user + "\n")

            # senf the login password
            self.telnet.read_until("Password: ")
            self.telnet.write(self.password + "\n")
            self.telnet.read_until(self.prompt)

            self.telnet_open = True


    def close_connection(self):
        self.telnet.close()
        self.telnet_open = False



    def get_file(self, remote, local):
        # TODO:
        pass


    def put_file(self, local, remote):
        # TODO:
        pass

    def run_cmd(self, command, mode="sync", timeout=None):
        self.telnet.write(command + "\n")
        return self.telnet.read_until(self.prompt)


