#!/usr/bin/env python

import sys
import os
import socket
import json
import time
from decoration import *
import argparse


class BtEdisonClient(argparse.ArgumentParser):

    SRV_IP = "192.168.2.15"
    DEFAULT_SOCK_TIMEOUT = 120
    CLIENT_FAILURE = "Failure"
    CHUNK_SIZE = 2048
    READ_TIME_INTERVAL = 0.1

    def __init__(self):
        """
        """
        argparse.ArgumentParser.__init__(self)
        self.cmd = ""
        self.args = []
        self.kwargs = {}
        self.add_argument("-s", "--addr", nargs=1)
        self.add_argument('cmd', nargs="?")
        self.add_argument('args', nargs=argparse.REMAINDER)

    def parse_args(self, args=None, namespace=None):
        """
        Parse options
        """
        usage = """{0} [-s <addr>] <command> [<args>]
        command is one of:
        - help [command]: gets from the server the list of available commands or the <command> help
        """.format(os.path.basename(__file__))
        obj = argparse.ArgumentParser.parse_args(self)
        if obj.addr is not None:
            BtEdisonClient.SRV_IP = obj.addr[0]
        self.cmd = obj.cmd
        if obj.args is None:
            obj.args = []
        for arg in obj.args:
            if "=" in arg:
                k, v = arg.split("=")
                self.kwargs[k] = v
            else:
                self.args.append(arg)
        if self.cmd is None:
            print usage
            sys.exit(0)

    def _recv_timeout(self, s, timeout=0.1):
        """
        Read data from socket with timeout

        :type s: socket
        :param s: socket to communicate with embedded server
        :type timeout: float
        :param timeout: reading timeout in seconds

        :rtype: string
        :return: data from embedded command server
        """
        # Make socket non blocking
        s.setblocking(0)

        chunks = []
        chunk = ''

        # Beginning time
        begin = time.time()

        while True:
            # If something has been read, break after timeout
            if chunks != [] and (time.time() - begin > timeout):
                break

            # Read something
            try:
                chunk = s.recv(self.CHUNK_SIZE)
                if chunk:
                    chunks.append(chunk)
                    # Update the beginning time for measurement
                    begin = time.time()
                else:
                    # Sleep for sometime before reading again
                    time.sleep(self.READ_TIME_INTERVAL)
            except:
                pass

        # Join all chunks to make final string
        return ''.join(chunks)

    def run_cmd(self):
        """
        """
        status = BtEdisonClient.CLIENT_FAILURE
        output = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(BtEdisonClient.DEFAULT_SOCK_TIMEOUT)
            s.connect((BtEdisonClient.SRV_IP, 8888))
            s.sendall(json.dumps([self.cmd, self.args, self.kwargs]))
            status, output = json.loads(self._recv_timeout(s))
        except socket.error as SE:
            print >> sys.stderr, SE
        except Exception as E:
            print >> sys.stderr, E
        s.close()
        return status, output

if __name__ == '__main__':
    client = BtEdisonClient()

    client.parse_args()
    if client.cmd is not None:
        req = BtEdisonClient()
        req.cmd = "get_args"
        req.args = [client.cmd]
        status, output = req.run_cmd()
        try:
            if client.kwargs:
                for k in output:
                    if k in client.kwargs:
                        #classic formatting: args is a dict
                        client.kwargs[k] = eval(output[k])(client.kwargs[k])
            elif len(client.args) > 0:
                for ind, k in enumerate(output.values()):
                    # args is a list
                    client.args[ind] = eval(k)(client.args[ind])
            status, output = client.run_cmd()
        except Exception as e:
            print "captured exception", e
        print output
        if status == SUCCESS:
            sys.exit(0)
        else:
            sys.exit(-1)
