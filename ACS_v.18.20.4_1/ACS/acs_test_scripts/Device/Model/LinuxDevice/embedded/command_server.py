#!/usr/bin/env python

import sys
import os
import json
from subprocess import Popen, PIPE
import socket
import gobject
import dbus
import dbus.glib
import dbus.mainloop.glib

from watchdogs import EXPOSED_OBJECTS
from watchdog import WatchdogHandler
from command import CommandCatalog
from log import Logger
from decoration import *
import re

sys.path.append(os.path.dirname(__file__))


class CommandServer:
    """
    the main class for command server
    """
    HOST = ""
    PORT = 8888
    CONNECTIONS_NUMBER = 5

    def __init__(self):
        """
        constructor
        """
        self._log = Logger("Core")
        self._cmd_catalog = CommandCatalog()
        EXPOSED_OBJECTS["cmd_catalog"] = self._cmd_catalog
        EXPOSED_OBJECTS["CommandServer"] = self
        self._log.debug("Creating watchdogs")
        self._watchdog = WatchdogHandler()
        self._CONNECTIONS = {}

        self._sock = None
        self._conn = None
        self._cmd = None
        self._args = None
        self._kwargs = None

    def start(self):
        self._log.info("Running watchdogs")
        self._watchdog.start()
        try:
            while True:
                if CmdSrv.wait_connection():
                    CmdSrv.get_request()
                    CmdSrv.handle_request()
        except KeyboardInterrupt:
            self._log.debug("captured keyboard interrupt!")
        self._log.debug("closing socket")
        CmdSrv.close_socket()
        self._log.debug("socket closed")

    def create_connection_socket(self):
        """
        allows to create the connection socket
        :return: the socket
        :rtype: socket
        """
        self._log.debug("creating socket")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._log.debug("   binding socket to %s:%s" % (CommandServer.HOST, CommandServer.PORT))
            self._sock.bind((CommandServer.HOST, CommandServer.PORT))
            self._sock.settimeout(1)
            self._sock.listen(CommandServer.CONNECTIONS_NUMBER)
            self._log.debug("   Start listening on socket")
            self._sock.listen(5)
        except socket.error as msg:
            self._log.error(msg)
            self._log.error("   Command server exit (1)")
            sys.exit(1)
        return self._sock

    def close_socket(self):
        self._log.debug("closing socket")
        self._sock.close()

    def create_dbus_session(self):
        """
        initiates a DBUS session to allow inter process communication
        :return: None
        """
        self._log.debug("creating DBus session")
        p = Popen('dbus-launch', shell=True, stdout=PIPE, stderr=PIPE)
        out, _ = p.communicate()
        r = re.findall("([^=]*)=(.*)\n", out)
        with open("/tmp/bus.conf", "w") as f:
            json.dump(dict(r), f)
        gobject.threads_init()
        dbus.glib.init_threads()
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

    def wait_connection(self):
        """
        waits for incoming connections
        :return: None
        """
        try:
            self._conn, addr = self._sock.accept()
            self._conn.settimeout(None)
            self._log.debug("Connected with %s: %s" % (addr[0], str(addr[1])))
            return True
        except:
            return False

    def get_request(self):
        """
        once connection has been established, get the request.
         (i.e. the data transiting on the socket)
         the data is encapsulated in JSON
        :return: None
        """
        self._cmd, self._args, self._kwargs = json.loads(self._conn.recv(1024))
        self._log.debug("Running command: %s %s %s" % (self._cmd, self._args, self._kwargs))

    def handle_request(self):
        """
        handles the received
        :return: the executor or threaded_executor function result
        :rtype: Status
        """
        try:
            cmd = self._cmd_catalog.get(self._cmd)
            if cmd is not None:
                try:
                    cmd_impl = cmd.instanciate(self._args, self._kwargs)
                    result = cmd_impl.execute()
                    if result is not None and result.thread is not None:
                        # result is None in the particular case of join_executor
                        self._CONNECTIONS[result.output] = (result.thread, result.queue)
                except Exception as e:
                    self._log.debug(e)
                    result = Status(FAILURE, e.message)
            else:
                self._log.error("Unknown command: %s" % cmd)
                result = Status(FAILURE, "Error: unknown command")
            if result is not None:
                self._conn.sendall(json.dumps([result.status, result.output]))
        except Exception as e:
            self._log.debug("captured unexpected error")
            self._log.debug(e)
            self._conn.sendall(json.dumps([FAILURE, "captured unexpected error"]))

    def join_executor(self, pid):
        """
        allows to join a previously started thread
        :param pid: the thread ID
        :type pid: int
        :return: the threaded_executor function result
        :rtype: Status
        """
        def join_cmd(conn, proc, q):
            proc.join()
            output = q.get()
            conn.sendall(json.dumps(output))
            conn.close()

        self._log.debug("joining previous process")
        proc_info = self._CONNECTIONS[pid]
        proc, q = proc_info

        t = threading.Thread(target=join_cmd, args=(self._conn, proc, q))
        t.start()
        del self._CONNECTIONS[proc.ident]
        self._log.debug("End of join_cmd")
        return None

if __name__ == "__main__":
    CmdSrv = CommandServer()
    # server can communicate inter process using DBUS
    CmdSrv.create_dbus_session()
    # Create socket
    CmdSrv.create_connection_socket()
    CmdSrv.start()
    sys.exit(0)
