#!/usr/bin/env python

import dbus
from bt.constants import Constants as Consts
from decoration import *
import time
import os
import socket
import threading
from handler import ProfileHandler
import subprocess
import shlex


class BtSppProfile(ProfileHandler):
    """
    Class to handle Serial Port Profile
    """

    _sock = {}
    _timeout = 60
    _result_key = None

    _opts = {
        "AutoConnect": False,
        "RequireAuthorization": False,
        "PSM": dbus.UInt16("3"),
        "Channel": dbus.UInt16("22"),
        "Name": "Edison SPP test",
        "Role": "server",
    }

    __results = {
        "connection": None,
        "download": None,
        "upload": None
    }

    _uuid = "1101"
    _role = "client"
    _result = None
    _fd = {}
    _data = {}
    _dev_path = None

    def __init__(self, *args, **kwargs):
        """
        initialize the object
        """
        ProfileHandler.__init__(self, dbus.SystemBus(), "/Intel/Edison/spp_profile", "1101", self._opts)

    @dbus.service.method(Consts.Bt.PROFILE_IFACE, in_signature="oha{sv}", out_signature="")
    def NewConnection(self, path, fd, properties):
        """
        actions performed when a new connection is incoming
        it redefines the dbus.service.method NewConnection

        basically, it listens to data received from the connected remote peer.
        As data listening is threated, several connections can be handled in parallel
        """
        addr = os.path.basename(path).replace("dev_", "").replace("_", ":")
        self._log.debug("Client connection")
        try:
            self._fd[addr] = fd.take()
            time.sleep(3)
            self._log.debug("Client connected({0}, {1})".format(path, self._fd[addr]))
            self._dev_path = path
            thrd = threading.Thread(target=self.listen_socket, kwargs={"addr": addr})
            thrd.start()
        except Exception as e:
            self._log.error(e)

    def listen_socket(self, addr, *args, **kwargs):
        """
        listens to the incoming data on the socket dedicated to a connection
        """
        sock = socket.fromfd(self._fd[addr], socket.AF_UNIX, socket.SOCK_STREAM)
        sock.setblocking(1)
        el_time = 0
        try:
            self._data[addr] = {"data": sock.recv(1)}
            sock.settimeout(1)
            el_time = time.time()
            while True:
                self._data[addr] += sock.recv(1024)
        except IOError:
            self._log.info("Closing connection with %s" % addr)
            el_time = time.time() - el_time - 1
            val, unit = self._compute_throughput(len(self._data[addr]) - 1, el_time)
            self._data[addr]["throughput"] = [val, unit]
            self._log.debug(self._data[addr])
        except Exception as e:
            self._log.error(e)
        finally:
            sock.close()
            del self._fd[addr]
            self._log.info("connection closed")

    @executor
    def get_data(self):
        """
        allows to get the data associated to a connection
        that's to say data and throughput
        """
        return SUCCESS, self._data

    @threaded_executor
    def send(self, addr=None, cmd=""):
        """
        allows to send data to other end of connection, either client or remote
        """
        if os.path.exists(cmd):
            self._log.debug("command is a file")
            f_size = os.stat(cmd).st_size
            with open(cmd, 'rb') as f:
                cmd = f.read()
        else:
            self._log.debug("command is a string: '%s'" % cmd)
            f_size = len(cmd)
        if self._role == "client":
            result = self.__client_write(addr, cmd, f_size)
        else:
            result = self.__server_write(addr, cmd, f_size)
        return result

    def __write_all_sockets(self, cmd):
        """
        server sends data to all connected clients
        """
        for fd in self._fd.values():
            os.write(fd, cmd)

    def __client_write(self, addr, cmd, size):
        """
        client writes out to server
        """
        status, output = self._connect_to_server(addr)
        if status == SUCCESS:
            self._log.debug("client send to server")
        else:
            self._log.error("error sending")
            return FAILURE, "error sending"
        f = os.open("/dev/rfcomm0", os.O_WRONLY)
        self._log.debug("transferring size %i" % size)
        try:
            os.write(f, str(size))
        except Exception as e:
            self._log.debug(e)
        os.close(f)
        with open("/dev/rfcomm0", "wb") as f:
            self._log.debug("transferring data")
            elapsed_time = time.time()
            f.write(cmd)
            elapsed_time = time.time() - elapsed_time
            throughput = self._compute_throughput(size, elapsed_time)
        self._p.terminate()
        return SUCCESS, throughput

    def __server_write(self, addr, cmd, size):
        """
        server writes out to client(s)
        """
        throughput = None
        try:
            if addr in self._fd:
                self._log.debug("send to address %s" % addr)
                # self._sock[addr].send(size)
                os.write(self._fd[addr], str(size))
                elapsed_time = time.time()
                # self._sock[addr].send(cmd)
                os.write(self._fd[addr], cmd)
                elapsed_time = time.time() - elapsed_time
                throughput = self._compute_throughput(size, elapsed_time)
            else:
                self._log.debug("send to all addresses")
                self.__write_all_sockets(str(size))
                elapsed_time = time.time()
                self.__write_all_sockets(cmd)
                elapsed_time = time.time() - elapsed_time
                throughput = self._compute_throughput(size * len(self._fd), elapsed_time)
        except Exception as e:
            self._log.error(e)
        return SUCCESS, throughput

    def _compute_throughput(self, f_size, elapsed_time):
        """
        computes the throughput
        """
        self._log.debug("size = {0}, time = {1}".format(f_size, elapsed_time))
        units = ["B", "kB", "MB"]
        f_s = f_size
        it = -1
        while f_s >= 1:
            f_size = f_s
            f_s = f_size / 1024
            it += 1
        self._log.debug("troughput is %.2f %s/s" % (f_size / elapsed_time, units[it]))
        return f_size / elapsed_time, units[it]

    @threaded_executor
    def receive(self, addr, sock=None):
        """
        allows to set the client in waiting for data mode
        connection is closed once client has received data
        """
        size = 0
        if self._role == "client":
            status, result = self._connect_to_server(addr)
            if status == SUCCESS:
                self._log.info("waiting incoming data size")
                try:
                    fd = os.open("/dev/rfcomm0", os.O_RDONLY)
                    size = int(os.read(fd, 1024))
                    self._log.info("got size %i" % size)
                    os.close(fd)
                except Exception as e:
                    self._log.error("file descriptor manipulation error: %s" % e.message)
                self._log.info("waiting incoming data")
                try:
                    with open("/dev/rfcomm0", "rb") as sock:
                        data = sock.read(size)
                        self._log.debug(data)
                except Exception as e:
                    self._log.error("file manipulation error: %s" % e.message)
                self._log.info("receive ended")
                sock.close()
                self._p.terminate()
            else:
                self._log.info("failure")
        else:
            self._log.info("connection closed")
            status = FAILURE
            result = "unneeded call for server"
        return status, result

    def _connect_to_server(self, addr, retry=3):
        """
        function that handles the connection of the client to the server
        """
        count = 0
        status = FAILURE
        result = ""
        while status == FAILURE and count < retry:
            self._log.debug("/usr/bin/rfcomm connect 0 %s 22" % addr)
            self._p = subprocess.Popen(shlex.split("/usr/bin/rfcomm connect 0 %s 22" % addr),
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            while self._p.poll() is None:
                if os.path.exists("/dev/rfcomm0"):
                    self._log.debug("success")
                    status = SUCCESS
                    result = "connection established"
                    break
                time.sleep(1)
            else:
                out, result = self._p.communicate()
                status = FAILURE
            count += 1
            self._log.info(result)
        return status, result

    @executor
    def stop(self):
        """
        allows to stop the server
        """
        for fd in self._fd:
            os.close(fd)
        self._role = "client"
        return self.unregister()

    @executor
    def start(self):
        """
        allows to start the server
        """
        self._role = "server"
        return self.register()
