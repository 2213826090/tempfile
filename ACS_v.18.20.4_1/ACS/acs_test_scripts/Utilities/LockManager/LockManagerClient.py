from UtilitiesFWK.Utilities import format_cmd_args
from Core.PathManager import Folders
from ErrorHandling.AcsToolException import AcsToolException
import subprocess
import psutil
import os
import sys
import time
import xmlrpclib

ON_POSIX = 'posix' in sys.builtin_module_names


class LockEquipementManagerClient():
    RCP_SERVER_IP = 'localhost'
    RCP_SERVER_PORT = 9000

    def __init__(self, server_ip, port, logger):
        self._server_ip = server_ip
        self._server_port = port
        self.client_proxy = None
        self._logger = logger

    def ping(self, retry=3):
        """
        Sends a simple request to the underlying I{XML RPC server} instance.

        :type retry: int
        :param retry: the maximum number of retries when a command fails.

        :rtype: bool
        :return: the result of the 'ping' command as boolean.
        """
        # We must not check whether a server is running
        self.client_proxy = self.get_server_proxy()
        ping_result = False
        current_retry = 0
        shall_retry = True
        while current_retry < retry and shall_retry:
            try:
                # is a server already started ?
                ping_result = self.client_proxy.ping()
            except (KeyboardInterrupt, SystemExit):
                raise
            except:
                # We want to catch any error or exception
                ping_result = False
            finally:
                current_retry += 1
        return ping_result

    def get_logger(self):
        """
        Gets the internal logger of the equipment
        """
        return self._logger

    def get_server_proxy(self):
        """
        Returns the XML RPC proxy server instance to use
        in order to read/write commands.

        @rtype: xmlrpclib.ServerProxy
        @return: a proxy to the XML RPC server.
        """
        if self.client_proxy is None:
            self.client_proxy = xmlrpclib.ServerProxy('http://' + self._server_ip + ':' + str(self._server_port))
        return self.client_proxy

    def _is_port_used(self, port):
        """
        Check is port is already used by a process
        """
        is_used = False
        try:
            is_used = not all([False for c in psutil.net_connections() if c.laddr[1] == port])
        except AttributeError:
            self._logger.warning("Please, update the version of psutil python module")
            self._logger.warning("Not able to verify if the adb server port is available")
        return is_used

    def check_rpc_server_running(self):
        """
        Check that a XML RPC server instance is listening for XML
        RPC requests, if not, starts one.
        """
        ping_status = self.ping()
        if not ping_status:
            if self._is_port_used(self._server_port):
                raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                       "%s already in use, select another port" % self._server_port)
            self.get_logger().debug("Starting XML-RPC server on port %s" % str(self._server_port))
            # Does not work
            # d = multiprocessing.Process(name='EquipementServerXMLRPC', target=start_server, args=(self._server_ip, self._server_port,))
            # d.daemon = True
            # d.start()
            # So, use an uggly way to run the server process
            log_folder = os.path.join(Folders.ACS_CACHE)
            cmd = "python LockManagerServer.py %s %s %s" % (self._server_ip, self._server_port, log_folder)
            DEVNULL = open(os.devnull, 'wb')
            subprocess.Popen(format_cmd_args(cmd),
                             shell=False,
                             stdout=DEVNULL,
                             stderr=subprocess.STDOUT,
                             cwd=os.path.abspath(os.path.dirname(__file__)),
                             close_fds=ON_POSIX)
            # Wait some time in order to give a chance
            # to the server to complete its starting procedure
            time.sleep(3)

    def wait_resource_is_ready(self, resource, info):
        """
        Wait until specified resource is ready

        :type resource: str
        :param resource: the name of the resource we are waiting for.

        :type info: tuple
        :param info: info about client (pid, timeout)
        """
        # get proxy instance
        self.check_rpc_server_running()
        # add lock on the resource
        self.client_proxy.request_resource_access(resource, info)
        # do a clean if dead process are still on the queue
        self.client_proxy.clean(resource)
        # calculate the max expected time according to other process timeout
        max_pending_time = self.client_proxy.get_max_pending_time(resource, info)
        if max_pending_time:
            self.get_logger().info("Max pending time before %s release : %s" % (resource, max_pending_time))

        begin_time = time.time()
        end_time = begin_time + float(max_pending_time)
        exec_time = begin_time
        empty_output_time = begin_time
        access_granted = self.client_proxy.access_granted_on_resource(resource, info)
        while exec_time < end_time and not access_granted:
            # if no output for 5 minutes, print an info
            if int(time.time() - empty_output_time) > 300:
                empty_output_time = time.time()
                self.get_logger().info("Waiting for %s for %.2f seconds" % (resource, (time.time() - begin_time)))
                # wait for free resource
            # do we have access now ?
            self.client_proxy.clean(resource)
            access_granted = self.client_proxy.access_granted_on_resource(resource, info)
            # wait 1s before next attempt
            time.sleep(1)
            exec_time = time.time()

        if not access_granted:
            raise AcsToolException(AcsToolException.HOST_OPERATION_TIMEOUT,
                                   "Cannot get the resource after %s seconds" % max_pending_time)
        else:
            self.get_logger().info("Access granted to %s" % (resource))

    def exec_function_on_resource(self, resource, timeout, func, arg):
        """
        Exec a function which require a shared resource with other process.

        :type resource: str
        :param resource: the name of the resource we are waiting for.

        :type timeout: int
        :param timeout: exec function timeout.

        :type func: callable
        :param func: the function to execute as soon as the resource is available.

        :type arg: tuple
        :param arg: arguments to pass to the function
        """
        return_result = None
        my_pid = os.getpid()
        info = (my_pid, timeout)
        try:
            # wait resource is available
            self.wait_resource_is_ready(resource, info)
            return_result = func(*arg)
        finally:
            if self.client_proxy is not None and self.ping():
                self.client_proxy.remove_request_resource(resource, info)
        return return_result

if __name__ == "__main__":
    import logging
    logging.basicConfig()
    ACSEquipementLogger = logging.getLogger("ACS.EQUIPEMENT_CLIENT")
    ACSEquipementLogger.setLevel(logging.DEBUG)
    flash_eq_client = LockEquipementManagerClient('localhost', 8008, ACSEquipementLogger)
    resource = 'FLASH'
    timeout = 90

    def my_sleep(sleep_time=10):
        print "sleep %s".format(sleep_time)
        time.sleep(sleep_time)
        print "sleep %s".format(sleep_time)
        time.sleep(sleep_time)
        print "end"
        return "OK"
    result = flash_eq_client.exec_function_on_resource(resource, timeout, my_sleep,
                                                       (10,))
    sys.exit(0)
