from SimpleXMLRPCServer import SimpleXMLRPCServer
from weakref import proxy
import logging
import multiprocessing
import os
import psutil
import sys


class EquipmentServices(object):

    """
    This class implements the server methods.
    Used to avoid recursion when registering an instance.
    """

    def __init__(self, calling_server):
        """
        Initializes this instance.

        :type calling_server: LockEquipementManagerServer
        :param calling_server: the XML RPC instance that will use this object's methods
        """
        self.__calling_server = proxy(calling_server)

    @property
    def locked_resources(self):
        return self.__calling_server.resources_locked

    def ping(self):
        """
        Ping the server

        :rtype: bool
        :return: True
        """
        self.__calling_server.get_logger().debug("Ping - Pong!")
        return True

    def get_server_pid(self):
        """
        Get the server process id

        :rtype: int
        :return: the server process id
        """
        self.__calling_server.get_logger().debug("Server pid %s" % (self.__calling_server._server_pid))
        return self.__calling_server._server_pid

    def request_resource_access(self, resource, info):
        """
        Request access to a resource

        :type resource: str
        :param resource: the name of resource to free.

        :type info: tuple
        :param info: info about client which request access to the resource (pid, timeout)

        :rtype: bool
        :return: True
        """
        if not resource in self.locked_resources:
            self.locked_resources[resource] = []
        current_lock = self.locked_resources[resource]
        # Need to be reassigned due to multiprocessing behavior
        # From multiprocessing doc:
        # Modifications to mutable values or items in dict and list proxies will not be propagated
        # through the manager, because the proxy has no way of knowing when its values or items are modified.
        # To modify such an item, you can re-assign the modified object to the container proxy
        current_lock.append(info)
        self.locked_resources[resource] = current_lock
        self.__calling_server.get_logger().debug("Add lock on %s by %s" % (resource, info))
        return True

    def remove_request_resource(self, resource, info):
        """
        Remove a process lock on a resource.

        :type resource: str
        :param resource: the name of resource to free.

        :type info: tuple
        :param info: info about client to free on the resource (pid, timeout)

        :rtype: bool
        :return: True
        """
        pid = info[0]
        if resource in self.locked_resources:
            old_infos = list(self.locked_resources[resource])
            self.locked_resources[resource] = [old_info for old_info in old_infos if old_info[0] != pid]
            self.__calling_server.get_logger().debug("Remove lock on %s of %s" % (resource, info))
            self.__calling_server.get_logger().debug("After removing lock")
            self.show_resources_locked()
        else:
            self.__calling_server.get_logger().error("resource %s does not exist!" % (resource))

        return True

    def access_granted_on_resource(self, resource, info):
        """
        Check if the access is granted on a resource

        :type resource: str
        :param resource: the name of resource to free.

        :type info: tuple
        :param info: info about client whcih request access to the resource (pid, timeout)

        :rtype: bool
        :return: resource is available or not
        """
        is_free = False
        pid = info[0]
        self.__calling_server.get_logger().debug("PID %s asks granted access on %s" % (pid, resource))
        resource_info = list(self.locked_resources.get(resource, [info]))
        if resource_info:
            self.__calling_server.get_logger().debug("%s resource locked by %s" % (resource, resource_info))
            is_free = resource_info[0][0] == pid
        else:
            self.__calling_server.get_logger().debug("%s resource is not locked by any process" % (resource))
            is_free = True
        return is_free

    def get_max_pending_time(self, resource, info):
        """
        Get the estimate max waiting time before resource access

        :type resource: str
        :param resource: the name of resource to free.

        :type info: tuple
        :param info: info about client which request access to the resource (pid, timeout)

        :rtype: bool
        :return: True
        """
        max_pending_time = 0
        # calculate the max pending time
        if self.locked_resources[resource]:
            for _, timeout in self.locked_resources[resource][:-1]:
                max_pending_time += timeout
        return max_pending_time

    def show_resources_locked(self):
        """
        Get the locked resources dictionary

        :rtype: dict
        :return: every process which request a lock on a resource
        """
        self.__calling_server.get_logger().info("Locked resource %s" % self.locked_resources)

    def clean(self, resource):
        """
        Remove all dead process on a specific resource

        :type resource: str
        :param resource: the name of resource to clean

        :rtype: bool
        :return: True
        """
        current_pids = psutil.pids()
        self.__calling_server.get_logger().info("Clean on %s" % resource)
        if resource in self.locked_resources:
            old_pids = list(self.locked_resources[resource])
            self.__calling_server.get_logger().debug("Before cleanup")
            self.show_resources_locked()
            self.locked_resources[resource] = [info_pid for info_pid in old_pids if info_pid[0] in current_pids]
            self.__calling_server.get_logger().debug("After cleanup")
            self.show_resources_locked()
        else:
            self.__calling_server.get_logger().debug("Nothing to clean on %s" % resource)
        return True

    def clean_all(self):
        """
        Remove all dead process for all resources

        :rtype: bool
        :return: True
        """
        for resource in self.__calling_server.resources_locked:
            self.__calling_server.clean(resource)
        return True

    def hard_reset(self):
        """
        Clear all locks on all resources

        :rtype: bool
        :return: True
        """
        self.locked_resources.clear()
        return True


class LockEquipementManagerServer(SimpleXMLRPCServer):

    def __init__(self, server_ip, server_port, log_folder):
        """
        Initializes this instance.

        :type server_ip: str
        :param server_ip: the IP address to be used by this instance

        :type server_port: int
        :param server_port: this server instance port number

        :type logger: logger
        :param logger: logger to use
        """
        # Call superclass initialization
        SimpleXMLRPCServer.__init__(self, (server_ip, server_port), logRequests=True, allow_none=True)
        mgr = multiprocessing.Manager()
        log_filename = "log_lockManagerServer_%s_port_%s.log" % (server_ip.replace('.', '-'), server_port)
        log_file = os.path.join(log_folder, log_filename)

        # The logger instance to use
        logging.basicConfig()
        self.__logger = logging.getLogger("ACS.EQUIPEMENT_SERVER")
        self.__logger.setLevel(logging.DEBUG)
        fh = logging.FileHandler(log_file)
        fh.setLevel(logging.DEBUG)
        fh_format = logging.Formatter('%(asctime)s - %(levelname)-8s - %(message)s')
        fh.setFormatter(fh_format)
        self.__logger.addHandler(fh)

        # Store server pid
        self._server_pid = os.getpid()

        # Register an instance offering the required methods
        self.register_instance(EquipmentServices(self))
        self.resources_locked = mgr.dict()
        self.__logger.info("Start %s on %s:%s" % (self.__class__.__name__, server_ip, server_port))

    def start(self):
        self.__is_running = True
        while self.__is_running:
            self.handle_request()

    def stop(self):
        """
        Stops this server instance.
        """
        self.__is_running = False
        self.server_close()

    def get_logger(self):
        return self.__logger


def start_server(ip, port, log_folder):
    server = LockEquipementManagerServer(ip, port, log_folder)
    server.start()

if __name__ == "__main__":
    start_server(sys.argv[1], int(sys.argv[2]), sys.argv[3])
