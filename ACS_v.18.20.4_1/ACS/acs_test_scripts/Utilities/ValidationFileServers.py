"""
:copyright: (c)Copyright Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:summary: Contains classes, functions, and other objects for creating file servers.
"""

import BaseHTTPServer
import logging
import os
import posixpath
import select
import SimpleHTTPServer
import socket
import thread
import urllib

"""
The ValidationHTTPServer improves upon the SimpleHTTPServer by adding these features:
  1. Ability to set the document root. Use .set_document_root()
  2. Ability to specify where to log messages. Can log to file path, file, or logger object. Use .set_logger().
  3. Ability to use a run file to specify to kill the server by removing the file. Use .set_run_file().
  4. An easy function for starting the server in the background. Use .serve_in_background().

The easiest way to start an HTTP server is to use the function:
    start_http_server(document_root=None, run_file=None, port=None, logger=None)
"""
class ValidationFileServer:
    """A file server class should have an implementation similar to this class. This will allow
    tests to easily swap one type of file server for another. For example, HTTP, FTP, etc."""
    def set_document_root(self, directory_path):
        raise Exception("Not Implemented")
    def start_server(self):
        raise Exception("Not Implemented")
    def set_logger(self, logging_obj):
        raise Exception("Not Implemented")
    def set_run_file(self, run_file_path):
        raise Exception("Not Implemented")

class ValidationHTTPRequestHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    """This class is an extension of the SimpleHTTPRequestHandler to add these features:
         1. Ability to set the document root. Use .set_document_root()
         2. Ability to specify where to log messages. Can log to file path, file, or logger object. Use .set_logger().
    """
    def __init__(self, request, client_address, server, document_root=None, logger=None):
        self.document_root = document_root
        self.logger = logger
        SimpleHTTPServer.SimpleHTTPRequestHandler.__init__(self, request, client_address, server)
    def set_document_root(self, directory_path):
        """Set the relative location of the files to be served. For example, a file directory_path/myfile.txt
        can be accessed with the URL http://server_domain/myfile.txt."""
        self.document_root = directory_path
    def translate_path(self, path):
        # Overriden from SimpleHTTPServer.SimpleHTTPRequestHandler.translate_path
        path = path.split('?',1)[0]
        path = path.split('#',1)[0]
        path = posixpath.normpath(urllib.unquote(path))
        words = path.split('/')
        words = filter(None, words)
        # This function is identical to SimpletHTTPServer.SimpleHTTPRequestHandler.translate_path, except for the following code
        #path = os.getcwd()
        if self.document_root == None:
            path = os.getcwd()
        else:
            path = self.document_root
        # END modifications
        for word in words:
            drive, word = os.path.splitdrive(word)
            head, word = os.path.split(word)
            if word in (os.curdir, os.pardir): continue
            path = os.path.join(path, word)
        return path
    def log_message(self, format, *args):
        """This function is overriden and extended to write the log message to a file."""
        log_string = "%s - - [%s] %s\n"%(self.address_string(), self.log_date_time_string(), format%args)
        if isinstance(self.logger, file):
            self.logger.write(log_string+"\r\n")
        elif isinstance(self.logger, logging.Logger):
            self.logger.info(log_string)
        elif isinstance(self.logger, str) and os.path.exists(self.logger):
            log_file = open(self.logger, 'a')
            log_file.write(log_string+"\r\n")
            log_file.close()
        else:
            sys.stderr.write(log_string)

class ValidationHTTPServer(BaseHTTPServer.HTTPServer):
    run_file = None
    document_root = None
    logger = None
    def __init__(self, server_address, RequestHandlerClass=ValidationHTTPRequestHandler):
        # Overriden to prevent the server from binding at creation. We will wait to bind until the server is started.
        BaseHTTPServer.HTTPServer.__init__(self, server_address, RequestHandlerClass, bind_and_activate=False)
    def set_run_file(self, run_file_path):
        """A run file can be used to trigger the server to shutdown. As long as the run 
        file exists, the server will continue to run after a call to serve_forever()."""
        self.run_file = run_file_path
    def set_document_root(self, directory_path):
        """Set the relative location of the files to be served. For example, a file directory_path/myfile.txt
        can be accessed with the URL http://server_domain/myfile.txt."""
        self.document_root = directory_path
    def set_logger(self, logging_obj):
        """Set the logger to be used for the server. Possible types can be a string representing the path
        to a file, a file object, or a logger object."""
        self.logger = logging_obj
    def finish_request(self, request, client_address):
        # Overriden from BaseHTTPServer.HTTPServer.finish_request. This method must be overriden to allow us
        #  to pass the document root and logger object to the request handler.
        self.RequestHandlerClass(request, client_address, self, self.document_root, self.logger)
    def start_server(self):
        """Easy function for starting the server. Calls serve_in_background()"""
        self.serve_in_background()
    def serve_in_background(self):
        """Starts the server in a new thread."""
        thread.start_new_thread(self.serve_forever, ())
    def serve_forever(self, poll_interval=0.5):
        # Overriden from BaseHTTPServer.HTTPServer.serve_forever and identical, except for the 
        #  commented lines and lines with # ADDED
        try:
            self._BaseServer__is_shut_down.clear()
            self.server_bind() # ADDED
            self.server_activate() # ADDED
            #while not self.__shutdown_request:
            while not self._BaseServer__shutdown_request and (self.run_file==None or os.path.exists(self.run_file)): # ADDED
                # XXX: Consider using another file descriptor or
                # connecting to the socket to wake this up instead of
                # polling. Polling reduces our responsiveness to a
                # shutdown request and wastes cpu at all other times.
                r, w, e = select.select([self], [], [], poll_interval)
                if self in r:
                    self._handle_request_noblock()
        except:
            import traceback
            traceback.print_exc()
        finally:
            self._BaseServer__shutdown_request = False
            self._BaseServer__is_shut_down.set()
        self.server_close() # Takes care of closing the socket # ADDED
        if isinstance(self.logger, logging.Logger):
            for handler in self.logger.handlers:
                try:
                    handler.close()
                except:
                    pass
        elif isinstance(self.logger, file):
            self.logger.close()

def start_http_server(document_root=None, run_file=None, port=None, logger=None):
    """An easy function to start an HTTP server in one line."""
    if port == None:
        port = get_unused_port()
    http_server = ValidationHTTPServer( server_address=('', port), RequestHandlerClass=ValidationHTTPRequestHandler)
    http_server.set_document_root(directory_path=document_root)
    http_server.set_logger(logger)
    if run_file != None:
        http_server.set_run_file(run_file_path=run_file)
        # Write the port to the run file
        f = open(run_file, 'w')
        f.write(str(port))
        f.close()
    http_server.start_server()
    return http_server

def get_unused_port(starting_port=23987, max_attempts=100): # Randomly chosen starting number
    """Attempts to bind to ports until it finds an unused port. Returns the port number."""
    while max_attempts>0:
        try:
            tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_socket.bind(('', starting_port))
            tcp_socket.close()
            return starting_port
        except:
            import traceback
            traceback.print_exc()
        max_attempts -= 1
        starting_port += 1
    raise Exception("Unable to find a free port to bind to.")