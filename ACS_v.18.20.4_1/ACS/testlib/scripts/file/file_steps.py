#!/usr/bin/env python

#######################################################################
#
# @filename:    file_steps.py
# @description: File operations test steps
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.base.base_step import step as base_step
from testlib.scripts.connections.local import local_steps
from testlib.scripts.connections.local.local_step import step as local_step

from testlib.scripts.file import file_utils

class compare_images(base_step):

    """ description:
            Compares the given files (similar to diff), but with a
            given <tolerance> against the root-mean-square difference
            of the two files

        usage:
            file_steps.compare_image(first_file = "file1.png",
                                     second_file = "file2.png",
                                     tolerance = 10)()

        tags:
            file, compare, rms, images
    """

    def __init__(self, first_file, second_file, tolerance = 0, **kwargs):
        self.first_file = first_file
        self.second_file = second_file
        self.diff = tolerance
        base_step.__init__(self, **kwargs)
        self.set_passm("Comparing {0} and {1} with {2} tolerance".format(first_file, second_file, self.diff))
        self.set_errorm("", "Comparing {0} and {1} with {2} tolerance".format(first_file, second_file, self.diff))

    def do(self):
        self.step_data = file_utils.rms_diff(self.first_file, self.second_file)

    def check_condition(self):
        return self.step_data <= self.diff


class create_file(local_step):

    """ description:
            Creates an empty file

        usage:
            file_steps.create_file(file_name = "FILE")()

        tags:
            file, random, create
    """

    def __init__(self, file_name, **kwargs):
        self.file_name = file_name
        local_step.__init__(self, **kwargs)
        self.set_passm("File {0} created".format(self.file_name))
        self.set_errorm("", "Cannot create file {0}".format(self.file_name))

    def do(self):
        local_steps.command(command = "touch " + self.file_name)()

    def check_condition(self):
        local_steps.command(command = "ls",
                                    stdout_grep = self.file_name)


class create_random_file(local_step):

    """ description:
            Creates a file of the given size with random data

        usage:
            file_steps.create_file(file_name = "FILE",
                                   size = 1024)()

        tags:
            file, random, create
    """

    def __init__(self, file_name, size, **kwargs):
        self.file_name = file_name
        self.size = size
        local_step.__init__(self, **kwargs)
        self.set_passm("File {0} created".format(self.file_name))
        self.set_errorm("", "Cannot create file {0}".format(self.file_name))

    def do(self):
        local_steps.command(command = "dd if=/dev/zero of=" + self.file_name
                            + " bs=" + str(self.size)
                            + " count=1")()

    def check_condition(self):
        local_steps.command(command = "ls",
                                    stdout_grep = self.file_name)


class remove_file(local_step):

    """ description:
            Removes the file given.

        usage:
            file_steps.create_file(file_name = "FILE",
                                   size = 1024)()

        tags:
            file, random, remove
    """

    def __init__(self, file_name, **kwargs):
        self.file_name = file_name
        local_step.__init__(self, **kwargs)
        self.set_passm("File {0} removed".format(self.file_name))
        self.set_errorm("", "Cannot remove file {0}".format(self.file_name))

    def do(self):
        local_steps.command(command = "rm -f " + self.file_name)()

    def check_condition(self):
        local_steps.command(command = "ls",
                                    stdout_not_grep = self.file_name)
