#!/usr/bin/env python


class Connection(object):

    def __init__(self):
        pass

    def open_connection(self):
        raise NotImplementedError("Method not overwritten")

    def close_connection(self):
        raise NotImplementedError("Method not overwritten")

    def get_file(self, remote, local):
        raise NotImplementedError("Method not overwritten")

    def put_file(self, local, remote):
        raise NotImplementedError("Method not overwritten")

    def run_cmd(self, command, mode="synch"):
        raise NotImplementedError("Method not overwritten")

    def kill_command(self, pid):
        raise NotImplementedError("Method not overwritten")

    def kill_all(self, pids):
        raise NotImplementedError("Method not overwritten")

    def cd(self, path):
        raise NotImplementedError("Method not overwritten")

    def set_env(self, var_name, var_value):
        raise NotImplementedError("Method not overwritten")

    def unset_env(self, var_name):
        raise NotImplementedError("Method not overwritten")

if __name__ == "__main__":
    pass
