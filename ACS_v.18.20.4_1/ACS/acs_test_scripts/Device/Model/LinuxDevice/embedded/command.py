import importlib
import pkgutil
import commands
from watchdogs import EXPOSED_OBJECTS
from decoration import *
from log import Logger


class CommandCatalog(object):
    """
    this class represents the commands' catalog for the command server
    the catalog is built dynamically from the content of the folder 'commands'
    for 1 command the following dictionary structure is required
    <command name (str)>:
        {"description": <a brief description of the command (str)>,
         "function": {"_module": <the dotted path to the module containing the command's executed function (str)>,
                      "_class": <the name of the class which function is executed (str)>,
                      "_name": <the function name(s) (str or list),
                      "_args": <a name - type mapping of all required arguments to either the class constructor or the
                               function (dict). for example {"name": str}>,
         "requires": <a list of required objects which are statically instanciated.
                      This object must be stored at watchdog instanciation in EXPORTED_OBJECTS dictionary (list)>},
    """
    def __init__(self):
        """
        initializes the command catalog
        """
        self._catalog = {}
        self._features = {}
        self._log = Logger("CMD")
        Command.LOG = self._log
        self.import_commands()

    def get(self, cmd):
        """
        allows to get the Command object corresponding to input string
        :param cmd: the command string
        :type cmd: str
        :return: the command object
        :rtype: Command
        """
        # self._log.debug(self._catalog)
        if cmd in self._catalog:
            return self._catalog[cmd]
        return None

    @executor
    def help(self, cmd=None):
        """
        displays help
        :param cmd: optional, the command for which help is required
        :type cmd: str
        :return: the help message
        :rtype: list or dict
        """
        if cmd is None:
            return self._help_msg()
        else:
            return self._command_help_msg(cmd)

    @executor
    def get_args(self, cmd):
        cmd = self.get(cmd)
        d = {}
        for key, val in cmd.kwargs.items():
            d[key] = val.__name__
        return SUCCESS, d

    def import_commands(self):
        """
        dynamically imports the commands' catalogs
        :return: None
        """
        for _, module, _ in pkgutil.walk_packages(commands.__path__):
            # load the module
            m = importlib.import_module("commands.%s" % module)
            self._features[module] = []
            for a in dir(m):
                if not a.startswith("__"):
                    self._extract_commands(module, m.__dict__[a])

    def _extract_commands(self, feature, cmd_dict):
        """
        extract the commands from the dictionaries and insert them into the catalog
        :param feature: the feature aka the name of the module containing the dictionary
        :type feature: str
        :param cmd_dict: the command dictionary
        :type cmd_dict: dict
        :return: None
        """
        for name, data in cmd_dict.items():
            if name in self._catalog:
                self._log.warning("%s command already exists. Replace oldest." % name)
            fct = dict(data["function"])
            del data["function"]
            data.update(fct)
            self._catalog[name] = Command(name=name, **data)
            self._features[feature].append(name)

    def _help_msg(self):
        """
        displays help message as list of available commands and their description

        :rtype dict
        :returns the help message
        """
        cmd_list = {}
        for module, commands in self._features.items():
            inner = {}
            for cmd in commands:
                inner[cmd] = self._catalog[cmd].description
            cmd_list[module] = inner
        return SUCCESS, cmd_list

    def _command_help_msg(self, cmd):
        """
        displays help message for a particular command including description and arguments
        :type cmd: str
        :param cmd: the command

        :rtype dict
        :returns the help message
        """
        if cmd in self._catalog:
            command = self._catalog[cmd]
            args = dict(command.kwargs)
            for k, v in args.items():
                args[k] = v.__name__
            msg = {"Description": command.description,
                   "Required arguments": args}
            return SUCCESS, {cmd: msg}
        return FAILURE, "unknown command: %s" % cmd


class Command(object):
    """
    class representing a command as stored in commands' dictionary
    """

    LOG = None

    def __init__(self, name, description, _name, _module, _class, _args, requires=None):
        """
        constructor
        """
        self.name = name
        self.description = description
        self.fct_module = _module
        if type(_name).__name__ == "list":
            self.fct_name = _name
        else:
            self.fct_name = [_name]
        self.fct_class = _class
        self.kwargs = _args
        self.requires = requires

    def instanciate(self, args, kwargs):
        """
        allows to get an instanciation of the command in order to execute the corresponding function
        :param args: the instantiating arguments
        :type args: list
        :param kwargs: the instantiating arguments
        :type kwargs: dict
        :return the command implementation
        :rtype CommandImpl
        """
        impl = CommandImpl(cmd_obj=self, args=args, kwargs=kwargs)
        return impl


class CommandImpl():
    """
    an implementation of the command stored in the commands' dictionary
    """
    def __init__(self, cmd_obj, args, kwargs):
        """

        """
        self.__cmd_obj = cmd_obj
        self.__executor = None
        self.args = args
        self.kwargs = kwargs
        self._complete_args()
        self._get_executor()

    def _get_executor(self):
        """
        returns the object that executes the function
        :return the object having the executed function as method
        :rtype object
        """
        if self.__cmd_obj.fct_class in EXPOSED_OBJECTS:
            self.__executor = EXPOSED_OBJECTS[self.__cmd_obj.fct_class]
        elif self.__cmd_obj.fct_module is not None:
            m = importlib.import_module(self.__cmd_obj.fct_module)
            if self.__cmd_obj.fct_class is not None:
                self.__executor = m.__dict__[self.__cmd_obj.fct_class](*self.args, **self.kwargs)
            else:
                self.__executor = m

    def _complete_args(self):
        """
        inserts into the arguments the required static objects (if any)
        :return: None
        """
        if self.__cmd_obj.requires is not None:
            for req in self.__cmd_obj.requires:
                Command.LOG.debug(req)
                if req in EXPOSED_OBJECTS:
                    Command.LOG.debug("found")
                    self.kwargs[req] = EXPOSED_OBJECTS[req]

    def execute(self):
        """
        method called to execute the requested command
        :return: the function result (function is an executor or a threaded_executor)
        :rtype: Status
        """
        result = None
        for name in self.__cmd_obj.fct_name:
            fct = getattr(self.__executor, name)
            if fct is None:
                Command.LOG.error("unknown command executor")
                result = Status(FAILURE, "unknown command executor")
            elif "_executor" in fct.__name__:
                Command.LOG.debug("%s is a %s" % (name, fct.__name__))
                result = fct(*self.args, **self.kwargs)
            else:
                Command.LOG.error("command executor is not a valid executor. "
                                  "(function should have been decorated with @executor)")
                result = Status(FAILURE, "invalid executor")
        return result
