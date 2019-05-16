COMMANDS = {
    "help":
    {"description": "requires available commands list. If a specific command is given after help,"
     "provides the description and required arguments for the command",
        "function": {"_module": None,
                      "_class": "cmd_catalog",
                      "_name": "help",
                      "_args": {"cmd": str}}},
    "join":
        {"description": "joins a previous deferred function execution",
         "function": {"_module": None,
                      "_class": "CommandServer",
                      "_name": "join_executor",
                      "_args": {"pid": int}}},
    "get_args":
        {"description": "allows to know the arguments and their type for a command",
         "function": {"_module": None,
                      "_class": "cmd_catalog",
                      "_name": "get_args",
                      "_args": {"cmd": str}}}
            }
