def acquire_acs_logger(logger):
    import __builtin__
    try:
        __builtin__.ACS_LOGGER = logger
    except:
        pass