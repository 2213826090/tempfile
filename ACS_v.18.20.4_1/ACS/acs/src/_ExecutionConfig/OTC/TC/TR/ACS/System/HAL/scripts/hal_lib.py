def get_process_pid(ps_out):
    if len(ps_out.split("\n")) == 2:
        return ps_out.split("\n")[1].split()[1]
    else:
        return None

def get_binding_count(halctl_out):
    if len(halctl_out.split("\n")) > 1:
        count = None
        for elem in reversed(halctl_out.split("\n")):
            if "Binding" in elem:
                return elem.split()[1][1:]
        return None

def get_missing_tokens(out, toks):
    missing_tokens = []
    for token in toks:
        if token not in out:
            missing_tokens.append(token)
    return missing_tokens


def check_help_message(halctl_out):
    # will check the presence of the parameter names in the help message
    # will not check the whole help message
    tokens = ["--help, -h:", "--add, -a:", "--dry-run, -d:",
                "--file, -f:", "--get-module, -g:", "--list, -l:",
                "--filter, -i:", "--del, -s:", "--loglevel, -e:",
                "--notify, -n:", "--verbose, -v:", "MODALIAS:"]
    #return missing_tokens
    return get_missing_tokens(halctl_out, tokens)

def check_module_info(halctl_output, mod_id, camera_mod_name, camera_mod_author, mandatory=True):
    if halctl_output == "Could not get HW module for audio":
        if mandatory:
            return False
        else:
            return True
    else:
        # check module info
        for line in halctl_output.split():
            if "HW module ID" in line:
                m_id = line.split(":")[1].strip()
                if m_id != mod_id:
                    return False

            if "HW module name" in line:
                m_name = line.split(":")[1].strip()
                if m_name != camera_mod_name:
                    return False

            if "HW module author" in line:
                m_aut = line.split(":")[1].strip()
                if m_aut != camera_mod_author:
                    return False
    return True

def check_kmod_usage(kmod_out):
    # will check the presence of the parameter names in the help message
    # will not check the whole help message
    tokens = ["-i module", "-r module", "-n module", "-v"]
    #return missing_tokens
    return get_missing_tokens(kmod_out, tokens)

def get_kmod_name(halctl_out):
    for line in reversed(halctl_out.split("\n")):
        if "kmod:" in line:
            return line.split(":")[1].strip('"')

def check_kmod_out(kmod_out):
    tokens = ["filename:", "name:", "lias:", "description:", "author:", "license:",
              "depends:", "intree:", "vermagic:", "signer:", "sig_key:", "sig_hashalgo:"]
    return get_missing_tokens(kmod_out, tokens)

def check_kmod_verbose(kmod_out):
    tokens = ["kmod_module_new_from_lookup:", "kmod_module_new_from_lookup:",
              "kmod_search_moddep:", "kmod_pool_get_module:", "kmod_pool_add_module:",
              "kmod_module_parse_depline:", "kmod_module_new_from_lookup:",
              "kmod_module_get_path:", "kmod_module_get_path:", "kmod_module_unref:",
              "kmod_pool_del_module:", "kmod_unref:"]
    tokens.extend(check_kmod_out(kmod_out))
    return get_missing_tokens(kmod_out, tokens)


def get_module_attribute(out, attr):
    if attr in out:
        for line in out.split("\n"):
            if attr in line:
                elem_list = line.split(":")
                return ":".join(elem_list[1:]).strip('"')
    else:
        return None

def get_module_alias(out):
    return get_module_attribute(out, "modalias:")

def get_ref_count(out):
    return get_module_attribute(out, "refcount:")

def get_module_type(out):
    return get_module_attribute(out, "type:")

def get_module_halid(out):
    return get_module_attribute(out, "hal_id:")

