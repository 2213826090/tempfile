import serial
import paramiko
import threading
import logging
import re
import time
import sys
import os
import pexpect.fdpexpect
from scp import SCPClient


class DebugCard(object):
    escape = '\x00\r'
    prompt = 'ACRN'
    NETWORK_PATH = '/etc/systemd/network'
    STATIC_NETWORK_FILE = os.path.join(NETWORK_PATH, 'eth-static.network')

    def __init__(self, port='/dev/ttyUSB3', baud=115200):
        self.serial = serial.Serial()
        self.serial.baudrate = baud
        self.serial.port = port
        self._sos_ip = None
        self._sos_gw = ''
        self._sos_username = None
        self._sos_password = None

    def open(self):
        if not self.serial.isOpen():
            logging.info('serial port opened: ' + self.serial.port)
            self.serial.open()
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.serial.flush()  # clean any invalid buffer
            self._expect = pexpect.fdpexpect.fdspawn(self.serial,
                                                     logfile=sys.stdout,
                                                     timeout=5)

    def close(self):
        if self.serial.isOpen():
            logging.info('serial port closed: ' + self.serial.port)
            self.serial.close()

    def __run_cmd(self, cmd):
        self._expect.send(self.escape)  # make sure in SOS shell
        self._expect.expect(self.prompt)

        raw_cmd = cmd + '\r\n'
        self._expect.sendline(raw_cmd)
        self._expect.expect(raw_cmd)  # exclude cmd echo

    def run_cmd(self, cmd):
        '''
        run SOS command
        '''
        assert 'vm_console' not in cmd, \
            'not supported, use switch_console instead'
        self.__run_cmd(cmd)

        self._expect.expect(self.prompt)  # wait until cmd execution done
        return self._expect.before  # return cmd output

    def run_linux_cmd(self, cmd):
        time.sleep(1)
        self._expect.sendline(cmd)
        self._expect.expect(cmd + '\r\n', timeout=5)
        time.sleep(1)
        self._expect.expect(self._cmd_prompt, timeout=30)
        # remove ansi escape (color)
        ansi_escape = re.compile(r'\x1B\[[0-?]*[ -/]*[@-~]')
        buf = ansi_escape.sub('', self._expect.before)
        self._expect.send('\r')
        return buf.strip()

    def get_ip_address(self, hw_card='acrn-br0'):
        ret = self.run_linux_cmd('ifconfig ' + hw_card)
        for l in ret.splitlines():
            if 'inet addr:' in l:
                return l.split()[1].split(':')[1]

    def get_gateway(self, hw_card='acrn-br0'):
        ret = self.run_linux_cmd('route -n | grep ' + hw_card)
        for l in ret.splitlines():
            if re.match(r'\d+', l):
                gateway = l.split()[1]
                if gateway != '0.0.0.0':
                    return gateway

    def switch_sos_console(self, login_name, login_pwd):
        '''
        switch to vm_console NUM
        '''
        logging.info('Call switch_sos_console()')
        cmd = 'vm_console'
        self.__run_cmd(cmd)
        time.sleep(2)
        self._expect.send('\r')  # active prompt
        self._cmd_prompt = login_name + '@'
        logging.info('Start SOS login ...')
        try:
            self._expect.expect('login:', timeout=40)  # check if login prompt
            logging.info('enter login name %s' % login_name)
            self._expect.sendline(login_name)
            self._expect.expect('Password:', timeout=0.5)  # check if Password prompt
            logging.info('enter login password %s' % login_pwd)
            self._expect.sendline(login_pwd)
            self._expect.expect(self._cmd_prompt)
            time.sleep(2)
        except:  # NOQA
            logging.warnning('Login SOS failed.')
            return False
        self.set_sos_username(login_name)
        self.set_sos_password(login_pwd)
        return True

    def set_static_sos_ip(self, ifname):
        """
        Set a static SOS IP address if SOS use DHCP to get IP,
        otherwise do nothing
        """
        # remove original /etc/systemd/network/eth-static.network file, then create new one
        cmd = "rm -f " + self.STATIC_NETWORK_FILE
        self.run_linux_cmd(cmd)

        # DHCP to get a new IP
        self.run_linux_cmd("dhclient -r")
        time.sleep(3)
        self.run_linux_cmd("dhclient")
        time.sleep(10) # wait for IP be allocated to network interface
        self._sos_ip = self.get_ip_address(ifname)
        self._sos_gw = self.get_gateway(ifname)
        if not self._sos_ip:
            self.get_logger().warning("Didn't get any SOS IP address.")
            return

        self.set_sos_ip_env(self._sos_ip)

        # write /etc/systemd/network/eth-static.network file, and scp to remote
        tmp_file = "/tmp/eth-static.network"
        fp = open(tmp_file, 'w')
        fp.write("[Match]\r\n")
        fp.write("name=" + ifname + "\r\n")
        fp.write("[Network]\r\n")
        fp.write("Address=" + self._sos_ip + "/24\r\n")
        fp.write("Gateway=" + self._sos_gw + "\r")
        fp.close()
        if self._sos_username and self._sos_password:
            ssh_cli = SSH(self._sos_ip, self._sos_username, self._sos_password)
            if ssh_cli:
                ssh_cli.push(tmp_file, self.STATIC_NETWORK_FILE)
                self.run_linux_cmd('systemctl restart systemd-networkd')
            else:
                logging.warning("Didn't get ssh connection.")
        # restart systemd-network to directly enable static IP address of SOS
        os.remove(tmp_file)

    def launch_uos(self):
        self.run_linux_cmd("~/launch_UOS.sh -V 2")

    def set_sos_ip_env(self, ip_addr):
        os.environ['ACRN_SOS_IP'] = ip_addr

    def get_sos_ip(self):
        if self._sos_ip:
            return self._sos_ip
        else:
            return os.environ.get('ACRN_SOS_IP')

    def set_sos_username(self, username):
        self._sos_username = username
        os.environ['ACRN_SOS_USERNAME'] = username

    def get_sos_username(self):
        if self._sos_username:
            return self._sos_username
        else:
            return os.environ.get('ACRN_SOS_USERNAME')

    def set_sos_password(self, password):
        self._sos_password = password
        os.environ['ACRN_SOS_PASSWORD'] = password

    def get_sos_password(self):
        if self._sos_password:
            return self._sos_password
        else:
            return os.environ.get('ACRN_SOS_PASSWORD')

    def get_ssh(self, username='', password=''):
        addr = self.get_sos_ip()
        name = username if username else self.get_sos_username()
        passwd = password if password else self.get_sos_password()
        if addr and name and passwd:
            return SSH(addr, name, passwd)

class SSH(object):
    def __init__(self, addr, username='root', password=''):
        self._addr = addr
        self._username = username
        self._password = password
        self.client = paramiko.client.SSHClient()

        policy = paramiko.client.MissingHostKeyPolicy()
        self.client.set_missing_host_key_policy(policy)
        self.client.connect(addr, username=username, password=password)

    def __get_transport(self):
        self.client.connect(self._addr,
                            username=self._username,
                            password=self._password)
        return self.client.get_transport()

    def run_cmd(self, cmd, timeout=10):
        '''
        run command in SSH shell

        return: (exit_code, stdout, stderr)
        '''
        logging.info('run_cmd: ' + cmd)
        proc = self.run_cmd_async(cmd)

        start = time.time()
        while time.time() - start < timeout:  # monitoring process
            # pull complete at top to make sure data recv
            if proc.poll() is not None:
                break
            time.sleep(0.1)
        else:
            print 'CMD Timeout(%ss): %s' % (timeout, cmd)  # TODO: raise
            proc.kill()
        ret_code = proc.poll()
        outdata = proc.stdout
        errdata = proc.stderr
        proc.destory()
        return (ret_code, outdata, errdata)

    def run_cmd_async(self, cmd):
        '''
        run command async in SSH shell

        return: Process object
        '''
        logging.info('run_cmd_async: ' + cmd)
        trans = self.__get_transport()
        proc = Process(trans, cmd)
        return proc

    def pull(self, remote, local):
        '''
        pull REMOTE file to LOCAL through ssh
        '''
        logging.info('pull %s --> %s' % (remote, local))
        trans = self.__get_transport()
        scp = SCPClient(trans)
        scp.get(remote, local)
        scp.close()
        trans.close()

    def push(self, local, remote):
        '''
        push LOCAL file to REMOTE through ssh
        '''
        logging.info('push %s --> %s' % (local, remote))
        trans = self.__get_transport()
        scp = SCPClient(trans)
        scp.put(local, remote)
        scp.close()
        trans.close()


class Process(object):
    def __init__(self, transport, cmd):
        self.transport = transport
        self.channel = transport.open_session()
        self.cmd = cmd
        self.stdout = ''
        self.stderr = ''

        self._lock = threading.Lock()
        self._completed = False
        self._run_cmd()

    def kill(self):
        ''' kill process '''
        logging.info('Process killed: ' + self.cmd)
        channel = self.transport.open_session()
        channel.exec_command('kill -s SIGINT ' + self.pid)

    def poll(self):
        ''' check if process is finished
        if yes, return exit code, else return None'''
        ret = None
        self._lock.acquire()
        if self._completed:
            ret = self._exit_code
        self._lock.release()
        return ret

    def _run_cmd(self):
        # tricks:
        # first part: 'echo $$' to get PID
        # last part: 'exit $?' to get exit code

        self.channel.get_pty()
        self.channel.invoke_shell()
        # pre config shell to disable promtp and echo
        pre = 'export PS1="" PS2=""; stty -echo \r\n'
        self.channel.send(pre)
        time.sleep(0.1)
        self.channel.recv(1000)

        cmd = 'echo $$; ' + self.cmd + '; exit $? \r\n'
        self.channel.send(cmd)
        # first to get PID
        buff = self.channel.recv(1000)
        index = buff.find('\n') + 1
        self.pid = buff[:index].strip()
        self.stdout = buff[index:]

        self.channel.setblocking(False)

        def check_proc():
            chan = self.channel
            while True:
                while chan.recv_ready():
                    self.stdout += chan.recv(1000)
                while chan.recv_stderr_ready():
                    self.stderr += chan.recv_stderr(1000)
                self._lock.acquire()
                self._completed = self.channel.exit_status_ready()
                if self._completed:
                    self._exit_code = self.channel.recv_exit_status()
                    self._lock.release()
                    break
                self._lock.release()
                time.sleep(.1)
            index = self.stdout.rfind('logout\r\n')
            self.stdout = self.stdout[:index]
        self._thread = threading.Thread(target=check_proc)
        self._thread.start()

    def destory(self):
        self.transport.close()

