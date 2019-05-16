#!/usr/bin/env python

import logging
import dbus
import dbus.service
import time
import os
from decoration import *
from bt.constants import Constants as Consts


class Rejected(dbus.DBusException):
    _dbus_error_name = "org.bluez.Error.Rejected"

class Canceled(dbus.DBusException):
    _dbus_error_name = "org.bluez.Error.Canceled"

class Timeout(dbus.DBusException):
    _dbus_error_name = "org.freedesktop.DBus.Error.NoReply"


class BtPairHandler(dbus.service.Object):

    CANCEL = "cancel"
    REJECT = "reject"
    ACCEPT = "accept"
    TIMEOUT = "timeout"
    WRONG_PASS = "wrong_pass"
    DEFAULT_REPLY = ACCEPT

    PAIRING_TIMEOUT = 60

    DEFAULT_PIN_CODE = "0000"
    DEFAULT_PASSKEY = "123456"
    DEFAULT_CAPABILITY = "KeyboardDisplay"

    INCOMING_REQUEST = "in"
    OUTGOING_REQUEST = "out"

    __instance = None
    __agent_initialized = False
    PATH = "/Intel/Edison/agent"

    def __new__(cls, *args, **kwargs):
        """
        """
        if cls.__instance is None:
            cls.__instance = dbus.service.Object.__new__(cls, *args, **kwargs)
            cls._log = cls.set_logger()
        return cls.__instance

    def __init__(self, adapter, passkey=DEFAULT_PASSKEY, pin_code=DEFAULT_PIN_CODE,
                 capability=DEFAULT_CAPABILITY, timeout=PAIRING_TIMEOUT,
                 reply=DEFAULT_REPLY, *args, **kwargs):
        """
        Agent constructor
        @type bus: DBus object
        @param bus: bus on which to listen for events
        @type path: 
        @param path:
        """
        self._log = self.set_logger()
        self._pairing_reply = reply
        self._pin_code = pin_code
        self._passkey = passkey
        self._timeout = timeout
        self._capability = capability
        self._answer = None
        self._direction = None
        self._device_obj = None
        self._dev_addr = None
        self._adapter = adapter
        if not BtPairHandler.__agent_initialized:
            try:
                bus = dbus.SystemBus()
                obj = bus.get_object(Consts.Bt.SVC_NAME, Consts.DbusConstants.BT_PATH)
                manager = dbus.Interface(obj, Consts.Bt.AGENT_MNGR_IFACE)
                dbus.service.Object.__init__(self, bus, BtPairHandler.PATH)
                manager.RegisterAgent(BtPairHandler.PATH, self._capability)
                manager.RequestDefaultAgent(BtPairHandler.PATH)
            except dbus.exceptions.DBusException as E:
                self._log.error(E)
            except:
                self._log.error("Failed to register pairing agent")
            BtPairHandler.__agent_initialized = True
            self._log.debug("Pairing agent registered")

    @staticmethod
    def set_logger():
        # Set logging
        # Log format
        log_fmt = '%(asctime)s   BtPair\t%(levelname)5s   %(message)s'
        # Log level => default log level is INFO
        log_lvl = logging.DEBUG
        log_file = "/tmp/bt_edison.log"
        # Retrieve logger
        logging.basicConfig(filename=log_file, level=log_lvl, format=log_fmt)
        Log = logging.getLogger('BtPair')
        Log.name = 'BtPair'
        return Log

    @dbus.service.method(Consts.Bt.AGENT_IFACE, in_signature="", out_signature="")
    def Release(self):
        """
        """
        self._log.debug("Agent.Release")

    @dbus.service.method(Consts.Bt.AGENT_IFACE, in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        """
        """
        self._log.debug("AuthorizeService (%s, %s)" % (device, uuid))
        authorize = "yes"
        if authorize == "yes":
            return
        raise Rejected("Connection rejected by user")

    @dbus.service.method(Consts.Bt.AGENT_IFACE, in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        """
        """
        self._log.debug("RequestPinCode (%s)" % (device))
        self._log.debug("Use %s configured PIN code" % (self._pin_code))
        return self._pin_code

    @dbus.service.method(Consts.Bt.AGENT_IFACE, in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        """
        """
        self._log.debug("RequestPasskey (%s)" % (device))
        return dbus.UInt32(self._passkey)

    @dbus.service.method(Consts.Bt.AGENT_IFACE, in_signature="ouq", out_signature="")
    def DisplayPasskey(self, device, passkey, entered):
        """
        """
        self._log.debug("DisplayPasskey (%s, %06u entered %u)" % (device, passkey, entered))

    @dbus.service.method(Consts.Bt.AGENT_IFACE, in_signature="os", out_signature="")
    def DisplayPinCode(self, device, pincode):
        """
        """
        self._log.debug("DisplayPinCode (%s, %s)" % (device, pincode))

    @dbus.service.method(Consts.Bt.AGENT_IFACE, in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        """
        """
        self._log.debug("RequestConfirmation (%s, %06d)" % (device, passkey))

        # Check peer device address in case of incoming connexion
        if self._direction == BtPairHandler.INCOMING_REQUEST and self._dev_addr is not None:
            device = os.path.basename(device).replace('dev_', '').replace('_', ':')
            if device != self._dev_addr:
                raise Rejected("Pairing request from Wrong peer device")

        if self._pairing_reply == BtPairHandler.ACCEPT:
            self._log.info("Request automatically accepted")
            if self._direction == BtPairHandler.INCOMING_REQUEST:
                self._answer = (SUCCESS, "Request automatically accepted")
        elif self._pairing_reply == BtPairHandler.CANCEL:
            self._log.info("Request automatically cancelled")
            self._answer = (FAILURE, "Pairing cancelled")
            raise Rejected("Pairing cancelled")
        elif self._pairing_reply == BtPairHandler.TIMEOUT:
            self._log.info("Wait for pairing request timeout")
            self._answer = (FAILURE, "timeout")
            while True:
                # Loop forever to generate a timeout on peer device
                # This loop will be break by get_result function timeout
                time.sleep(1)
        elif self._pairing_reply == BtPairHandler.REJECT:
            self._log.info("Request automatically rejected")
            self._answer = (FAILURE, "Pairing rejected")
            raise Rejected("Pairing rejected")
        elif self._pairing_reply == BtPairHandler.WRONG_PASS:
            self._log.info("Request automatically rejected")
            self._answer = (FAILURE, "Passkey doesn't match")
            raise Rejected("Passkey doesn't match")
        else:
            self._answer = (FAILURE, "User rejected request")
            raise Rejected("User rejected request")

        self._log.debug("End of Agent.RequestConfirmation: %s" % device)
        return

    @dbus.service.method(Consts.Bt.AGENT_IFACE, in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        """
        """
        self._log.info("Request authorization")
        if self._paring_reply == BtPairHandler.REJECT:
            self._log.info("Pairing request rejected")
            raise Rejected("Pairing rejected")
        elif self._paring_reply == BtPairHandler.CANCEL:
            self._log.info("Pairing request cancelled")
            raise Canceled("Pairing cancelled")
        else:
            self._log.info("Authorize incoming pairing attempt")
            return

    @dbus.service.method(Consts.Bt.AGENT_IFACE, in_signature="", out_signature="")
    def Cancel(self):
        """
        Pairing cancel callback
        """
        self._log.debug("Agent.Cancel")

    def _pair_reply(self):
        """
        Remote device pairing reply callback
        """
        self._log.info("Peer device has accepted pairing request")
        self._answer = (SUCCESS, "Peer device has accepted pairing request")

    def _pair_error(self, error):
        """
        Pairing error callback
        """
        err_name = error.get_dbus_name()
        if self._answer is None:
            if err_name == "org.freedesktop.DBus.Error.NoReply" and self._device_obj:
                self._log.error("Timed out. Cancelling pairing")
                self._answer = (FAILURE, "Timed out. Cancelling pairing")
                self.device_obj.CancelPairing()
            else:
                self._log.error("Creating device failed: %s" % error)
                self._answer = (FAILURE, "Creating device failed: %s" % error)

    @executor
    def pair(self, addr, *args, **kwargs):
        """
        Start pairing request with device at dev_addr address
        """
        self._log.info("Pairing with %s device" % str(addr))
        self._dev_addr = addr
        devices = self._adapter.get_devices()
        if addr in devices:
            self._device_obj = devices[addr].get_dbus_device_iface()
            self._direction = BtPairHandler.OUTGOING_REQUEST
            self._device_obj.Pair(reply_handler=self._pair_reply, error_handler=self._pair_error, timeout=self._timeout)
        else:
            self._log.error("Device not found")
            self._answer = (FAILURE, "Device not found")

    @executor
    def wait_for_pairing(self, addr=None, *args, **kwargs):
        """
        Wait for a remote device pairing request
        """
        self._dev_addr = addr
        if self._dev_addr == None:
            self._log.info("Waiting for remote device pairing request")
        else:
            self._log.info("Waiting for %s remote device pairing request" % self._dev_addr)
        # Set request direction 
        self._direction = BtPairHandler.INCOMING_REQUEST

    @threaded_executor
    def get_result(self, *args, **kwargs):
        """
        Get pairing result
        """
        begin_time = time.time()
        while (time.time() - begin_time) < self._timeout:
            if self._answer is not None:
                break
            time.sleep(1)

        if self._answer is None:
            self._answer = (FAILURE, "Timeout")

        return self._answer
