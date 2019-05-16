"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: Wrapper for COM port XML RPC server. Those functions use a USBRly08 instance
        to get a server proxy instance (similar to a 'handle' role in other wrappers).
:since: 17/10/2011
:author: asebbane
"""

import sys
from ErrorHandling.TestEquipmentException import TestEquipmentException

OUTPUT_SUCCESS_PREFIX = "output:SUCCESS"


def Write(eqt, cmd, retry=3):
    """
    Writes the given I{command}.

    :type eqt: USBRly08
    :param eqt: the equipment that uses the wrapper

    :type cmd: str
    :param cmd: the command to write

    :type retry: int
    :param retry: the maximum number of retries when a command fails.

    :rtype: str
    :return: the result of the command as str.
    """
    proxy = eqt.get_server_proxy()
    result = None
    current_exception = None
    current_retry = 0
    shall_retry = True
    while current_retry < retry and shall_retry:
        # We allow the protected access here.
        # pylint: disable=W0212
        eqt._check_rpc_server_running()
        try:
            result = proxy.write(cmd)
            current_exception = None
            shall_retry = False
        except Exception as exc:  # pylint: disable=W0703
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(exc))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(sys.exc_info()[0]))
        finally:
            current_retry += 1
    if current_retry >= retry and current_exception is not None:
        # We disable the pylint warning because the raised
        # object will never be None at this point
        # pylint: disable=E0702
        raise current_exception
    return result


def Read(eqt, byte_count, retry=3):
    """
    Read the given I{command}.

    :type eqt: USBRly08
    :param eqt: the equipment that uses the wrapper

    :type byte_count: int
    :param byte_count: the number of bytes to read

    :type retry: int
    :param retry: the maximum number of retries when a command fails.

    :rtype: str
    :return: the result of the read command as str.
    """
    proxy = eqt.get_server_proxy()
    result = None
    current_exception = None
    current_retry = 0
    shall_retry = True
    while current_retry < retry and shall_retry:
        # We allow the protected access here.
        # pylint: disable=W0212
        eqt._check_rpc_server_running()
        try:
            result = proxy.read(byte_count)
            current_exception = None
            shall_retry = False
        except Exception as exc:  # pylint: disable=W0703
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(exc))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(sys.exc_info()[0]))
        finally:
            current_retry += 1
    if current_retry >= retry and current_exception is not None:
        # We disable the pylint warning because the raised
        # object will never be None at this point
        # pylint: disable=E0702
        raise current_exception
    return result


def Stop(eqt, retry=3):
    """
    Stops the underlying I{XML RPC server} instance.

    :type eqt: USBRly08
    :param eqt: the equipment that uses the wrapper

    :type retry: int
    :param retry: the maximum number of retries when a command fails.

    :rtype: str
    :return: the result of the read command as str.
    """
    proxy = eqt.get_server_proxy()
    result = None
    current_exception = None
    current_retry = 0
    shall_retry = True
    while current_retry < retry and shall_retry:
        # We allow the protected access here.
        # pylint: disable=W0212
        eqt._check_rpc_server_running()
        try:
            result = proxy.stop()
            if not result.startswith(OUTPUT_SUCCESS_PREFIX):
                current_exception = TestEquipmentException(
                    TestEquipmentException.SPECIFIC_EQT_ERROR,
                    result)
            else:
                shall_retry = False
        except Exception as exc:  # pylint: disable=W0703
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(exc))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            current_exception = TestEquipmentException(
                TestEquipmentException.DEFAULT_ERROR_CODE,
                str(sys.exc_info()[0]))
        finally:
            current_retry += 1
    if current_retry >= retry and current_exception is not None:
        # We disable the pylint warning because the raised
        # object will never be None at this point
        # pylint: disable=E0702
        raise current_exception
    return result


def SetRelayOn(eqt, line, retry=3):
    """
    Enables input/output line.

    :type eqt: USBRly08
    :param eqt: the equipment that uses the wrapper

    :type retry: int
    :param retry: the maximum number of retries when a command fails.

    :type line: integer
    :param line: the number of line to enable. 0 means all
    """
    proxy = eqt.get_server_proxy()
    result = None
    current_exception = None
    current_retry = 0
    shall_retry = True
    while current_retry < retry and shall_retry:
        # We allow the protected access here.
        # pylint: disable=W0212
        eqt._check_rpc_server_running()
        try:
            if line:
                result = proxy.set_relay_on(line)
            else:
                result = proxy.set_all_relay_on()
            if not result.startswith(OUTPUT_SUCCESS_PREFIX):
                current_exception = TestEquipmentException(
                    TestEquipmentException.SPECIFIC_EQT_ERROR,
                    result)
            else:
                shall_retry = False
        except Exception as exc:  # pylint: disable=W0703
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(exc))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(sys.exc_info()[0]))
        finally:
            current_retry += 1
    if current_retry >= retry and current_exception is not None:
        # We disable the pylint warning because the raised
        # object will never be None at this point
        # pylint: disable=E0702
        raise current_exception
    return result


def SetRelayStates(eqt, states, retry=3):
    """
    Sends a single byte that represents the desired relay states.
    All on: 255, all off: 0.
    :type states: integer
    :param states: the desired states far all relays. An integer
    from 0 to 255.

    :type retry: int
    :param retry: the maximum number of retries when a command fails.
    """
    proxy = eqt.get_server_proxy()
    result = None
    current_exception = None
    current_retry = 0
    shall_retry = True
    while current_retry < retry and shall_retry:
        # We allow the protected access here.
        # pylint: disable=W0212
        eqt._check_rpc_server_running()
        try:
            result = proxy.set_relay_states(states)
            if not result.startswith(OUTPUT_SUCCESS_PREFIX):
                current_exception = TestEquipmentException(
                    TestEquipmentException.SPECIFIC_EQT_ERROR,
                    result)
            else:
                shall_retry = False
        except Exception as exc:  # pylint: disable=W0703
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(exc))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(sys.exc_info()[0]))
        finally:
            current_retry += 1
    if current_retry >= retry and current_exception is not None:
        # We disable the pylint warning because the raised
        # object will never be None at this point
        # pylint: disable=E0702
        raise current_exception
    return result


def SetRelayOff(eqt, line, retry=3):
    """
    Disables input/output line.

    :type eqt: USBRly08
    :param eqt: the equipment that uses the wrapper

    :type line: integer
    :param line: the number of line to disable. 0 means all

    :type retry: int
    :param retry: the maximum number of retries when a command fails.
    """
    proxy = eqt.get_server_proxy()
    result = None
    current_exception = None
    current_retry = 0
    shall_retry = True
    while current_retry < retry and shall_retry:
        # We allow the protected access here.
        # pylint: disable=W0212
        eqt._check_rpc_server_running()
        try:
            if line:
                result = proxy.set_relay_off(line)
            else:
                result = proxy.set_all_relay_off()
            if not result.startswith(OUTPUT_SUCCESS_PREFIX):
                current_exception = TestEquipmentException(
                    TestEquipmentException.SPECIFIC_EQT_ERROR,
                    result)
            else:
                shall_retry = False
        except Exception as exc:  # pylint: disable=W0703
            print str(exc)
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(exc))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(sys.exc_info()[0]))
        finally:
            current_retry += 1
    if current_retry >= retry and current_exception is not None:
        # We disable the pylint warning because the raised
        # object will never be None at this point
        # pylint: disable=E0702
        raise current_exception
    return result


def PressRelay(eqt, line, duration, retry=3):
    """
    Press (on/off) line.

    :type eqt: USBRly08
    :param eqt: the equipment that uses the wrapper

    :type line: integer
    :param line: the number of line to press

    :type duration: integer
    :param duration: press duration

    :type retry: int
    :param retry: the maximum number of retries when a command fails.
    """
    proxy = eqt.get_server_proxy()
    result = None
    current_exception = None
    current_retry = 0
    shall_retry = True
    while current_retry < retry and shall_retry:
        # We allow the protected access here.
        # pylint: disable=W0212
        eqt._check_rpc_server_running()
        try:
            result = proxy.press_relay(line, duration)
            if not result.startswith(OUTPUT_SUCCESS_PREFIX):
                current_exception = TestEquipmentException(
                    TestEquipmentException.SPECIFIC_EQT_ERROR,
                    result)
            else:
                shall_retry = False
        except Exception as exc:  # pylint: disable=W0703
            print str(exc)
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(exc))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(sys.exc_info()[0]))
        finally:
            current_retry += 1
    if current_retry >= retry and current_exception is not None:
        # We disable the pylint warning because the raised
        # object will never be None at this point
        # pylint: disable=E0702
        raise current_exception
    return result


def Ping(eqt, retry=3):
    """
    Sends a simple request to the underlying I{XML RPC server} instance.

    :type eqt: USBRly08
    :param eqt: the equipment that uses the wrapper

    :rtype: bool
    :return: the result of the 'ping' command as boolean.

    :type retry: int
    :param retry: the maximum number of retries when a command fails.
    """
    # We must not check whether a server is running
    proxy = eqt.get_server_proxy()
    ping_result = False
    current_retry = 0
    shall_retry = True
    while current_retry < retry and shall_retry:
        try:
            result = proxy.ping()
            if result.startswith(OUTPUT_SUCCESS_PREFIX):
                ping_result = True
            else:
                shall_retry = False
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            # We want to catch any error or exception
            # pylint: disable=W0702
            ping_result = False
        finally:
            current_retry += 1
    return ping_result


def SetParam(eqt, name, value, retry=3):
    """
    Set parameter to the given I{command}.

    :type eqt: USBRly08
    :param eqt: the equipment that uses the wrapper

    :type name: str
    :param name: name of the parameter

    :rtype: str
    :param: value: parameter value

    :type retry: int
    :param retry: the maximum number of retries when a command fails.

    :rtype: bool
    :return: True if success
    """
    proxy = eqt.get_server_proxy()
    result = None
    current_exception = None
    current_retry = 0
    shall_retry = True
    while current_retry < retry and shall_retry:
        # We allow the protected access here.
        # pylint: disable=W0212
        eqt._check_rpc_server_running()
        try:
            result = proxy.set_param(name, value)
            current_exception = None
            shall_retry = False
        except Exception as exc:  # pylint: disable=W0703
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(exc))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            current_exception = TestEquipmentException(
                TestEquipmentException.SPECIFIC_EQT_ERROR,
                str(sys.exc_info()[0]))
        finally:
            current_retry += 1
    if current_retry >= retry and current_exception is not None:
        # We disable the pylint warning because the raised
        # object will never be None at this point
        # pylint: disable=E0702
        raise current_exception
    return result
