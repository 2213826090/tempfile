#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -------------------------------------------------------------------------------
# @copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
# The source code contained or described here in and all documents related
# to the source code ("Material") are owned by Intel Corporation or its
# suppliers or licensors. Title to the Material remains with Intel Corporation
# or its suppliers and licensors. The Material contains trade secrets and
# proprietary and confidential information of Intel or its suppliers and
# licensors.

# The Material is protected by worldwide copyright and trade secret laws and
# treaty provisions. No part of the Material may be used, copied, reproduced,
# modified, published, uploaded, posted, transmitted, distributed, or disclosed
# in any way without Intel's prior express written permission.

# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise. Any license under such intellectual property rights must be express
# and approved by Intel in writing.

# @organization: INTEL MCG PSI
# @summary: <enter_summary_details_here>
# @since: 4/18/14
# @author: nbrissox
# -------------------------------------------------------------------------------
from setups import DEBUG as GLOBAL_DEBUG


class DictSet(list):

    """
    This Class represents a set of dict.
    Basically, it ensures the uniqueness of the dict item contained in the list based on a key

    """

    def __init__(self, iterable, key='name'):
        """
        Constructor

        :param iterable: see list definition
        :param key: The key on which discrimination is done

        """
        super(DictSet, self).__init__(iterable)

        values = dict((v[key], v) for v in self).values()
        self.clear()

        for value in values:
            self.append(value)

    def clear(self):
        """
        Clears all items in self

        """
        del self[:]


class StringBase(object):

    """
    This base implements String, Dictionary Str representations

    Override methods:

        * __str__
        * __unicode__

    .. note:: DEBUG attribute

        If set to True (or 1) the instance's str or unicode representation is enhanced for debugging.
        Just override it in your Subclass according your needs :)

    """

    DEBUG = GLOBAL_DEBUG

    def __str__(self):
        """
        Format String & Unicode representations in a pretty generic manner

        .. note:: for now, there won't be any effect if your subclass is not of one above type

        """
        # default behavior
        message = super(StringBase, self).__str__()

        if self.DEBUG:
            classname = self.__class__.__name__
            # Class attributes & methods
            cls_dict = dict(self.__class__.__dict__)
            # Instance attributes & methods
            cls_dict.update(self.__dict__)
            # Attribute/Value pair separator
            separator = '\n' + '-' * 120 + '\n'
            message = '{1}:\n\t{0}\n'.format(super(StringBase, self).__str__(), classname)
            primitives = int, bool, str, unicode, float, long,
            for attr, value in cls_dict.iteritems():
                if attr not in ('__doc__', '__module__'):
                    message += "{1}: {0}\n".format(value, attr)
                    if not isinstance(value, primitives):
                        try:
                            doc = ""
                            for line in value.__doc__.split('\n'):
                                doc += '\t{0}\n'.format(line)
                            doc = doc.strip()
                        except AttributeError:
                            doc = value
                        message += "\t{1}.__doc__: {0}{2}".format(doc, attr, separator)

        return message

    __unicode__ = __str__


class AttributeStr(StringBase, str):

    """
    Str subclass which allows arbitrary attribute access.

    """

    # noinspection PyInitNewSignature
    def __new__(cls, chars, **kwargs):
        """

        :param chars:
        :param chars:

        :param kwargs:
        :param kwargs:

        :return:
        :return:

        """
        obj = str.__new__(cls, chars)
        for k, v in kwargs.iteritems():
            setattr(obj, k, v)
        return obj


def str2bool(text):
    """
    Accepts as bool value one of the boolean representation
    such as [true, on, 1, yes] for True and [false, off, 0, no] for False
    if none of the above is matched it returns None

    :type   text: string
    :param  text: string representing a boolean value

    :rtype:     boolean
    :return:    Boolean value True or False

    >>> str2bool("true")
    True
    >>> str2bool("1")
    True
    >>> str2bool("yes")
    True
    >>> str2bool("on")
    True
    >>> str2bool("false")
    False
    >>> str2bool("off")
    False
    >>> str2bool("0")
    False
    >>> str2bool("no")
    False
    """
    return str(text).lower() in ("true", "1", "yes", "on")
