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

:organization: INTEL QCTV
:summary: implementation class for SQLite3 Database connection
:since: 18/08/2014
:author: jduran4x
"""
import sqlite3 as lite


class SQLite(object):
    def __init__(self, database):
        self._conn = None
        self._cursor = None
        self._db_file = database

    def connect(self):
        """
        connects to the database
        """
        # create a connection only if not already connected
        if self._conn is None:
            self._conn = lite.connect(self._db_file)
            self._conn.row_factory = lite.Row
            self._conn.text_factory = str
        self._cursor = self._conn.cursor()

    def save(self):
        """
        save the queried modification to the database
        """
        self._conn.commit()

    def close(self):
        """
        close the database connection
        """
        self._cursor.close()
        self._conn.close()
        del self._cursor
        del self._conn
        self._conn = None
        self._cursor = None

    def send_request(self, query, *args):
        """
        send the database request
        :param query: the SQL query
        :type query: str
        :param args: the values to be set by the query
        :type args: tuple
        :return: the query answer
        :rtype: list
        """
        if len(args) > 0:
            if type(args[0]).__name__ in ["list", "dict", "tuple"]:
                self._cursor.execute(query, args[0])
            else:
                self._cursor.execute(query, args)
        else:
            self._cursor.execute(query)
        result = self._cursor.fetchall()
        return result

    def send_many_requests(self, query, *args):
        """
        send several database requests at once
        :param query: the SQL query
        :type query: str
        :param args: the values to be set by the query
        :type args: tuple
        :return: the query answer
        :rtype: list
        """
        if len(args) > 0:
            if type(args[0]).__name__ in ["list", "dict", "tuple"]:
                self._cursor.executemany(query, args[0])
            else:
                self._cursor.executemany(query, args)
        else:
            self._cursor.execute(query)
        result = self._cursor.fetchall()
        return result

