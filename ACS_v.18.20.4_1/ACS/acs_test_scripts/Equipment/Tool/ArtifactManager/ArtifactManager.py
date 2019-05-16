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
:summary: This file implements a ArtifactManager class to manage all binaries
:since:24/02/2014
:author: nbrissox
"""

import os
from os import path
import shutil
import traceback
import zipfile
import logging
from urlparse import urljoin

from Core.PathManager import Folders
from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Caching import SysErrors, ArtifactoryCacheManager, compute_file_hash
import UtilitiesFWK.Utilities as Util
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.Tool.Interface.IArtifactManager import IArtifactManager
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.HttpDownloaderUtil import HttpDownloaderUtil



# Use a volatile cache for now, waiting for full artifact cache implementation
# avoid to mess us this system cache directory until full cache implementation is not finished
DEFAULT_CACHE_ARTIFACTS_PATH = path.join(Folders.ACS_CACHE, 'Artifacts')
DEFAULT_ARTIFACT_URI = "https://mcg-depot.intel.com/artifactory/acs_test_artifacts/"
DEFAULT_MAX_CACHE = 10000
ZIP_EXT_SUPPORTED = [".zip", ".ZIP"]


class ArtifactManager(EquipmentBase, IArtifactManager):
    """
    ArtifactManager class that manages all device artifacts.
    It permits to store a local/distant file (an artifact) in a cache folder.

    """

    @staticmethod
    def _compute_remote_uri(artifact_name, artifact_root_uri):
        """
        Generates the full artifact uri according to artifact name as::

            * Http URL
            * Local file system path

        :param artifact_name: the name of artifact
        :type  artifact_name: str

        :param artifact_root_uri: the uri of artifact source
        :type  artifact_root_uri: str

        :return: uri to the artifact to process
        :rtype: str

        """
        if HttpDownloaderUtil.is_http_uri(artifact_root_uri):
            if not artifact_root_uri.endswith('/'):
                artifact_root_uri += '/'
            artifact_to_process = urljoin(artifact_root_uri, artifact_name)
        else:
            error_msg = "Mal-Formed URI {0} for artifact {1}".format(artifact_root_uri, artifact_name)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        return artifact_to_process

    @property
    def whoami(self):
        """
        Override this to change Logger's name.

        :return: The Logger's name to be used by :mod:`logging`, :func:`logging.getLogger`
        :rtype: str

        .. warning:: The Logger's name maximum length is about 13 chars,
            if longer then it'll be truncated and prefixed with ``..``

        """
        return "ART_MGR"

    @property
    def artifact_root_uri(self):
        """
        :return: the uri source
        :rtype: str

        """
        return self.__artifact_root_uri

    @property
    def cache_artifacts_path(self):
        """
        :return: the path to the artifacts cache directory
        :rtype: str

        """
        return self.__cache_artifacts_path

    @property
    def cache_size(self):
        """
        :return: maximum cache size (Mb)
        :rtype: int

        """
        return self.__cache_size

    @property
    def http_config(self):
        """
        :return: http config as dict (keys: [http_proxy, creds, http_timeout])
        :rtype: dict

        """
        return self.__http_config

    @property
    def cache_engine(self):
        """
        Property for __cache_engine

        :return: the Cache System Engine
        :rtype: Cache.ArtifactoryCacheManager

        """
        return self.__cache_engine

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor that always returns the same instance of ArtifactManager

        """
        EquipmentBase.__init__(self, name, model, eqt_params)

        self.__http_config = dict(proxy='system',
                                  creds='anonymous',
                                  http_timeout=10)

        self.__bench_params = bench_params
        self.__cache_size = DEFAULT_MAX_CACHE
        self.__artifact_root_uri = DEFAULT_ARTIFACT_URI
        self.__cache_artifacts_path = DEFAULT_CACHE_ARTIFACTS_PATH

        self.__cache_engine = None
        self.__http_downloader = None

    def _download2cache(self, resp, a_name):
        """

        :param resp: :class:`requests.Response <requests.Response>` (:mod:`requests`)
        :type resp: HTTPResponse

        :param a_name: Artifact's name
        :type a_name: str

        :return: A Cached Artifact instance
        :rtype: Cache.CachedArtifact

        :raises: AcsBaseException.OPERATION_FAILED
                 AcsToolException.HOST_OPERATION_TIMEOUT,
                 AcsConfigException.INVALID_PARAMETER,
                 ArtifactoryCacheManager.Error,
                 IOError,
                 OSError,
                 WindowsError (if windows platform)

        """
        cache_exc = self.__cache_engine.Error
        try:
            _, _, local_path = self.__http_downloader.download(resp)
            if not path.isfile(local_path):
                raise AcsToolException(AcsToolException.OPERATION_FAILED,
                                       "Downloaded Artifact {0} with path: "
                                       "{1} is not VALID!".format(a_name, local_path))
            art = self.__cache_engine.add(a_name, local_path)
        except (SysErrors, cache_exc):
            # We ensure that if any exception occur, we wrapped it into more detailed one properly
            msg = ("Exception occurred while Downloading "
                   "to Cache remote Artifact {0} - "
                   "Detailed exception below:\n{1}".format(a_name, traceback.format_exc()))
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        return art

    def _get_artifact_from_local_host(self, artifact_name, artifact_uri):
        """
        Copy an artifact from local host.

        :param artifact_uri: uri to the artifact to retrieved
        :type  artifact_uri: str

        :param artifact_name: the name of artifact
        :type  artifact_name: str

        :return: path to the artifact
        :rtype: CachedArtifact

        """
        artifact = self.cache_engine.get(artifact_name)
        current_local_path = path.normpath(r'{0}/{1}'.format(path.abspath(artifact_uri), artifact_name))
        current_local_hash = compute_file_hash(current_local_path)
        destination_in_cache = path.normpath(r'{0}/{1}'.format(self.__cache_artifacts_path, artifact_name))
        # We've got a local artifact in Cache
        if artifact:
            # Are they identical?
            if artifact.hash != current_local_hash:
                try:
                    self.logger.debug('Updating Cache artifact `{0}` with newer one ``{1}``')
                    # No, update it
                    shutil.copy2(current_local_path, destination_in_cache)
                    # Updating the CachedArtifact instance
                    artifact.update(hash=current_local_hash)
                except (shutil.Error, SysErrors):
                    # Failed to update Artifact, but previous one still usable
                    self.logger.warning('Local Artifactory could not be copied in Cache!'
                                        'However, Previous Artifact ``{0}`` has been found in Cache'
                                        'BE AWARE: Use of this Artifact might cause '
                                        'unexpected results'.format(artifact.value))
        else:
            # The artifact is not in Cache, adding it
            try:
                shutil.copy2(current_local_path, destination_in_cache)
                artifact = self.cache_engine.add(artifact_name)
            except (shutil.Error, SysErrors):
                raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, 'Artifact: ``{0}`` is '
                                                                            'neither reachable from local host, '
                                                                            'nor from Cache!'.format(artifact_name))
        return artifact

    def _get_artifact_from_http_server(self, artifact_name, artifact_uri, transfer_timeout):
        """
        Download an artifact from http server.

        :param artifact_uri: uri to the artifact to retrieved
        :type  artifact_uri: str

        :param artifact_name: the name of artifact
        :type  artifact_name: str

        :param transfer_timeout: timeout to download the artifact
        :type  transfer_timeout: int

        :return: path to the artifact
        :rtype: str

        :raise: AcsConfigException.FILE_NOT_FOUND if artifact could not be found

        """
        dest_file = os.path.join(self.__cache_artifacts_path, os.path.normpath(artifact_name))
        if not os.path.exists(os.path.dirname(dest_file)):
            os.makedirs(os.path.dirname(dest_file))

        artifact = self.cache_engine.get(artifact_name)
        url = self._compute_remote_uri(artifact_name, artifact_uri)
        self.__http_downloader = http_downloader = HttpDownloaderUtil(url=url,
                                                                      destination=dest_file,
                                                                      override=True,
                                                                      download_timeout=transfer_timeout,
                                                                      logger=self.logger,
                                                                      **self.__http_config)
        # check args
        response = http_downloader.init()

        # We 've got a local artifact
        if artifact:
            # We got response from the remote host?
            if response:
                remote_md5 = http_downloader.get_md5(response)
                # We've got same file version ?
                if remote_md5 != artifact.hash:
                    # We must update file
                    artifact = self._download2cache(response, artifact_name)
                    artifact.update(hash=remote_md5)
                else:
                    self.logger.info('Artifact {0} is already available in cache: '
                                     '{1} | md5 {2}'.format(artifact.key, artifact.value, artifact.hash))
            else:
                # We could not fetch remote host, but we've got a local artifact
                self.logger.warning('Remote Artifactory could not be fetched!\n'
                                    'However, Artifact ``{0}`` is present in Cache\n'
                                    'BE AWARE: Use of this Artifact might cause '
                                    'unexpected results'.format(artifact.value))
        else:
            # We do not have a local artifact, we need to download it and add it to cache
            #  We got response from the remote host?
            if response:
                artifact = self._download2cache(response, artifact_name)
            else:
                raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND,
                                         'Artifact: ``{0}`` is '
                                         'neither reachable from remote host, '
                                         'nor from Cache!'.format(artifact_name))
        return artifact

    def init(self):
        """
        initialize attributes

        """
        self.logger.info("Initialization")
        # source uri
        if self.__bench_params.has_parameter('URI'):
            self.__artifact_root_uri = self.__bench_params.get_param_value('URI')
        else:
            error_msg = "No URI specified for ARTIFACT_MANAGER"
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)
        # cache folder
        if self.__bench_params.has_parameter('cache_folder'):
            self.__cache_artifacts_path = self.__bench_params.get_param_value('cache_folder')
            try:
                if not path.isdir(self.__cache_artifacts_path):
                    folder_created = False
                     # If the folder does not exist, try to create it
                    os.makedirs(self.__cache_artifacts_path)
                    self.logger.info("Temporaries cache Files will be stored into folder: %s"
                                     % str(self.__cache_artifacts_path))
                    folder_created = True
                else:
                    folder_created = True
            except Exception as ex:  # pylint: disable=W0703
                self.logger.error("Error during creation of cache_folder ARTIFACT_MANAGER (%s)" % str(ex))
                folder_created = False

            # If failure at folder creation, raise an exception
            if not folder_created:
                error_msg = ("Wrong cache_folder ARTIFACT_MANAGER parameter's : "
                             "{0} folder does not exist".format(self.__cache_artifacts_path))
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # cache max size
        self.__cache_size = int(self.__bench_params.get_param_value('max_cache_size',
                                                                    DEFAULT_MAX_CACHE)) * Util.Measurements.Data.MB
        if self.__cache_size < 0:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     'The `MAX_CACHE_SIZE` must be a POSITIVE value!')

        # http config
        self.__http_config["proxy"] = self.__bench_params.get_param_value('proxy', 'system')
        self.__http_config["http_timeout"] = int(self.__bench_params.get_param_value('http_timeout', 10))
        self.__http_config["creds"] = self.__bench_params.get_param_value('creds', 'anonymous')

        self.__cache_engine = ArtifactoryCacheManager(self.__cache_artifacts_path,
                                                      self.logger,
                                                      max_size_in_bytes=self.__cache_size)

    def get_artifact(self, artifact_name, artifact_root_uri="", transfer_timeout=60 * 10):
        """
        Retrieve an artifact on the local host

        :param artifact_name: the name of artifact (eg. its path, relative to root uri)
        :type  artifact_name: str

        :param artifact_root_uri: the uri of artifact source. Use it to override URI from bench param.
        :type  artifact_root_uri: str

        :param transfer_timeout: timeout to transfer the artifact on local host
        :type  transfer_timeout: int

        :return: path to the downloaded artifact
        :rtype: str

        """
        if not artifact_root_uri:
            artifact_root_uri = self.__artifact_root_uri

        is_local = not HttpDownloaderUtil.is_http_uri(artifact_root_uri)
        logical_art_path = path.join(self.__cache_artifacts_path, path.normpath(artifact_name))

        if not path.exists(path.dirname(logical_art_path)):
            os.makedirs(path.dirname(logical_art_path))

        if is_local:
            artifact = self._get_artifact_from_local_host(artifact_name, artifact_root_uri)
        else:
            artifact = self._get_artifact_from_http_server(artifact_name, artifact_root_uri, transfer_timeout)

        return artifact.value

    def unzip_artifact(self, zip_path, filters=None, keep_filtered=False, extract_to=""):
        """"
        Unzip zipfile into output directory. Files that are unzip can be filtered.

        :param zip_path: zipfile path to extract
        :type  zip_path: str.

        :param filters: patterns (as str) used to filter src_dir content.
                        Patterns ending with '/' are used to filter directories, other patterns are used for files.
                        Patterns can include wildcard (*).
                        Example : ['*.py', '.gitkeep', '*android*/', 'doc/] will filter all python files,
                        all .gitkeep files, all directories containing 'android' in their name,
                        and all 'doc' directories
        :type  filters: list

        :param keep_filtered: If True, only files matching filters will be unzip.
                              If False, files matching filters are ignored, all other are unzip.
        :type  keep_filtered: bool

        :type extract_to: str
        :param extract_to: path where unzip the file.
                           if leave blank, the file will be extract at the same place of the zip.

        :return: Unzip file location with a '/' at the end or False
                the trailing slash is very important because based on its presence in the file path returned,
                only the contents of the archive will be pushed to the device and not the parent folder as well
        :rtype: string or boolean
        """
        logging.getLogger("ARTIFACT_MGMT").info("Unzip %s" % zip_path)
        if os.path.splitext(zip_path)[-1] in ZIP_EXT_SUPPORTED:
            zip_file = zipfile.ZipFile(zip_path, 'r')
            if zip_file:
                if extract_to == "":
                    zip_output = zip_path.strip(".zip") + "/"
                else:
                    zip_output = extract_to
                zip_file.extractall(zip_output)
                zip_file.close()
                return zip_output
            else:
                return False
        else:
            logging.getLogger("ARTIFACT_MGMT").warning("%s is not a supported zip file" % zip_path)
            return False
