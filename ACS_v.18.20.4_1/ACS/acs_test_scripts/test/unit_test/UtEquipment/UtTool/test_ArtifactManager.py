import os
import time

from mock import patch, PropertyMock

from acs_test_scripts.Equipment.Tool.ArtifactManager import ArtifactManager as Mgr
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException
from unit_test_fwk.UTestBase import UTestBase
from UtilitiesFWK import Caching
from UtilitiesFWK.Utilities import BenchConfigParameters


class ArtifactManagerTestCase(UTestBase):

    def setUp(self):
        UTestBase.setUp(self)

        self.art_makedirs = patch("acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.os.makedirs")
        self.art_remove = patch("acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.os.unlink")
        self.art_copyfile = patch("acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.shutil.copyfile")
        self.art_copystart = patch("acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.shutil.copystat")

        self.mocked_art_makedirs = self.art_makedirs.start()
        self.mocked_art_remove = self.art_remove.start()

        self.art_copyfile.start()
        self.art_copystart.start()

        self.__cache_folder = os.path.dirname(__file__)

    def tearDown(self):
        UTestBase.tearDown(self)

        #self.compute_file_hash.stop()
        self.art_makedirs.stop()
        self.art_remove.stop()
        self.art_copyfile.stop()
        self.art_copystart.stop()

    def create_artifacts_mgr(self, name="ARTIFACT_MANAGER", config=None, local=0):
        """

        :param name: ArtifactManager's name
        :type name: str

        :param config: a configuration dictionary
        :type config: dict

        :return: the initialized ArtifactManager
        :rtype: ArtifactManager

        """
        if not isinstance(config, dict):
            config = config or {
                "URI": {"value": self.__cache_folder if local else "http://test"},
                "cache_folder": {"value": self.__cache_folder}
            }
        mgr = Mgr.ArtifactManager(name, "ArtifactManager", {}, BenchConfigParameters(config))
        mgr.init()
        return mgr

    def test_compute_artifact_uri_ok_url(self):
        art_manager = Mgr.ArtifactManager("ARTIFACT_MANAGER", "ArtifactManager", dict(), dict())
        http_src = "https://mcg-depot.intel.com/artifactory/acs_test_artifacts/"
        artifact_uri = art_manager._compute_remote_uri("media/dalida.mp3", http_src)
        self.assertEqual(artifact_uri,
                         "https://mcg-depot.intel.com/artifactory/acs_test_artifacts/media/dalida.mp3")

    def test_compute_init_ko_no_uri(self):
        with self.assertRaises(AcsConfigException):
            self.create_artifacts_mgr(config={})

    def test_compute_artifact_uri_ko_empty_source(self):
        art_manager = Mgr.ArtifactManager("ARTIFACT_MANAGER", "ArtifactManager", dict(), dict())
        with self.assertRaises(AcsConfigException):
            art_manager._compute_remote_uri("media/dalida.mp3",	"")

    def test_compute_artifact_uri_ko_wrong_http_url_format(self):
        art_manager = Mgr.ArtifactManager("ARTIFACT_MANAGER", "ArtifactManager", dict(), dict())
        with self.assertRaises(AcsConfigException):
            art_manager._compute_remote_uri("media/dalida.mp3",	"http:/google.com")

    def test_compute_artifact_uri_ko_wrong_https_url_format(self):
        art_manager = Mgr.ArtifactManager("ARTIFACT_MANAGER", "ArtifactManager", dict(), dict())
        with self.assertRaises(AcsConfigException):
            art_manager._compute_remote_uri("media/dalida.mp3",	"https:/google.com")

    def test_compute_artifact_uri_ko_local_file_uri_exist(self):
        art_manager = Mgr.ArtifactManager("ARTIFACT_MANAGER", "ArtifactManager", dict(), dict())
        with self.assertRaises(AcsConfigException):
            # local source artifact MUST be a dict
            art_manager._compute_remote_uri("media/dalida.mp3", __file__)

    def test_compute_artifact_uri_ko_dir_source_exist(self):
        art_manager = Mgr.ArtifactManager("ARTIFACT_MANAGER", "ArtifactManager", dict(), dict())
        with self.assertRaises(AcsConfigException):
            # local source artifact dir ok
            # but artifact in this folder does not exist!
            art_manager._compute_remote_uri("doesnotexist", self.__cache_folder)

    def test_compute_artifact_uri_ok_dir_source_exist(self):
        art_manager = Mgr.ArtifactManager("ARTIFACT_MANAGER", "ArtifactManager", dict(), dict())
        # can't use hardcoded value here about filename, __file__ has random return value extension .pyc or .py
        filename = os.path.basename(__file__)
        with self.assertRaises(AcsConfigException):
            artifact_uri = art_manager._compute_remote_uri("../UtTool/{0}".format(filename), self.__cache_folder)

    def test_compute_init_ok_as_default_no_config_except_uri(self):
        art_manager = self.create_artifacts_mgr(config={"URI": {"value": "http://test"}})

        self.assertEqual("http://test", art_manager.artifact_root_uri)
        self.assertEqual(Mgr.DEFAULT_MAX_CACHE * 1024 ** 2, art_manager.cache_size)
        self.assertEqual(Mgr.DEFAULT_CACHE_ARTIFACTS_PATH, art_manager.cache_artifacts_path)

    def test_compute_init_config_parameter_ok(self):
        max_cache_size = 1024 ** 2 * 10000
        my_config = {"URI": {"value": "http://test"},
                     "cache_folder": {"value": os.path.dirname(__file__)},
                     "max_cache_size": {"value": "10000"},
                     "proxy": {"value": 'my_proxy'},
                     "http_timeout": {"value": '25'}}

        art_manager = self.create_artifacts_mgr(config=my_config)

        self.assertEqual("http://test", art_manager.artifact_root_uri)
        self.assertEqual(max_cache_size, art_manager.cache_size)
        self.assertEqual(os.path.dirname(__file__), art_manager.cache_artifacts_path)
        self.assertEqual('my_proxy', art_manager.http_config['proxy'])
        self.assertEqual(25, art_manager.http_config['http_timeout'])

    def test_compute_init_config_ko_wrong_cache_folder(self):
        my_config = {"cache_folder": {"value": __file__}}
        with self.assertRaises(AcsConfigException):
            self.create_artifacts_mgr(config=my_config)

    def test_retrieve_artifact_in_cache_ok_in_cache(self):
        file_name = os.path.basename(__file__)
        art_manager = self.create_artifacts_mgr()
        art_manager.cache_engine.add(file_name)
        artifact = art_manager.cache_engine.get(file_name)
        self.assertEqual(artifact.value, __file__)

    @patch.object(Caching, 'compute_file_hash')
    @patch.object(Mgr, 'compute_file_hash')
    @patch.object(Caching.CachedArtifact, 'hash', new_callable=PropertyMock)
    @patch.object(Caching.os, 'walk')
    @patch.object(Caching.path, 'isfile')
    @patch.object(Caching.os, 'stat')
    @patch.object(Mgr.path, 'isdir')
    def test_get_artifact_from_local_host_ok_retrieved_in_cache(self, mocked_isdir,
                                                                mocked_os_stat, mocked_isfile, mocked_walk,
                                                                mock_hash, mock_cfh_art, mock_cfh_caching):
        source_folder = os.path.dirname(__file__)
        my_config = {"URI": {"value": os.path.abspath('.')},
                     "cache_folder": {"value": source_folder}}
        art_manager = self.create_artifacts_mgr(config=my_config)

        mocked_walk.return_value = ((root, dirs, files) for root, dirs, files in [
            (source_folder, [], ["/available/here/my_file"])
        ])

        mocked_os_stat.return_value = type('FakeStat', (object,), {
            'st_mtime': time.time(),
            'st_size': 500
        })

        mocked_isfile.return_value = 1
        mocked_isdir.return_value = 1

        local_file = art_manager.get_artifact("/available/here/my_file")
        expected_path = os.path.normpath(r'{0}{2}{1}'.format(source_folder, "/available/here/my_file", os.sep))
        self.assertEqual(expected_path, local_file)


    @patch.object(Mgr, 'compute_file_hash')
    @patch.object(Caching.os, 'walk')
    def test_get_artifact_from_local_host_ok_not_retrieved(self, mocked_walk, mock_cfh_art):
        """
        ArtifactoryCacheManager::

            Mocking `root, dirs, files in os.walk(self.caching_dir)` to emulate
            :meth:`ArtifactoryCacheManager._load` called while initialized.

        """
        mocked_walk.return_value = ((root, dirs, files) for root, dirs, files in [
            (os.path.dirname(__file__), [], [os.path.basename(__file__)])
        ])

        source_folder = '.'
        cache_folder = os.path.dirname(__file__)
        file_name = os.path.basename(__file__)
        my_config = {"URI": {"value": source_folder},
                     "cache_folder": {"value": cache_folder}}

        art_manager = self.create_artifacts_mgr(config=my_config)

        local_file = art_manager.get_artifact(file_name, source_folder)

        self.assertEqual(local_file, __file__)

    @patch("acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.HttpDownloaderUtil")
    @patch.object(Caching.os, 'stat')
    @patch.object(Mgr.path, 'isdir')
    @patch.object(Caching.path, 'isfile')
    def test_get_artifact_from_cache_same_hash_in_both(self, mocked_isfile, mocked_isdir, mocked_os_stat, mocked_http):
        source_folder = os.path.join('.')
        my_config = {"URI": {"value": source_folder},
                     "cache_folder": {"value": self.__cache_folder}}

        mocked_isdir.return_value = 1
        mocked_isfile.return_value = 1
        mocked_os_stat.return_value = type('FakeStat', (object,), {
            'st_mtime': time.time(),
            'st_size': 500
        })

        filename = os.path.basename(__file__)
        art_manager = self.create_artifacts_mgr(config=my_config)
        mocked_http.return_value.get_md5.return_value = Caching.compute_file_hash(__file__)

        local_file = art_manager.get_artifact(filename, "http://my_server/{0}".format(filename), 10)
        self.assertEqual(local_file, os.path.join(self.__cache_folder, filename))

    @patch.object(Caching, 'compute_file_hash')
    @patch("acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.HttpDownloaderUtil")
    @patch.object(Caching.os, 'stat')
    @patch.object(Caching.path, 'isfile')
    @patch.object(Mgr.path, 'isdir')
    def test_get_artifact_from_http_server_ok_not_retrieved_in_cache(self, mocked_isdir, mocked_isfile, mocked_os_stat,
                                                                     mock_http, mock_compute_file_hash):
        source_folder = os.path.join('.')
        cache_folder = os.path.dirname(__file__)
        my_config = {"URI": {"value": source_folder},
                     "cache_folder": {"value": cache_folder}}

        mocked_os_stat.return_value = type('FakeStat', (object,), {
            'st_mtime': time.time(),
            'st_size': 500
        })

        mocked_isfile.return_value = 1
        mocked_isdir.return_value = 1

        art_manager = self.create_artifacts_mgr(config=my_config)

        mock_http.return_value.download.return_value = (0, "no error", os.path.join(self.__cache_folder, "my_artifact"))
        mock_http.return_value.get_md5.return_value = '123456'

        local_file = art_manager.get_artifact("my_artifact", "http://my_server/my_artifact", 10)
        self.assertEqual(local_file, os.path.join(cache_folder, "my_artifact"))


    @patch.object(Mgr, 'compute_file_hash')
    @patch("acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.HttpDownloaderUtil")
    def test_get_artifact_from_http_server_ko_not_retrieved_in_cache(self, mocked_http, mock_compute_file_hash):
        source_folder = os.path.join('.')
        cache_folder = os.path.dirname(__file__)
        my_config = {"URI": {"value": source_folder},
                     "cache_folder": {"value": cache_folder}}

        art_manager = self.create_artifacts_mgr(config=my_config)

        mocked_http.return_value.download.return_value = (-1, "error", "")
        mocked_http.return_value.get_md5.return_value = '123456'

        with self.assertRaises(AcsToolException):
            art_manager.get_artifact("my_artifact", "http://my_server", 10)

    @patch("acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.HttpDownloaderUtil")
    @patch.object(Caching.os, 'stat')
    @patch.object(Caching.path, 'isfile')
    @patch.object(Mgr.path, 'isdir')
    def test_get_artifact_file_does_not_exists_must_raise(self, mocked_isdir,
                                                          mocked_isfile, mocked_os_stat, mocked_http):

        mocked_os_stat.return_value = type('FakeStat', (object,), {
            'st_mtime': time.time(),
            'st_size': 500
        })

        mocked_isdir.return_value = 1
        mocked_isfile.return_value = 0

        artifact_name = "/my/file/doest/not/exist"
        cache_folder = os.path.join(os.path.dirname(__file__), '..')
        my_config = {"URI": {"value": "http://test"},
                     "cache_folder": {"value": cache_folder}}

        art_manager = self.create_artifacts_mgr(config=my_config)
        cache_mgr = art_manager.cache_engine

        with self.assertRaises(cache_mgr.Error):
            cache_mgr.add(artifact_name)

    @patch("acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.HttpDownloaderUtil")
    def test_retrieve_artifact_in_cache_with_hash_mismatch_updated_artifact_must_be_downloaded(self, mocked_http):

        filename = os.path.basename(__file__)
        mocked_http.return_value.get_md5.return_value = '#222222222'
        mocked_http.return_value.download.return_value = (0, "no error", os.path.join(self.__cache_folder, filename))
        # use above folder as cache
        art_manager = self.create_artifacts_mgr()
        # Cache is empty at this state
        # can't use hardcoded value here about filename, __file__ has random return value extension .pyc or .py
        # artifact must be not be retrieved because not the same md5
        artifact_path = art_manager.get_artifact(filename)
        self.assertEqual(artifact_path, __file__)

    @patch("acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.HttpDownloaderUtil")
    @patch.object(Mgr.ArtifactManager, 'logger')
    def test_retrieve_artifact_in_cache_but_out_dated_no_connection_found_user_warning_should_appear(self,
                                                                                                     mocked_logger,
                                                                                                     mocked_http):
        filename = os.path.basename(__file__)
        mocked_http.return_value.get_md5.return_value = '#222222222'
        mocked_http.return_value.init.return_value = None
        mocked_http.return_value.download.return_value = (0, "no error", os.path.join(self.__cache_folder, filename))
        # use above folder as cache
        art_manager = self.create_artifacts_mgr()
        # Cache is empty at this state
        # can't use hardcoded value here about filename, __file__ has random return value extension .pyc or .py
        # artifact must be not be retrieved because not the same md5
        artifact_path = art_manager.get_artifact(filename)
        self.assertEqual(artifact_path, __file__)

        self.assertEqual(1, mocked_logger.warning.call_count)
        self.assertIsNotNone(mocked_logger.warning.call_args)
        self.assertTrue('BE AWARE:' in mocked_logger.warning.call_args[0][0])

    @patch.object(Caching.CachedArtifact, 'hash', new_callable=PropertyMock)
    @patch.object(Mgr.ArtifactManager, 'logger')
    @patch('acs_test_scripts.Equipment.Tool.ArtifactManager.ArtifactManager.shutil.copyfile')
    def test_local_cache_different_hash_artifact_must_be_updated_but_exception_occurred(self, mocked_copyfile,
                                                                                        mocked_logger,
                                                                                        mocked_hash):

        mocked_hash.return_value = '#FAKE#MD5#HASH'
        mocked_copyfile.return_value = None
        mocked_copyfile.side_effect = IOError

        filename = os.path.basename(__file__)
        art_manager = self.create_artifacts_mgr()
        artifact_path = art_manager.get_artifact(filename, self.__cache_folder)

        self.assertEqual(artifact_path, __file__)
        self.assertEqual(1, mocked_logger.warning.call_count)
        self.assertIsNotNone(mocked_logger.warning.call_args)
        self.assertTrue('BE AWARE:' in mocked_logger.warning.call_args[0][0])

    @patch.object(Caching.CachedArtifact, 'hash', new_callable=PropertyMock)
    @patch.object(Mgr.ArtifactManager, 'logger')
    def test_local_cache_different_hash_artifact_must_be_updated(self, mocked_logger, mocked_hash):

        mocked_hash.return_value = '#FAKE#MD5#HASH'

        filename = os.path.basename(__file__)
        art_manager = self.create_artifacts_mgr()
        artifact_path = art_manager.get_artifact(filename, self.__cache_folder)

        self.assertEqual(artifact_path, __file__)
        self.assertEqual(1, mocked_logger.debug.call_count)
        self.assertIsNotNone(mocked_logger.debug.call_args)
        self.assertTrue('Updating Cache artifact' in mocked_logger.debug.call_args[0][0])

    def test_local_cache_same_artifact_returned_from_cache(self):
        filename = os.path.basename(__file__)
        art_manager = self.create_artifacts_mgr(local=1)
        artifact_path = art_manager.get_artifact(filename)
        self.assertEqual(artifact_path, __file__)


