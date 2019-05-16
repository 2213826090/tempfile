import os
from testlib.util.repo import Artifactory
from testlib.util.config import TestConfig

ARTIFACTORY_URL = TestConfig().getConfValue(section="artifactory", key="location")

AUDIO_REPO_URL = os.path.join(ARTIFACTORY_URL, "Multimedia_Audio")




def get_app(app):
    arti = Artifactory(AUDIO_REPO_URL)
    return arti.get("app/" + app)


def get_media_content(media):
    arti = Artifactory(AUDIO_REPO_URL)
    return arti.get("media_content/" + media)

def get_other_content(content):
    arti = Artifactory(ARTIFACTORY_URL)
    return arti.get(content)

__all__ = ['get_app', 'get_media_content', "get_other_content"]
