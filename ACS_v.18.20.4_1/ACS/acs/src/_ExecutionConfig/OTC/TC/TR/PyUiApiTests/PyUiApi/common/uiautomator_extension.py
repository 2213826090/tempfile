# -*- coding: utf-8 -*-
from bs4 import BeautifulSoup
import sys

reload(sys)
sys.setdefaultencoding("utf-8")


class UiAutomatorExtended:
    @staticmethod
    def get_mapping_using_class(classN):
        '''
        Used for retrieving mapping between textviews and their related status
        ex: The WiFi "On" text and the switch button value associated with it [("On","true")]
        :param classN: the classname of the ui buttons (ex: android.widget.Switch, )
        :return: a list of tuples [(text,value),(text,value),...] where value can be "true", "false" or "text"
        '''
        xml_dump = d.dump()
        modified_xml_string = xml_dump.replace("class", "className")
        soup = BeautifulSoup(modified_xml_string)
        elems = soup.find_all(classname=classN)
        status = []
        text = []
        for elem in elems:
            text.append(UiAutomatorExtended.get_text_view_cousin(elem))
            if elem["checkable"] == "true":
                status.append(elem["checked"].encode('ascii', errors='replace'))
            else:
                status.append(elem["text"].encode('ascii', errors='replace'))
        return zip(text, status)

    @staticmethod
    def get_text_view_cousin(elem):
        if "android.widget.TextView" in str(elem):
            for child in elem.children:
                if str(child) != "\n":
                    if "android.widget.TextView" in str(child):
                        if str(child).count("node") == 2:
                            return child["text"].encode('ascii', errors="replace")
                        else:
                            return UiAutomatorExtended.get_text_view_cousin(child)
                    else:
                        return
        else:
            return UiAutomatorExtended.get_text_view_cousin(elem.parent)

    @staticmethod
    def change_status_for_text(text, classN):
        menu = UiAutomatorExtended.get_mapping_using_class(classN)
        for key, value in enumerate(menu):
            if text == value[0] and d(className=classN)[key].wait.exists(timeout=5000):
                d(className=classN)[key].click()
                return 1
        return "Could not find text %s in list %s" % (text, str(menu))