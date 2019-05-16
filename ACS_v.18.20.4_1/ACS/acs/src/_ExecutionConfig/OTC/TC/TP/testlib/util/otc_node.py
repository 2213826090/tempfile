#!/usr/bin/env python
#coding:utf-8
'''
Created on Jun 26, 2014

@author: yongga1x
'''
import time
from uiautomator import JsonRPCError

class OTCNode():
    '''
    OTCNode , It is father of all node in devices
    u should run OTCNode.findNode() first to init the node
    if you want to need to use uiautomator, you can use OTCNode.node to do it
    '''
    def __init__(self, device):
        self.d = device
        self.node = None

    def __getNode__(self , **kwargs):
        try:
            self.node = self.d(**kwargs)
        except JsonRPCError:
            self.node = None
        return self.node

    def findNode(self , expectation = True ,**kwargs):
        '''
        if u want to find it appear, should input expectation = True
        if u want to find it disappear, should input expectation = False
        return autotest.base.OtcNode.OtcNode object
        '''
        self.node = None
        error_msg = ""
        if expectation:
            for i in range(5):
                self.node = self.__getNode__(**kwargs)
                for key in kwargs:
                    error_msg = "self.d(%s=%s) not find!" % (key, kwargs[key])
                if self.node:
                    break
                else:
                    print "not find node"
                    time.sleep(2)
            assert self.node.exists, error_msg
        else:
            self.node = self.__getNode__(**kwargs)
        return self.node

    def getNodeChild(self ,node , **kwargs):
        return node.child(**kwargs)

    def getNodeText(self):
        try:
            if self.node:
                tx = self.node.text
                if tx:
                    tx = tx.decode('utf-8')
                    return tx
        except JsonRPCError:
            return None

    def getContentDescription(self):
        if self.node:
            try:
                return self.node.contentDescription
            except JsonRPCError:
                return ""

    def click(self):
        '''
        Usage:
        n.click()  # click on the center of the ui object
        u should run OTCNode.findNode() first to init the node
        '''
        if self.node:
            self.node.click()

    def exists(self):
        '''
        check if the object exists in current window.
        u should run OTCNode.findNode() first
        '''
        if self.node:
            return self.node.exists

    def long_click(self):
        if self.node:
            self.node.long_click()

    def set_text(self, text):
        if self.node:
            return self.node.set_text(text)

    def clear_text(self, text):
        if self.node:
            return self.node.clear_text(text)

    def child(self, **kwargs):
        '''
        return autotest.utils.uiautomator.AutomatorDeviceObject object
        '''
        if self.node:
            return self.node.child(**kwargs)

    def child_by_text(self, txt, **kwargs):
        '''
        return autotest.utils.uiautomator.AutomatorDeviceObject object
        '''
        if self.node:
            return self.node.child_by_text(txt, **kwargs)

    def child_by_description(self, txt, **kwargs):
        '''
        return autotest.utils.uiautomator.AutomatorDeviceObject object
        '''
        if self.node:
            return self.node.child_by_description(txt, **kwargs)

    def child_by_instance(self, inst, **kwargs):
        '''
        return autotest.utils.uiautomator.AutomatorDeviceObject object
        '''
        if self.node:
            return self.node.child_by_instance(inst, **kwargs)

    def count(self):
        if self.node:
            return self.node.count

    def getBounds(self):
        if self.node:
            return self.node.bounds