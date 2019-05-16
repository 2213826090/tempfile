#!/usr/bin/env python

class UUIDs:

    A2DP_SOURCE = "0000110A-0000-1000-8000-00805F9B34FB"
    A2DP_SINK = "0000110B-0000-1000-8000-00805F9B34FB"
    HFP_AG = "0000111F-0000-1000-8000-00805F9B34FB"
    HFP_HF = "0000111E-0000-1000-8000-00805F9B34FB"
    HSP_AG = "00001112-0000-1000-8000-00805F9B34FB"
    NAP = "00001116-0000-1000-8000-00805f9b34fb"

    @staticmethod
    def get(cls, name):
        """
        """
        if name == "nap":
            return UUIDs.NAP
        else:
            return None
