from ErrorHandling.TestEquipmentException import TestEquipmentException


class IINSTANTLogger(object):
    """
        Class IMTBFAnalyzer: virtual interface for mtbf analyzer
    """
    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_server_addr(self):
        """
            Return analysis server address
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
