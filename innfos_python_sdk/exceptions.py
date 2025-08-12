"""
Custom exceptions for the INNFOS Python SDK
"""

class GluonError(Exception):
    """Base exception for all INNFOS Gluon SDK errors"""
    pass

class ProtocolError(GluonError):
    """Exception raised for protocol-related errors"""
    pass

class ActuatorError(GluonError):
    """Exception raised for actuator-related errors"""
    pass

class CommunicationError(GluonError):
    """Exception raised for communication errors"""
    pass