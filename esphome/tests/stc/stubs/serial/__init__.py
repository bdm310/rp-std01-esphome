"""Just enough of pyserial for importing stcgal.protocols; no port is ever opened."""

PARITY_NONE = "N"
PARITY_EVEN = "E"


class SerialException(Exception):
    pass


class SerialTimeoutException(SerialException):
    pass


class Serial:
    def __init__(self, *args, **kwargs):
        raise SerialException("the test stub cannot open ports")
