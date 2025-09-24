"""Main implementation of DL/T645 protocol and utilities

Usage:

.. code-block:: python

    import serial
    import dlt645


    ser = serial.Serial(
        "/dev/ttyUSB0",
        baudrate=1200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )

    station_addr = dlt645.get_addr(ser)

    # requesting active energy
    frame = dlt645.Frame(station_addr)
    frame.data = "00000000"
    ser.write(frame.dump())
    framedata = dlt645.read_frame(dlt645.iogen(ser))
    # the data will be the full payload (energy valu and data identification)
    print(framedata.data)

    # shorthand function to directly get the active energy value
    dlt645.get_active_energy(station_addr, ser)

"""
from .__meta__ import __version__  # noqa: F401
from typing import Generator, Optional, Union, Dict, Any
from datetime import datetime
from .constants import (
    AWAKEN,
    BROADCAST_ADDR,
    DLT645_2007,
    END,
    FUNCTION_CODES,
    MAIN,
    NO_MORE_DATA,
    RESPONSE_CORRECT,
    START,
)
from .exceptions import FrameChecksumError, FrameFormatError, ReadTimeoutError

b_awaken = AWAKEN.to_bytes(1, byteorder="big")
b_start = START.to_bytes(1, byteorder="big")
b_end = END.to_bytes(1, byteorder="big")


def iogen(flo) -> Generator[bytes, None, None]:
    """Simple data generator for a file-like object, returns bytes one by one.

    :param flo: a file-like object instance
    :yields: individual bytes from the file-like object
    """
    byte = flo.read(1)
    while byte != b"":
        yield byte
        byte = flo.read(1)


def read_frame(readgen: Generator[bytes, None, None]) -> 'Frame':
    """Read a frame from a data generator, return a :class:`Frame` instance.

    :param readgen: a generator returning data one byte at a time
    :return: parsed Frame instance
    :raises ReadTimeoutError: if no data is received within the timeout period
    :raises FrameFormatError: if the received frame format is invalid
    :raises FrameChecksumError: if the received frame checksum is invalid
    """
    framedata = bytearray()
    start_count = 0  # Count of start bytes encountered

    for byte in readgen:
        if byte == b"":
            raise ReadTimeoutError

        if byte == b_awaken:
            continue

        framedata.extend(byte)

        # Count start bytes to help with synchronization
        if byte == b_start:
            start_count += 1
        # Check for end byte to complete frame
        # Need at least two start bytes for a valid frame
        elif byte == b_end and start_count >= 2:
            frame = Frame()
            frame.load(framedata)
            return frame

    # If we exit the loop without finding a complete frame, raise an error
    raise FrameFormatError("Incomplete frame received")


def write_frame(flo, frame: 'Frame', awaken: bool = True) -> None:
    """Write a frame to byte form to be written on a data line

    :param flo: a file-like object instance
    :param frame: a Frame instance to be written
    :param awaken: whether to prefix the message with wake up bytes \
        (defaults to True)
    """
    if awaken:
        payload = 4 * b_awaken + frame.dump()
    else:
        payload = frame.dump()

    flo.write(payload)


class Frame:
    """DL/T645 frame representation with mechanisms to load/dump a frame
    from/into a data byte string.

    :param addr: station address
    :param control: a control code representation
    """

    #: Protocol compatibitilty
    #:
    #: :meta hide-value:
    compat = DLT645_2007
    #: Raw frame data
    #:
    #: :meta hide-value:
    frame: Optional[bytearray] = None
    #: Station address
    #:
    #: :meta hide-value:
    addr: Optional[str] = None
    #: Structure representing the control code portion of a frame in a more
    #: human readable way
    #:
    #: :meta hide-value:
    control: Dict[str, Any] = {
        "direction": MAIN,
        "response": RESPONSE_CORRECT,
        "more": NO_MORE_DATA,
        "function": FUNCTION_CODES[DLT645_2007][
            "READ_DATA"
        ],
    }
    #: data portion of a frame
    #:
    #: :meta hide-value:
    data: Optional[str] = None

    def __init__(
        self,
        addr: Optional[str] = None,
        control: Optional[Dict[str, Any]] = None
    ):
        self.addr = addr
        if control is not None:
            self.control = control

    def __str__(self) -> str:
        if self.frame is None:
            return "Empty Frame"
        return bytetostr(self.frame)

    def load(self, framedata: Union[bytearray, bytes]) -> None:
        """Load a payload into a frame.

        :param framedata: a byte-like object holding a payload
        :raises FrameFormatError: if the frame format is invalid
        :raises FrameChecksumError: if the frame checksum is invalid
        """
        if not framedata or len(framedata) < 12:  # Minimal frame length check
            raise FrameFormatError(f"Frame data is too short ({len(framedata)} bytes)")

        if framedata[0] != START or framedata[7] != START or framedata[-1] != END:
            raise FrameFormatError(
                f"Format error in frame ({framedata})"
            )

        self.frame = bytearray(framedata)
        if not self.is_valid():
            raise FrameChecksumError(f"Checksum error in frame ({framedata})")

        # Parse frame components
        self.addr = bytetostr(load_addr(self.frame[1:7]))
        self.control = load_ctrl(self.frame[8])
        length = self.frame[9]

        # Validate length before slicing
        if 10 + length > len(self.frame) - 2:  # Leave room for checksum and end byte
            raise FrameFormatError(
                f"Frame length field ({length}) exceeds actual data size"
            )

        self.data = bytetostr(load_data(self.frame[10 : 10 + length]))

    def dump(self) -> bytes:
        """Dump a frame as a byte-like object.

        :return: serialized frame data
        """
        if self.addr is None:
            addr = BROADCAST_ADDR
        else:
            addr = dump_addr(self.addr)

        ctrl = dump_ctrl(self.control)
        data = dump_data(self.data)
        length = len(data)
        b_length = length.to_bytes(1, byteorder="big")

        framedata = b_start + addr + b_start + ctrl + b_length + data
        cs = checksum(framedata)
        b_cs = cs.to_bytes(1, byteorder="big")
        return framedata + b_cs + b_end

    @property
    def checksum(self) -> int:
        """Frame checksum

        :return: calculated checksum value
        :raises FrameFormatError: if no frame data is available
        """
        if self.frame is None:
            raise FrameFormatError("No frame data available")

        return checksum(self.frame[:-2])

    def is_valid(self, cs: Optional[int] = None) -> bool:
        """Checksum validation

        :param cs: checksum byte to validate against (defaults to frame's checksum)
        :return: True if checksum is valid, False otherwise
        """
        if self.frame is None:
            return False

        if cs is None:
            cs = self.frame[-2]

        try:
            return self.checksum == cs
        except (IndexError, ValueError):
            return False


def checksum(data):
    """Return the checksum of a byte-like object

    :param bytearray data: a byte-like object containing data to use for the checksum
    """
    return sum(data) & 0xFF


def bytetostr(bdata: Union[bytearray, bytes]) -> str:
    """Convert a byte-like object to a hexadecimal string.

    :param bdata: a byte-like payload
    :return: hexadecimal string representation of the input bytes
    """
    return ''.join([f"{byte:02x}" for byte in bdata])


def load_addr(data):
    """Read an address from a byte-like object, return a byte-like object
    representing the address.

    :param bytearray data: a byte-like payload
    """
    bdata = bytearray(data)
    bdata.reverse()
    return bdata


def dump_addr(addr):
    """Dump an address to a byte-like object.

    :param str addr: a station address
    """
    bdata = bytearray([int(addr[i : i + 2], 16) for i in range(0, len(addr), 2)])
    bdata.reverse()
    return bdata


def load_ctrl(data):
    """Read control code information, return a dict structure representing the
    different control code parts.

    :param bytearray data: a byte-like payload
    """
    ctrl = bytearray(data)[0]
    return {
        "direction": ctrl >> 7,
        "response": ctrl >> 6 & 0b01,
        "more": ctrl >> 5 & 0b001,
        "function": ctrl & 0b00011111,
    }


def dump_ctrl(control):
    """Dump a control code representation to a byte-like object.

    :param dict control: a control code representation
    """
    dir = control["direction"] << 7
    resp = control["response"] << 6
    more = control["more"] << 5
    func = control["function"]
    ctrl = dir + resp + more + func
    return ctrl.to_bytes(1, byteorder="big")


def load_data(data):
    """Read data information, return a byte-like structure representing the
    data.

    :param bytearray data: a byte-like payload
    """
    bdata = bytearray(data)
    bdata.reverse()
    retdata = bytearray()
    for byte in bdata:
        retdata.append((byte + 0x100 - 0x33) % 0x100)

    return retdata


def dump_data(data):
    """Dump a data payload to a byte-like object.

    :param str data: a data payload
    """
    if data is None:
        return b""

    bdata = bytearray([
        (int(data[i : i + 2], 16) + 0x33) % 0x100
        for i in range(0, len(data), 2)
    ])
    bdata.reverse()
    return bdata


def get_addr(flo, r_flo=None) -> str:
    """Utility function to read a station's address, returns the address.

    :param flo: a file-like object instance for write
    :param r_flo: a file-like object instance for read (defaults to flo)
    :return: station address as a hexadecimal string
    """
    if r_flo is None:
        r_flo = flo

    # Create frame to request address using broadcast target address
    frame = Frame()
    frame.control = {
        "direction": MAIN,
        "response": RESPONSE_CORRECT,
        "more": NO_MORE_DATA,
        "function": FUNCTION_CODES[DLT645_2007]["READ_ADDR"]
    }
    write_frame(flo, frame)

    resp = read_frame(iogen(r_flo))
    return resp.addr


def set_addr(addr: str, n_addr: str, flo, r_flo=None) -> str:
    """Utility function to set a station's address.

    :param addr: current station address
    :param n_addr: new station address to set
    :param flo: a file-like object instance for write
    :param r_flo: a file-like object instance for read (defaults to flo)
    :return: response data from the station
    :raises FrameFormatError: if the response from the station is invalid
    """
    if r_flo is None:
        r_flo = flo

    frame = Frame(addr)
    frame.control = {
        "direction": MAIN,
        "response": RESPONSE_CORRECT,
        "more": NO_MORE_DATA,
        "function": FUNCTION_CODES[DLT645_2007]["WRITE_ADDR"]
    }
    frame.data = n_addr
    write_frame(flo, frame)

    resp = read_frame(iogen(r_flo))

    if resp and resp.addr == n_addr:
        return resp.addr
    raise FrameFormatError("Invalid response from the station")


def get_active_energy(addr: str, flo, r_flo=None) -> float:
    """Utility function to directly read active energy field, returns the
    energy value in kWh.

    A file-like object is required for the communication, if 'r_flo' is
    ``None`` then 'flo' will be used for both read and write. This is useful
    when using a ``socketserver.StreamRequestHandler`` that provides different
    file-like objects for read and write.

    :param addr: a station address
    :param flo: a file-like object instance for write
    :param r_flo: a file-like object instance for read (defaults to flo)
    :return: active energy value in kWh
    """
    if r_flo is None:
        r_flo = flo

    frame = Frame(addr)
    # Data identification for active energy
    frame.data = "00000000"
    write_frame(flo, frame)

    resp = read_frame(iogen(r_flo))
    # Test the data identification
    if resp and resp.data and resp.data[-8:] == "00000000":
        return int(resp.data[:-8]) / 100
    return 0.0


def set_time(flo, r_flo=None) -> None:
    """Utility function to set the time of a station.

    A file-like object is required for the communication, if 'r_flo' is
    ``None`` then 'flo' will be used for both read and write.

    :param flo: a file-like object instance for write
    :param r_flo: a file-like object instance for read (defaults to flo)
    """

    if r_flo is None:
        r_flo = flo

    frame = Frame('999999999999')

    frame.control = {
        "direction": MAIN,
        "response": RESPONSE_CORRECT,
        "more": NO_MORE_DATA,
        "function": FUNCTION_CODES[DLT645_2007]["BROADCAST_TIME"]
    }

    frame.data = datetime.now().strftime("%y%m%d%H%M%S")
    write_frame(flo, frame)
