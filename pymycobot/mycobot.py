from abc import ABCMeta, abstractmethod
from collections import Iterable
import enum
import serial
import six
import struct
import time


class Frame(enum.IntEnum):
    HEADER = 0xFE
    FOOTER = 0xFA


class Command(enum.IntEnum):
    GET_ROBOT_VERSION = 0x01
    GET_SYSTEM_VERSION = 0x02
    POWER_ON = 0x10
    POWER_OFF = 0x11
    IS_POWERED_ON = 0x12
    SET_FREE_MOVE = 0x13
    IS_CONTROLLER_CONNECTED = 0x14
    READ_NEXT_ERROR = 0x15
    GET_ANGLES = 0x20
    WRITE_ANGLE = 0x21
    WRITE_ANGLES = 0x22
    GET_COORDS = 0x23
    WRITE_COORD = 0x24
    WRITE_COORDS = 0x25
    PROGRAM_PAUSE = 0x26
    IS_PROGRAM_PAUSED = 0x27
    IS_MOVING = 0x2B
    TASK_STOP = 0x29
    JOG_ANGLE = 0x30
    JOG_ABSOLUTE = 0x31
    JOG_COORD = 0x32
    SEND_JOG_INCREMENT = 0x33
    SET_ENCODER = 0x3A
    GET_ENCODER = 0x3B
    SET_ENCODERS = 0x3C
    GET_SPEED = 0x40
    SET_SPEED = 0x41
    GET_FEED_OVERRIDE = 0x42
    SEND_OVERRIDE = 0x43
    GET_ACCELERATION = 0x44
    GET_JOINT_MIN = 0x4A
    GET_JOINT_MAX = 0x4B
    SET_JOINT_MIN = 0x4C
    SET_JOINT_MAX = 0x4D
    IS_SERVO_ENABLED = 0x50
    IS_ALL_SERVO_ENABLED = 0x51
    SET_SERVO_DATA = 0x52
    GET_SERVO_DATA = 0x53
    SET_SERVO_CALIBRATION = 0x54
    VOID_JOINT_BRAKE = 0x55
    SET_PIN_MODE = 0x60
    SET_PIN_DATA = 0x61
    GET_PIN_DATA = 0x62
    SET_GRIPPER_STATE = 0x66
    SET_LED = 0x6A


class Axis(enum.IntEnum):
    X = 1
    Y = 2
    Z = 3
    Rx = 4
    Ry = 5
    Rz = 6


class CoordSystem(enum.IntEnum):
    ANGULAR = 0
    LINEAR = 1


class PyMyCobotError(Exception):
    """The base exception thrown from this library"""

    pass


class FormatError(PyMyCobotError):
    def __init__(self, message, expected=None, actual=None):
        if expected and actual:
            message = f"{message}: expected={expected:#x}, actual={actual:#x}"
        super(FormatError, self).__init__(message)


class IllegalCommandError(FormatError):
    pass


class IllegalReplyError(FormatError):
    pass


@six.add_metaclass(ABCMeta)
class AbstractCommand(object):
    @staticmethod
    def is_frame_header(data, pos):
        return data[pos] == Frame.HEADER and data[pos + 1] == Frame.HEADER

    @staticmethod
    def is_frame_footer(data, pos):
        return data[pos] == Frame.FOOTER

    @staticmethod
    def parse_bool(data):
        return struct.unpack("?", data)[0]

    @staticmethod
    def parse_int8(data):
        return struct.unpack("b", data)[0]

    @staticmethod
    def parse_int16(data):
        return struct.unpack(">h", data)[0]

    @staticmethod
    def encode_int16(data):
        return list(struct.pack(">h", data))

    @staticmethod
    def fromhex(v):
        return v.decode("hex") if six.PY2 else bytes.fromhex(v)

    @staticmethod
    def _angle_to_int(v):
        return round(v * 100)

    @staticmethod
    def _int_to_angle(v, is_radian=False):
        return round(v / 100.0, 3)

    @staticmethod
    def _coord_to_int(v):
        return int(v * 10)

    @staticmethod
    def _int_to_coord(v):
        return round(v / 10.0, 2)

    @staticmethod
    def _flatten(v):
        return [
            e
            for d in v
            for e in (
                AbstractCommand._flatten(d)
                if isinstance(d, Iterable)
                and not isinstance(d, six.string_types)
                else [d]
            )
        ]

    def __init__(self, id):
        self.id = id

    def prepare_data(self, data):
        if data is None:
            data = []
        return data

    def get_bytes(self, data):
        return bytes(
            self._flatten(
                [
                    Frame.HEADER,
                    Frame.HEADER,
                    len(data) + 2,
                    self.id,
                    data,
                    Frame.FOOTER,
                ]
            )
        )


class AbstractCommandWithoutReply(AbstractCommand):
    pass


class AbstractCommandWithReply(AbstractCommand):
    def __init__(self, id, reply_data_frame_length):
        super(AbstractCommandWithReply, self).__init__(id)
        self.reply_data_frame_length = reply_data_frame_length

    @abstractmethod
    def parse_reply(self, data):
        raise NotImplementedError()

    def parse(self, received):
        print("Received: ")
        print(received)
        n_recv = len(received)
        if n_recv == 0:
            raise IllegalReplyError("No reply is found")
        for pos in range(n_recv - 1):
            if self.is_frame_header(received, pos):
                pos += 2
                break
        else:
            raise IllegalReplyError("Frame Header is not found")
        data_frame_len = received[pos]
        if self.reply_data_frame_length != data_frame_len:
            raise IllegalReplyError(
                "Invalid data frame length",
                self.reply_data_frame_length,
                data_frame_len,
            )
        if n_recv - pos < data_frame_len or not self.is_frame_footer(
            received, pos + data_frame_len
        ):
            raise IllegalReplyError(
                "Invalid data frame", data_frame_len, n_recv - pos
            )
        reply_len = data_frame_len - 2  # except cmd and footer
        cmd = received[pos + 1]
        if cmd != self.id:
            raise IllegalReplyError("Invalid Command", self.id, cmd)
        return self.parse_reply(received[pos + 2 : pos + 2 + reply_len])


class AbstractCommandWithInt8Reply(AbstractCommandWithReply):
    def __init__(self, id):
        super(AbstractCommandWithInt8Reply, self).__init__(id, 0x3)

    def parse_reply(self, data):
        return self.parse_int8(data)


class AbstractCommandWithInt16ListReply(AbstractCommandWithReply):
    @abstractmethod
    def parse_value(self, count, v):
        return self.parse_int16(v)

    def parse_reply(self, data):
        parsed = []
        count = 0
        for pos in range(0, len(data), 2):
            parsed.append(self.parse_value(count, data[pos : pos + 2]))
            ++count
        return parsed


class AbstractCommandWithCoordsReply(AbstractCommandWithInt16ListReply):
    def __init__(self, id):
        super(AbstractCommandWithCoordsReply, self).__init__(id, 0x10)

    @abstractmethod
    def parse_value(self, count, v):
        return self._int_to_coord(
            super(AbstractCommandWithCoordsReply, self).parse_value(count, v)
        )


class AbstractCommandWithBoolReply(AbstractCommandWithReply):
    def __init__(self, id):
        super(AbstractCommandWithBoolReply, self).__init__(id, 0x03)

    def parse_reply(self, data):
        return self.parse_bool(data)


# Command Implementations
class GetRobotVersion(AbstractCommandWithInt8Reply):
    def __init__(self):
        super(GetRobotVersion, self).__init__(Command.GET_ROBOT_VERSION)


class GetSystemVersion(AbstractCommandWithInt8Reply):
    def __init__(self):
        super(GetSystemVersion, self).__init__(Command.GET_SYSTEM_VERSION)


class PowerOn(AbstractCommandWithoutReply):
    def __init__(self):
        super(PowerOn, self).__init__(Command.POWER_ON)


class PowerOff(AbstractCommandWithoutReply):
    def __init__(self):
        super(PowerOff, self).__init__(Command.POWER_OFF)


class IsPoweredOn(AbstractCommandWithBoolReply):
    def __init__(self):
        super(IsPoweredOn, self).__init__(Command.IS_POWERED_ON)


class SetFreeMove(AbstractCommandWithoutReply):
    def __init__(self):
        super(SetFreeMove, self).__init__(Command.SET_FREE_MOVE)


class IsControllerConnected(AbstractCommandWithBoolReply):
    def __init__(self):
        super(IsControllerConnected, self).__init__(
            Command.IS_CONTROLLER_CONNECTED
        )


class GetAngles(AbstractCommandWithInt16ListReply):
    def __init__(self):
        super(GetAngles, self).__init__(Command.GET_ANGLES, 0x0E)

    def parse_value(self, count, v):
        return self._int_to_angle(super(GetAngles, self).parse_value(count, v))


class WriteAngle(AbstractCommandWithoutReply):
    def __init__(self):
        super(WriteAngle, self).__init__(Command.WRITE_ANGLE)

    def prepare_data(self, data):
        data = super(WriteAngle, self).prepare_data(data)
        return self._flatten(
            [
                data[0] - 1,
                self.encode_int16(self._angle_to_int(data[1])),
                data[2],
            ]
        )


class WriteAngles(AbstractCommandWithoutReply):
    def __init__(self):
        super(WriteAngles, self).__init__(Command.WRITE_ANGLES)

    def prepare_data(self, data):
        data = self._flatten(super(WriteAngles, self).prepare_data(data))
        prepared = []
        for v in data[:-1]:
            prepared.extend(self.encode_int16(self._angle_to_int(v)))
        prepared.append(data[-1])  # speed
        return prepared


class GetCoords(AbstractCommandWithInt16ListReply):
    def __init__(self):
        super(GetCoords, self).__init__(Command.GET_COORDS, 0x0E)

    def parse_value(self, count, v):
        v = super(GetCoords, self).parse_value(count, v)
        return self._int_to_coord(v) if count < 3 else self._int_to_angle(v)


class WriteCoord(AbstractCommandWithoutReply):
    def __init__(self):
        super(WriteCoord, self).__init__(Command.WRITE_COORD)

    def prepare_data(self, data):
        return self.flatten(
            [
                data[0] - 1,
                self.encode_int16(self._coord_to_int(data[1])),
                data[2],
            ]
        )


class WriteCoords(AbstractCommandWithoutReply):
    def __init__(self):
        super(WriteCoords, self).__init__(Command.WRITE_COORDS)

    def prepare_data(self, data):
        prepared = []
        data = self._flatten(super(WriteCoords, self).prepare_data(data))
        for v in data[:3]:  # x,y,z
            prepared.extend(self.encode_int16(self._coord_to_int(v)))
        for v in data[3:-2]:  # rx,ry,rz
            prepared.extend(self.encode_int16(self._angle_to_int(v)))
        prepared.extend(data[-2:])  # speed, mode
        return prepared


class SetServoCalibration(AbstractCommandWithoutReply):
    def __init__(self):
        super(SetServoCalibration, self).__init__(
            Command.SET_SERVO_CALIBRATION
        )


class SetLED(AbstractCommandWithoutReply):
    def __init__(self):
        super(SetLED, self).__init__(Command.SET_LED)

    def prepare_data(self, data):
        return self.fromhex(data)


class IsMoving(AbstractCommandWithBoolReply):
    def __init__(self):
        super(IsMoving, self).__init__(Command.IS_MOVING)


COMMANDS = {
    Command.GET_ROBOT_VERSION: GetRobotVersion(),
    Command.GET_SYSTEM_VERSION: GetSystemVersion(),
    Command.POWER_ON: PowerOn(),
    Command.POWER_OFF: PowerOff(),
    Command.IS_POWERED_ON: IsPoweredOn(),
    Command.SET_FREE_MOVE: SetFreeMove(),
    Command.IS_CONTROLLER_CONNECTED: IsControllerConnected(),
    Command.GET_ANGLES: GetAngles(),
    Command.WRITE_ANGLE: WriteAngle(),
    Command.WRITE_ANGLES: WriteAngles(),
    Command.GET_COORDS: GetCoords(),
    Command.WRITE_COORD: WriteCoord(),
    Command.WRITE_COORDS: WriteCoords(),
    Command.IS_MOVING: IsMoving(),
    Command.SET_SERVO_CALIBRATION: SetServoCalibration(),
    Command.SET_LED: SetLED(),
}


class MyCobot:
    def __init__(self, port, baudrate=115200, timeout=0.1):
        self._serial = serial.Serial(port, baudrate, timeout=timeout)

    def get_robot_version(self):
        return self._emit_command(COMMANDS[Command.GET_ROBOT_VERSION])

    def get_system_version(self):
        return self._emit_command(COMMANDS[Command.GET_SYSTEM_VERSION])

    def power_on(self):
        return self._emit_command(COMMANDS[Command.POWER_ON])

    def power_off(self):
        return self._emit_command(COMMANDS[Command.POWER_OFF])

    def is_powered_on(self):
        return self._emit_command(COMMANDS[Command.IS_POWERED_ON])

    def set_free_move(self):
        return self._emit_command(COMMANDS[Command.SET_FREE_MOVE])

    def is_controller_connected(self):
        return self._emit_command(COMMANDS[Command.IS_CONTROLLER_CONNECTED])

    def get_angles(self, is_radian=False):
        return self._emit_command(COMMANDS[Command.GET_ANGLES])

    def set_angle(self, id, angle, speed):
        return self._emit_command(
            COMMANDS[Command.WRITE_ANGLE], [id, angle, speed]
        )

    def set_angles(self, angles, speed):
        return self._emit_command(
            COMMANDS[Command.WRITE_ANGLES], [angles, speed]
        )

    def get_coords(self):
        return self._emit_command(COMMANDS[Command.GET_COORDS])

    def set_coord(self, axis, coord, speed):
        return self._emit_command(
            COMMANDS[Command.WRITE_COORD], [axis, coord, speed]
        )

    def set_coords(self, coords, speed, mode):
        return self._emit_command(
            COMMANDS[Command.WRITE_COORDS], [coords, speed, mode]
        )

    def is_moving(self):
        return self._emit_command(COMMANDS[Command.IS_MOVING])

    def set_servo_calibration(self, joint_no):
        return self._emit_command(
            COMMANDS[Command.SET_SERVO_CALIBRATION], joint_no - 1
        )

    def set_led(self, rgb):
        return self._emit_command(COMMANDS[Command.SET_LED], rgb)

    def _emit_command(self, cmd, data=None):
        data = cmd.prepare_data(data)
        print("Prepared:")
        print(data)
        b = cmd.get_bytes(data)
        print("Sending:")
        print(b)
        self._serial.write(b)
        self._serial.flush()
        time.sleep(0.05)
        if isinstance(cmd, AbstractCommandWithReply):
            if self._serial.inWaiting() > 0:
                received = self._serial.read(self._serial.inWaiting())
                return cmd.parse(received)
            else:
                raise IllegalReplyError("No reply is found")
        return None
