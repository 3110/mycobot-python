from abc import ABCMeta, abstractmethod
from collections import Iterable
import math
import serial
import six
import struct
import time

ATOM_FIRMWARE_VERSION = "2.6"
DUMP_BYTES = 8


class Frame(object):
    HEADER = 0xFE
    FOOTER = 0xFA


class Command(object):
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
    IS_IN_POSITION = 0x2A
    IS_MOVING = 0x2B
    TASK_STOP = 0x29
    JOG_ANGLE = 0x30
    JOG_ABSOLUTE = 0x31
    JOG_COORD = 0x32
    SEND_JOG_INCREMENT = 0x33
    JOG_STOP = 0x34
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
    IS_SERVO_ENABLE = 0x50
    IS_ALL_SERVO_ENABLE = 0x51
    SET_SERVO_DATA = 0x52
    GET_SERVO_DATA = 0x53
    SET_SERVO_CALIBRATION = 0x54
    VOID_JOINT_BRAKE = 0x55
    RELEASE_SERVO = 0x56
    FOCUS_SERVO = 0x57
    SET_PIN_MODE = 0x60
    SET_DIGITAL_OUTPUT = 0x61
    GET_DIGITAL_INPUT = 0x62
    GET_GRIPPER_VALUE = 0x65
    SET_GRIPPER_STATE = 0x66
    SET_GRIPPER_VALUE = 0x67
    SET_GRIPPER_INI = 0x68
    IS_GRIPPER_MOVING = 0x69
    SET_LED = 0x6A


class Angle(object):
    J1 = 1
    J2 = 2
    J3 = 3
    J4 = 4
    J5 = 5
    J6 = 6


class Axis(object):
    X = 1
    Y = 2
    Z = 3
    RX = 4
    RY = 5
    RZ = 6


class CoordSystem(object):
    ANGULAR = 0
    LINEAR = 1


class GripperState(object):
    OPEN = 0
    CLOSE = 1


class PositionType(object):
    ANGLE = 0
    COORDS = 1


class PinMode(object):
    INPUT = 0
    OUTPUT = 1
    INPUT_PULLUP = 2


class PyMyCobotError(Exception):
    """The base exception thrown from this library"""

    pass


class CommandNotFoundError(PyMyCobotError):
    def __init__(self, cmd_id):
        msg = "Command ID: 0x{:02x}".format(cmd_id)
        super(FormatError, self).__init__(msg)


class FormatError(PyMyCobotError):
    def __init__(self, msg, expected=None, actual=None):
        if expected and actual:
            msg = "{}: expected=0x{:02x}, actual=0x{:02x}".format(
                msg, expected, actual
            )
        super(FormatError, self).__init__(msg)


class IllegalCommandError(FormatError):
    pass


class IllegalDataError(FormatError):
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
        return list(v.decode("hex") if six.PY2 else bytes.fromhex(v))

    @staticmethod
    def _angle_to_int(v):
        if ATOM_FIRMWARE_VERSION == "2.3":
            return round(v * (math.pi * 1000 / 180))
        else:
            return round(v * 100)

    @staticmethod
    def _int_to_angle(v):
        if ATOM_FIRMWARE_VERSION == "2.3":
            return round(v * (180 / (math.pi * 1000)))
        else:
            return round(v / 100.0, 3)

    @staticmethod
    def _coord_to_int(v):
        return int(v * 10)

    @staticmethod
    def _int_to_coord(v):
        return round(v / 10.0, 2)

    @staticmethod
    def _int_to_radian(v):
        return round(AbstractCommand._int_to_angle(v) * (math.pi / 180), 3)

    @staticmethod
    def _radian_to_int(v):
        return AbstractCommand._angle_to_int(v * (180 / math.pi))

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

    def __init__(self, id, reply_data_len=0):
        self.id = id
        self.reply_data_len = reply_data_len

    def prepare_data(self, data):
        if data is None:
            data = ()
        return self._flatten(data)

    def has_reply(self):
        return self.reply_data_len > 0

    def get_bytes(self, data):
        data = self._flatten(
            [
                Frame.HEADER,
                Frame.HEADER,
                len(data) + 2,
                self.id,
                data,
                Frame.FOOTER,
            ]
        )
        if six.PY2:
            return [ord(c) if type(c) == str else c for c in data]
        else:
            return bytes(data)


class AbstractCommandWithoutReply(AbstractCommand):
    def __init__(self, id):
        super(AbstractCommandWithoutReply, self).__init__(id, 0)


class AbstractCommandWithReply(AbstractCommand):
    def __init__(self, id, reply_data_len):
        super(AbstractCommandWithReply, self).__init__(id, reply_data_len)

    @abstractmethod
    def parse_reply(self, data):
        raise NotImplementedError()

    def parse(self, received):
        # print("Received: ")
        # print(received)
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
        if self.reply_data_len != data_frame_len:
            raise IllegalReplyError(
                "Invalid data frame length",
                self.reply_data_len,
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
        super(AbstractCommandWithInt8Reply, self).__init__(id, 0x03)

    def parse_reply(self, data):
        return self.parse_int8(data)


class AbstractCommandWithInt16ListReply(AbstractCommandWithReply):
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
    def __init__(self, is_radian=False):
        super(GetAngles, self).__init__(Command.GET_ANGLES, 0x0E)
        self.is_radian = is_radian

    def parse_value(self, count, v):
        v = super(GetAngles, self).parse_value(count, v)
        if self.is_radian:
            return self._int_to_radian(v)
        else:
            return self._int_to_angle(v)


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
    def __init__(self, is_radian=False):
        super(WriteAngles, self).__init__(Command.WRITE_ANGLES)
        self.is_radian = is_radian

    def prepare_data(self, data):
        data = self._flatten(super(WriteAngles, self).prepare_data(data))
        prepared = []
        for v in data[:-1]:
            if self.is_radian:
                prepared.extend(self.encode_int16(self._radian_to_int(v)))
            else:
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

    def prepare_data(self, data):
        return [data[0] - 1]


class ReleaseServo(AbstractCommandWithoutReply):
    def __init__(self):
        super(ReleaseServo, self).__init__(Command.RELEASE_SERVO)


class FocusServo(AbstractCommandWithoutReply):
    def __init__(self):
        super(FocusServo, self).__init__(Command.FOCUS_SERVO)


class SetPinMode(AbstractCommandWithoutReply):
    def __init__(self):
        super(SetPinMode, self).__init__(Command.SET_PIN_MODE)


class SetDigitalOutput(AbstractCommandWithoutReply):
    def __init__(self):
        super(SetDigitalOutput, self).__init__(Command.SET_DIGITAL_OUTPUT)


class GetDigitalInput(AbstractCommandWithInt8Reply):
    def __init__(self):
        super(GetDigitalInput, self).__init__(Command.GET_DIGITAL_INPUT)


class GetGripperValue(AbstractCommandWithInt16ListReply):
    def __init__(self):
        super(GetGripperValue, self).__init__(Command.GET_GRIPPER_VALUE, 0x04)

    def parse_reply(self, data):
        return super(GetGripperValue, self).parse_reply(data)[0]


class SetGripperState(AbstractCommandWithoutReply):
    def __init__(self):
        super(SetGripperState, self).__init__(Command.SET_GRIPPER_STATE)

    def prepare_data(self, data):
        data = self._flatten(super(SetGripperState, self).prepare_data(data))
        return self._flatten([data[0], data[1]])


class SetGripperValue(AbstractCommandWithoutReply):
    def __init__(self):
        super(SetGripperValue, self).__init__(Command.SET_GRIPPER_VALUE)

    def prepare_data(self, data):
        data = self._flatten(super(SetGripperValue, self).prepare_data(data))
        return self._flatten([self.encode_int16(data[0]), data[1]])


class SetGripperIni(AbstractCommandWithoutReply):
    def __init__(self):
        super(SetGripperIni, self).__init__(Command.SET_GRIPPER_INI)


class IsGripperMoving(AbstractCommandWithBoolReply):
    def __init__(self):
        super(IsGripperMoving, self).__init__(Command.IS_GRIPPER_MOVING)


class SetLED(AbstractCommandWithoutReply):
    def __init__(self):
        super(SetLED, self).__init__(Command.SET_LED)

    def prepare_data(self, data):
        return self.fromhex(data[0])


class IsInPosition(AbstractCommandWithBoolReply):
    def __init__(self):
        super(IsInPosition, self).__init__(Command.IS_IN_POSITION)

    def prepare_data(self, data):
        data = self._flatten(super(IsInPosition, self).prepare_data(data))
        position_type = data.pop()
        if position_type == PositionType.ANGLE:
            return self._flatten(
                [self.encode_int16(self._coord_to_int(v)) for v in data]
            )
        elif position_type == PositionType.COORDS:
            return self._flatten(
                [
                    self.encode_int16(
                        self._coord_to_int(v)
                        if i < 3
                        else self._radian_to_int(v)
                    )
                    for i, v in enumerate(data)
                ]
            )
        else:
            raise IllegalDataError(
                "Illegal Position Type: {}".format(self.position_type)
            )


class IsMoving(AbstractCommandWithBoolReply):
    def __init__(self):
        super(IsMoving, self).__init__(Command.IS_MOVING)


class JogAngle(AbstractCommandWithoutReply):
    def __init__(self):
        super(JogAngle, self).__init__(Command.JOG_ANGLE)


class JogCoord(AbstractCommandWithoutReply):
    def __init__(self):
        super(JogCoord, self).__init__(Command.JOG_COORD)


class JogStop(AbstractCommandWithoutReply):
    def __init__(self):
        super(JogStop, self).__init__(Command.JOG_STOP)


class IsServoEnable(AbstractCommandWithBoolReply):
    def __init__(self):
        super(IsServoEnable, self).__init__(Command.IS_SERVO_ENABLE)


class IsAllServoEnable(AbstractCommandWithBoolReply):
    def __init__(self):
        super(IsAllServoEnable, self).__init__(Command.IS_ALL_SERVO_ENABLE)


class MyCobot:
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
        Command.IS_IN_POSITION: IsInPosition(),
        Command.IS_MOVING: IsMoving(),
        Command.JOG_ANGLE: JogAngle(),
        Command.JOG_COORD: JogCoord(),
        Command.JOG_STOP: JogStop(),
        Command.IS_SERVO_ENABLE: IsServoEnable(),
        Command.IS_ALL_SERVO_ENABLE: IsAllServoEnable(),
        Command.SET_SERVO_CALIBRATION: SetServoCalibration(),
        Command.RELEASE_SERVO: ReleaseServo(),
        Command.FOCUS_SERVO: FocusServo(),
        Command.SET_PIN_MODE: SetPinMode(),
        Command.SET_DIGITAL_OUTPUT: SetDigitalOutput(),
        Command.GET_DIGITAL_INPUT: GetDigitalInput(),
        Command.GET_GRIPPER_VALUE: GetGripperValue(),
        Command.SET_GRIPPER_STATE: SetGripperState(),
        Command.SET_GRIPPER_VALUE: SetGripperValue(),
        Command.SET_GRIPPER_INI: SetGripperIni(),
        Command.IS_GRIPPER_MOVING: IsGripperMoving(),
        Command.SET_LED: SetLED(),
    }

    def __init__(self, port, baudrate=115200, timeout=0.1, debug=False):
        self._serial = serial.Serial(port, baudrate, timeout=timeout)
        self.debug = debug
        self._reset()

    @classmethod
    def get_command(cls, cmd_id):
        if cmd_id in cls.COMMANDS:
            return cls.COMMANDS[cmd_id]
        else:
            raise CommandNotFoundError(cmd_id)

    def get_robot_version(self):
        return self._emit_command(self.get_command(Command.GET_ROBOT_VERSION))

    def get_system_version(self):
        return self._emit_command(self.get_command(Command.GET_SYSTEM_VERSION))

    def power_on(self):
        return self._emit_command(self.get_command(Command.POWER_ON))

    def power_off(self):
        return self._emit_command(self.get_command(Command.POWER_OFF))

    def is_powered_on(self):
        return self._emit_command(self.get_command(Command.IS_POWERED_ON))

    def set_free_move(self):
        return self._emit_command(self.get_command(Command.SET_FREE_MOVE))

    def is_controller_connected(self):
        return self._emit_command(
            self.get_command(Command.IS_CONTROLLER_CONNECTED)
        )

    def get_angles(self):
        cmd = self.get_command(Command.GET_ANGLES)
        cmd.is_radian = False
        return self._emit_command(cmd)

    def get_radians(self):
        cmd = self.get_command(Command.GET_ANGLES)
        cmd.is_radian = True
        return self._emit_command(cmd)

    def set_angle(self, id, angle, speed):
        return self._emit_command(
            self.get_command(Command.WRITE_ANGLE), id, angle, speed
        )

    def set_angles(self, angles, speed):
        cmd = self.get_command(Command.WRITE_ANGLES)
        cmd.is_radian = False
        return self._emit_command(cmd, angles, speed)

    def set_radians(self, angles, speed):
        cmd = self.get_command(Command.WRITE_ANGLES)
        cmd.is_radian = True
        return self._emit_command(cmd, angles, speed)

    def get_coords(self):
        return self._emit_command(self.get_command(Command.GET_COORDS))

    def set_coord(self, axis, coord, speed):
        return self._emit_command(
            self.get_command(Command.WRITE_COORD), axis, coord, speed
        )

    def set_coords(self, coords, speed, mode):
        return self._emit_command(
            self.get_command(Command.WRITE_COORDS), coords, speed, mode
        )

    def is_in_position(self, positions, position_type):
        cmd = self.get_command(Command.IS_IN_POSITION)
        cmd.position_type = position_type
        return self._emit_command(cmd, positions, position_type)

    def is_moving(self):
        return self._emit_command(self.get_command(Command.IS_MOVING))

    def jog_angle(self, joint_no, direction, speed):
        return self._emit_command(
            self.get_command(Command.JOG_ANGLE), joint_no, direction, speed
        )

    def jog_coord(self, coord_no, direction, speed):
        return self._emit_command(
            self.get_command(Command.JOG_COORD), coord_no, direction, speed
        )

    def jog_stop(self):
        return self._emit_command(self.get_command(Command.JOG_STOP))

    def is_servo_enable(self, joint_no):
        return self._emit_command(
            self.get_command(Command.IS_SERVO_ENABLE), joint_no
        )

    def is_all_servo_enable(self):
        return self._emit_command(
            self.get_command(Command.IS_ALL_SERVO_ENABLE)
        )

    def set_servo_calibration(self, joint_no):
        return self._emit_command(
            self.get_command(Command.SET_SERVO_CALIBRATION), joint_no
        )

    def release_servo(self, joint_no):
        return self._emit_command(
            self.get_command(Command.RELEASE_SERVO), joint_no
        )

    def focus_servo(self, joint_no):
        return self._emit_command(
            self.get_command(Command.FOCUS_SERVO), joint_no
        )

    def set_pin_mode(self, pin_no, pin_mode):
        return self._emit_command(
            self.get_command(Command.SET_PIN_MODE), pin_no, pin_mode
        )

    def set_digital_output(self, pin_no, pin_mode):
        return self._emit_command(
            self.get_command(Command.SET_DIGITAL_OUTPUT), pin_no, pin_mode
        )

    def get_digital_input(self, pin_no):
        return self._emit_command(
            self.get_command(Command.GET_DIGITAL_INPUT), pin_no
        )

    def get_gripper_value(self):
        return self._emit_command(self.get_command(Command.GET_GRIPPER_VALUE))

    def set_gripper_state(self, state, speed):
        return self._emit_command(
            self.get_command(Command.SET_GRIPPER_STATE), state, speed
        )

    def set_gripper_value(self, value, speed):
        return self._emit_command(
            self.get_command(Command.SET_GRIPPER_VALUE), value, speed
        )

    def set_gripper_ini(self):
        return self._emit_command(self.get_command(Command.SET_GRIPPER_INI))

    def is_gripper_moving(self):
        return self._emit_command(self.get_command(Command.IS_GRIPPER_MOVING))

    def set_led(self, rgb):
        return self._emit_command(self.get_command(Command.SET_LED), rgb)

    def _emit_command(self, cmd, *data):
        data = cmd.prepare_data(data)
        # self._dump(data, "Prepared:")
        b = cmd.get_bytes(data)
        self._dump(b, "Sending:")
        self._serial.write(b)
        self._serial.flush()
        time.sleep(0.05)
        if cmd.has_reply():
            if self._serial.inWaiting() > 0:
                received = self._serial.read(self._serial.inWaiting())
                self._dump(received, "Received:")
                return cmd.parse(received)
            else:
                raise IllegalReplyError("No reply is found")
        return None

    def _dump_row(self, row, data):
        six.print_("%02x:" % (row * DUMP_BYTES), end=" ")
        n = len(data)
        for i in range(DUMP_BYTES):
            c = ("%02x" % data[i]) if i < n else "  "
            six.print_(c, end=" ")
        for i in range(DUMP_BYTES):
            if i < n:
                c = chr(data[i]) if 32 <= data[i] <= 127 else "."
            else:
                c = " "
            six.print_(c, end="")
        six.print_()

    def _dump(self, data, header=None):
        if not self.debug:
            return
        if header:
            six.print_(header)
        rows = len(data) // DUMP_BYTES
        row = 0
        for row in range(rows):
            self._dump_row(
                row, data[row * DUMP_BYTES : row * DUMP_BYTES + DUMP_BYTES]
            )
        remain = len(data) % DUMP_BYTES
        if remain > 0:
            if row > 0:
                row += 1
            self._dump_row(row, data[(-1 * remain) :])

    def _reset(self):
        while self._serial.inWaiting():
            self._serial.read(self._serial.inWaiting())
