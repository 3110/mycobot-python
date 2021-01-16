from abc import ABCMeta, abstractmethod
import enum
import serial
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
    GET_ANGLES = 0x20
    WRITE_ANGLE = 0x21
    WRITE_ANGLES = 0x22
    GET_COORDS = 0x23
    IS_MOVING = 0x2B
    SET_SERVO_CALIBRATION = 0x54
    SET_LED = 0x6A


class IllegalReplyError(Exception):
    pass


class AbstractCommand(metaclass=ABCMeta):
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
    def _angle_to_int(v, is_radian=False):
        return round(v * 1000 if is_radian else v * (314 / 18))

    @staticmethod
    def _int_to_angle(v, is_radian=False):
        return v / 1000 if is_radian else v * (18 / 314)

    @abstractmethod
    def id(self):
        raise NotImplementedError()

    @abstractmethod
    def has_reply(self):
        raise NotImplementedError()

    def prepare_data(self, data):
        if data is None:
            data = []
        return data


class AbstractCommandWithoutReply(AbstractCommand):
    def has_reply(self):
        return False


class AbstractCommandWithReply(AbstractCommand):
    @staticmethod
    def is_frame_header(data, pos):
        return data[pos] == Frame.HEADER and data[pos + 1] == Frame.HEADER

    @abstractmethod
    def parse_reply(self, data):
        raise NotImplementedError()

    def has_reply(self):
        return True

    def parse(self, received):
        print("Received: ")
        print(received)
        n_recv = len(received)
        if n_recv == 0:
            return
        for pos in range(n_recv - 1):
            if self.is_frame_header(received, pos):
                break
        else:
            raise IllegalReplyError("Frame Header is not found")
        data_len = received[pos + 2] - 1
        cmd_id = received[pos + 3]
        if cmd_id != self.id():
            raise IllegalReplyError(
                "expected = 0x%02x, actual = 0x%02x" % (self.id(), cmd_id)
            )
        data_pos = pos + 4
        return self.parse_reply(received[data_pos : data_pos + data_len - 1])

    def prepare_data(self, data):
        if data is None:
            data = []
        return data


class AbstractCommandWithInt8Reply(AbstractCommandWithReply):
    def parse_reply(self, data):
        return self.parse_int8(data)


class AbstractCommandWithJointReply(AbstractCommandWithReply):
    @abstractmethod
    def parse_value(self, v):
        return self.parse_int16(v)

    def parse_reply(self, data):
        parsed = []
        for pos in range(0, len(data), 2):
            parsed.append(self.parse_value(data[pos : pos + 2]))
        return parsed


class AbstractCommandWithBoolReply(AbstractCommandWithReply):
    def parse_reply(self, data):
        return self.parse_bool(data)


class GetRobotVersion(AbstractCommandWithInt8Reply):
    def id(self):
        return Command.GET_ROBOT_VERSION


class GetSystemVersion(AbstractCommandWithInt8Reply):
    def id(self):
        return Command.GET_SYSTEM_VERSION


class PowerOn(AbstractCommandWithoutReply):
    def id(self):
        return Command.POWER_ON


class PowerOff(AbstractCommandWithoutReply):
    def id(self):
        return Command.POWER_OFF


class IsPoweredOn(AbstractCommandWithBoolReply):
    def id(self):
        return Command.IS_POWERED_ON


class SetFreeMove(AbstractCommandWithoutReply):
    def id(self):
        return Command.SET_FREE_MOVE


class IsControllerConnected(AbstractCommandWithBoolReply):
    def id(self):
        return Command.IS_CONTROLLER_CONNECTED


class GetAngles(AbstractCommandWithJointReply):
    def __init__(self, is_radian=False):
        self.is_radian = is_radian

    def id(self):
        return Command.GET_ANGLES

    def parse_value(self, v):
        return self._int_to_angle(super().parse_value(v))


class WriteAngle(AbstractCommandWithoutReply):
    def __init__(self, is_radian=False):
        self.is_radian = is_radian

    def id(self):
        return Command.WRITE_ANGLE

    def prepare_data(self, data):
        data = super().prepare_data(data)
        return [
            data[0] - 1,
            *self.encode_int16(self._angle_to_int(data[1], self.is_radian)),
            data[2],
        ]


class WriteAngles(AbstractCommandWithoutReply):
    def __init__(self, is_radian=False):
        self.is_radian = is_radian

    def id(self):
        return Command.WRITE_ANGLES

    def prepare_data(self, data):
        prepared = []
        data = super().prepare_data(data)
        for v in data[:-1]:
            prepared.extend(
                self.encode_int16(self._angle_to_int(v, self.is_radian))
            )
        prepared.append(data[-1])  # speed
        return prepared


class GetCoords(AbstractCommandWithJointReply):
    def id(self):
        return Command.GET_COORDS

    def parse_value(self, v):
        return super().parse_value(v) / 10


class SetServoCalibration(AbstractCommandWithoutReply):
    def id(self):
        return Command.SET_SERVO_CALIBRATION


class SetLED(AbstractCommandWithoutReply):
    def id(self):
        return Command.SET_LED


class IsMoving(AbstractCommandWithBoolReply):
    def id(self):
        return Command.IS_MOVING


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
        cmd = COMMANDS[Command.GET_ANGLES]
        cmd.is_radian = is_radian
        return self._emit_command(cmd)

    def set_angle(self, id, angle, speed, is_radian=False):
        data = [id, angle, speed]
        cmd = COMMANDS[Command.WRITE_ANGLE]
        cmd.is_radian = is_radian
        return self._emit_command(cmd, data)

    def set_angles(self, angles, speed, is_radian=False):
        data = [*angles, speed]
        cmd = COMMANDS[Command.WRITE_ANGLES]
        cmd.is_radian = is_radian
        return self._emit_command(cmd, data)

    def get_coords(self):
        return self._emit_command(COMMANDS[Command.GET_COORDS])

    def is_moving(self):
        return self._emit_command(COMMANDS[Command.IS_MOVING])

    def set_servo_calibration(self, joint_no):
        data = [joint_no - 1]
        return self._emit_command(
            COMMANDS[Command.SET_SERVO_CALIBRATION], data
        )

    def set_led(self, rgb):
        return self._emit_command(
            COMMANDS[Command.SET_LED], bytes.fromhex(rgb)
        )

    def _emit_command(self, cmd, data=None):
        data = cmd.prepare_data(data)
        b = self._get_bytes(cmd, data)
        print("Sending:")
        print(b)
        self._serial.write(b)
        self._serial.flush()
        time.sleep(0.05)
        if cmd.has_reply():
            if self._serial.inWaiting() > 0:
                received = self._serial.read(self._serial.inWaiting())
                return cmd.parse(received)
            else:
                raise IllegalReplyError("No reply is found")
        return None

    def _get_bytes(self, cmd, data):
        return bytes(
            [
                Frame.HEADER,
                Frame.HEADER,
                len(data) + 2,
                cmd.id(),
                *data,
                Frame.FOOTER,
            ]
        )
