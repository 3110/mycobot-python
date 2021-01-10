from abc import ABCMeta, abstractmethod
import serial
import struct
import time


FRAME_HEADER = 0xFE
FRAME_FOOTER = 0xFA

GET_ROBOT_VERSION = 0x01
GET_SYSTEM_VERSION = 0x02
POWER_ON = 0x10
POWER_OFF = 0x11
IS_POWERED_ON = 0x12
SET_FREE_MOVE = 0x13
GET_ANGLES = 0x20
SET_ANGLES = 0x22
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
        return struct.pack(">h", data)

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
        return data[pos] == FRAME_HEADER and data[pos + 1] == FRAME_HEADER

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
        return GET_ROBOT_VERSION


class GetSystemVersion(AbstractCommandWithInt8Reply):
    def id(self):
        return GET_SYSTEM_VERSION


class PowerOn(AbstractCommandWithoutReply):
    def id(self):
        return POWER_ON


class PowerOff(AbstractCommandWithoutReply):
    def id(self):
        return POWER_OFF


class IsPoweredOn(AbstractCommandWithBoolReply):
    def id(self):
        return IS_POWERED_ON


class SetFreeMove(AbstractCommandWithoutReply):
    def id(self):
        return SET_FREE_MOVE


class GetAngles(AbstractCommandWithJointReply):
    def __init__(self, is_radian=False):
        self.is_radian = is_radian

    def id(self):
        return GET_ANGLES

    def parse_value(self, v):
        if self.is_radian:
            return super().parse_value(v) / 1000
        else:
            return super().parse_value(v) * (18 / 314)


class SetAngles(AbstractCommandWithoutReply):
    def __init__(self, is_radian=False):
        self.is_radian = is_radian

    def id(self):
        return SET_ANGLES

    def prepare_data(self, data):
        prepared = []
        data = super().prepare_data(data)
        for v in data[:-1]:
            d = v * 1000 if self.is_radian else v * (314 / 18)
            prepared.extend(list(self.encode_int16(round(d))))
        prepared.append(data[-1])  # speed
        return prepared


class GetCoords(AbstractCommandWithJointReply):
    def id(self):
        return GET_COORDS

    def parse_value(self, v):
        return super().parse_value(v) / 10


class SetServoCalibration(AbstractCommandWithoutReply):
    def id(self):
        return SET_SERVO_CALIBRATION


class SetLED(AbstractCommandWithoutReply):
    def id(self):
        return SET_LED


class IsMoving(AbstractCommandWithBoolReply):
    def id(self):
        return IS_MOVING


COMMANDS = {
    GET_ROBOT_VERSION: GetRobotVersion(),
    GET_SYSTEM_VERSION: GetSystemVersion(),
    POWER_ON: PowerOn(),
    POWER_OFF: PowerOff(),
    IS_POWERED_ON: IsPoweredOn(),
    SET_FREE_MOVE: SetFreeMove(),
    GET_ANGLES: GetAngles(),
    SET_ANGLES: SetAngles(),
    GET_COORDS: GetCoords(),
    IS_MOVING: IsMoving(),
    SET_SERVO_CALIBRATION: SetServoCalibration(),
    SET_LED: SetLED(),
}


class MyCobot:
    def __init__(self, port, baudrate=115200, timeout=0.1):
        self._serial = serial.Serial(port, baudrate, timeout=timeout)

    def get_robot_version(self):
        return self._emit_command(COMMANDS[GET_ROBOT_VERSION])

    def get_system_version(self):
        return self._emit_command(COMMANDS[GET_SYSTEM_VERSION])

    def power_on(self):
        return self._emit_command(COMMANDS[POWER_ON])

    def power_off(self):
        return self._emit_command(COMMANDS[POWER_OFF])

    def is_powered_on(self):
        return self._emit_command(COMMANDS[IS_POWERED_ON])

    def set_free_move(self):
        return self._emit_command(COMMANDS[SET_FREE_MOVE])

    def get_angles(self, is_radian=False):
        cmd = COMMANDS[GET_ANGLES]
        cmd.is_radian = is_radian
        return self._emit_command(cmd)

    def set_angles(self, degrees, speed, is_radian=False):
        data = [*degrees, speed]
        cmd = COMMANDS[SET_ANGLES]
        cmd.is_radian = is_radian
        return self._emit_command(cmd, data)

    def get_coords(self):
        return self._emit_command(COMMANDS[GET_COORDS])

    def is_moving(self):
        return self._emit_command(COMMANDS[IS_MOVING])

    def set_servo_calibration(self, joint_no):
        data = [joint_no - 1]
        return self._emit_command(COMMANDS[SET_SERVO_CALIBRATION], data)

    def set_led(self, rgb):
        return self._emit_command(COMMANDS[SET_LED], bytes.fromhex(rgb))

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
                FRAME_HEADER,
                FRAME_HEADER,
                len(data) + 2,
                cmd.id(),
                *data,
                FRAME_FOOTER,
            ]
        )
