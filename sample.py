import argparse
import time
from pymycobot.mycobot import MyCobot

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port", help="Serial Port")
parser.add_argument("--debug", action="store_true", help="Debug")
args = parser.parse_args()

mycobot = MyCobot(args.port, debug=args.debug)
mycobot.power_on()
mycobot.set_led("00f000")
mycobot.set_angles([0, 0, 0, 0, 0, 0], 50)
time.sleep(5)
mycobot.set_angles([0, 140, 154, 156, -84, 6], 50)
time.sleep(5)
mycobot.set_free_move()
