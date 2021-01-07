import argparse
import time
from pymycobot.mycobot import MyCobot

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port", help="Serial Port")
args = parser.parse_args()

mycobot = MyCobot(args.port)
mycobot.power_on()
mycobot.set_led("00f000")
mycobot.set_angles([0, 0, 0, 0, 0, 0], 50)
time.sleep(5)
mycobot.set_angles([0, 140, 160, 160, -95, 0], 50)
time.sleep(5)
mycobot.set_free_mode()
