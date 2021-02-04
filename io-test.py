import time
from pymycobot.mycobot import MyCobot


def io_test(mc):
    print("Start check IO part of API")
    print()

    mc.set_pin_mode(19, 1)
    time.sleep(1)
    mc.set_digital_output(19, 1)
    time.sleep(5)
    mc.set_digital_output(19, 0)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", help="Serial Port")
    parser.add_argument("--debug", action="store_true", help="Debug")
    args = parser.parse_args()

    mycobot = MyCobot(args.port, debug=args.debug)
    io_test(mycobot)