from pymycobot.mycobot import MyCobot, GripperState


INITIAL_GRIPPER_POSITION = 248


def gripper_test(mc):
    print("Start check IO part of API")
    print("")

    flag = mc.is_gripper_moving()
    print("Is gripper moving: {}".format(flag))
    time.sleep(1)

    print("Set the current position to zero")
    mc.set_gripper_ini()
    time.sleep(2)

    print("")
    mc.set_gripper_value(INITIAL_GRIPPER_POSITION - 15, 50)
    time.sleep(2)

    print("")
    mc.set_gripper_value(INITIAL_GRIPPER_POSITION - 15, 50)
    time.sleep(2)

    print("")
    mc.set_gripper_state(GripperState.OPEN, 70)
    time.sleep(2)

    print("")
    mc.set_gripper_state(GripperState.CLOSE, 70)
    time.sleep(2)

    print("")
    print(mc.get_gripper_value())


if __name__ == "__main__":
    import argparse
    import time

    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", help="Serial Port")
    parser.add_argument("--debug", action="store_true", help="Debug")
    args = parser.parse_args()

    mycobot = MyCobot(args.port, debug=args.debug)
    gripper_test(mycobot)
