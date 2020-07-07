import sys, os, datetime, copy

robot = [None] * 128
human = [None] * 128
steps = 0
emergency = False

for i in xrange(128):
    robot[i] = [0.0] * 128
    human[i] = [0.0] * 128

def belongs_to_robot(robot, human, x, y):
    if 60 <= x <= 74 and 29 <= y <= 66:
        return True # Fixed robot area in the center
    if robot[x][y] > 0:
        return True # The pixel itself already belongs to the robot
    for dx in xrange(-2, +3):
        for dy in xrange(-2, +3):
            if 0 <= x + dx < 128 and 0 <= y + dy < 128 and robot[x + dx][y + dy] > 0.5:
                return True
    return False

def surveillance_callback(image_msg):
    global robot, human, steps, emergency

    robot_new = copy.deepcopy(robot)
    human_new = copy.deepcopy(human)
    change_count = 0

    for y in xrange(0, 128):
        for x in xrange(0, 128):
            neutral = False
            if 5 <= x <= 92 and 33 <= y <= 56:
                neutral = True # Conveyor belt
            if 90 <= x <= 106 and 60 <= y <= 76:
                neutral = True  # Sink right
            # if 17 <= x <= 92 and 33 <= y <= 56:
            #     continue  # TODO: Sink middle
            # if 17 <= x <= 92 and 33 <= y <= 56:
            #     continue  # TODO: Sink left

            value = ord(image_msg.data[(127-y)*128*3 + x*3])

            if value > 127:  # definitively new pixels
                if belongs_to_robot(robot_new, human_new, x, y):
                    robot_new[x][y] = min(robot_new[x][y] + 5.0, 8.0)
                    human_new[x][y] = 0.0
                elif not neutral:
                    human_new[x][y] = min(human_new[x][y] + 5.0, 8.0)
                    robot_new[x][y] = 0.0
            elif value < 127:  # definitively removed pixels
                robot_new[x][y] = 0.0
                human_new[x][y] = 0.0

    # Check again backwards to catch human/robot misdetection
    for y in xrange(0, 128):
        y = 127 - y
        for x in xrange(0, 128):
            x = 127 - x
            if human_new[x][y] > 0 and belongs_to_robot(robot_new, human_new, x, y):
                robot_new[x][y] = min(robot_new[x][y] + 5.0, 8.0)
                human_new[x][y] = 0.0
                if human[x][y] > 0:
                    change_count += 1
            if human_new[x][y] > 0:
                human_new[x][y] = max(0, human_new[x][y] - 0.5)
                if human_new[x][y] != 0:
                    found = False
                    for dx in xrange(-1, 2):
                        for dy in xrange(-1, 2):
                            if dx == dy == 0:
                                continue
                            if 0 <= x + dx < 128 and 0 <= y + dy < 128 and human_new[x + dx][y + dy] > 0:
                                found = True
                    if not found:
                        human_new[x][y] = 0
            robot_new[x][y] = max(0, robot_new[x][y] - 0.5)

    robot = robot_new
    human = human_new

    sys.stdout.write(chr(27) + 'c')
    if change_count > 25 and steps > 10:
        emergency = True
    steps += 1

    if os.getenv("SURVEILLANCE_DEBUG") == "1":
        s = ""
        for y in xrange(0, 128):
            if y % 2 != 0:
                continue
            s += ("  " if y < 10 else " " if y < 100 else "") + str(y)
            for x in xrange(0, 128):
                if robot[x][y] > 4 or robot[x][y+1] > 4:
                    s += "R"
                elif robot[x][y] > 0 or robot[x][y+1] > 0:
                    s += "r"
                elif human[x][y] > 4 or human[x][y+1] > 4:
                    s += "!"
                elif human[x][y] > 0 or human[x][y+1] > 0:
                    s += "."
                else:
                    s += " "
            s += "\n"
        print("Movement Surveillance Data: " + str(image_msg.width) + "x" + str(image_msg.height) + " (" + str(len(image_msg.data)) + " bytes, from " + str(datetime.datetime.now()) + ") - emergency: " + str(emergency))
        print("   0         1         2         3         4         5         6         7         8         9         10        11        12")
        print("   01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567")
        sys.stdout.write(s)
        sys.stdout.flush()
