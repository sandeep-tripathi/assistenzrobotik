import sys, os, datetime

robot = [None] * 128
human = [None] * 128

for i in xrange(128):
    robot[i] = [0.0] * 128
    human[i] = [0.0] * 128

def belongs_to_robot(x, y):
    global robot, human
    if 60 <= x <= 74 and 29 <= y <= 66:
        return True # Fixed robot area in the center
    if robot[x][y] > 0:
        return True # The pixel itself already belongs to the robot
    if (x < 127 and robot[x+1][y]) > 0 or (y < 127 and robot[x][y+1] > 0) or (x > 0 and robot[x-1][y]) > 0 or (y > 0 and robot[x][y-1] > 0):
        return True # A directly adjacent pixel belongs to the robot
    if (x < 127 and y < 127 and robot[x+1][y+1]) > 0 or (x > 0 and y < 127 and robot[x-1][y+1] > 0) or (x > 0 and y > 0 and robot[x-1][y-1]) > 0 or (x < 127 and y > 0 and robot[x+1][y-1] > 0):
        return True # A diagonally adjacent pixel belongs to the robot
    if \
            (x > 1 and y > 1 and robot[x-2][y-2] > 1) or \
            (x > 1 and y > 0 and robot[x-2][y-1] > 1) or \
            (x > 1 and robot[x-2][y] > 1) or \
            (x > 1 and y < 127 and robot[x-2][y+1] > 1) or \
            (x > 1 and y < 126 and robot[x-2][y+2] > 1) or \
            (x > 0 and y < 126 and robot[x-1][y+2] > 1) or \
            (y < 126 and robot[x][y+2] > 1) or \
            (x < 127 and y < 126 and robot[x+1][y+2] > 1) or \
            (x < 126 and y < 126 and robot[x+2][y+2] > 1) or \
            (x < 126 and y < 127 and robot[x+2][y+1] > 1) or \
            (x < 126 and robot[x+2][y] > 1) or \
            (x < 126 and y > 0 and robot[x+2][y-1] > 1) or \
            (x < 126 and y > 1 and robot[x+2][y-2] > 1) or \
            (x < 127 and y > 1 and robot[x+1][y-2] > 1) or \
            (y > 1 and robot[x][y-2] > 1) or \
            (x > 0 and y > 1 and robot[x-1][y-2] > 1):
        return True # A pixel two further definitively belongs to the robot

def surveillance_callback(image_msg):
    global robot, human

    # for y in range(0, 128):
    #     for x in range(0, 128):
    #         n = ord(image_msg.data[y*128+x])
    #         sys.stdout.write(". " if n == 127 else "0 " if n < 127 else "1 ")
    #     sys.stdout.write("\n")

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
                if belongs_to_robot(x, y):
                    robot[x][y] = min(robot[x][y] + 5.0, 8.0)
                    human[x][y] = 0.0
                elif not neutral:
                    human[x][y] = min(human[x][y] + 5.0, 8.0)
                    robot[x][y] = 0.0
            elif value < 127:  # definitively removed pixels
                robot[x][y] = 0.0
                human[x][y] = 0.0

    # Check again backwards to catch human/robot misdetection
    for y in xrange(0, 128):
        y = 127 - y
        for x in xrange(0, 128):
            x = 127 - x
            if human[x][y] > 0 and belongs_to_robot(x, y):
                robot[x][y] = min(robot[x][y] + 5.0, 8.0)
                human[x][y] = 0.0
            robot[x][y] = max(0, robot[x][y] - 0.5)
            human[x][y] = max(0, human[x][y] - 0.5)

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
        sys.stdout.write(chr(27) + 'c')
        print("Movement Surveillance Data: " + str(image_msg.width) + "x" + str(image_msg.height) + " (" + str(len(image_msg.data)) + " bytes, from " + str(datetime.datetime.now()) + ")")
        print("   0         1         2         3         4         5         6         7         8         9         10        11        12")
        print("   01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567")
        sys.stdout.write(s)
        sys.stdout.flush()
